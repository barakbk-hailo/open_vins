/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <chrono>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/version.h>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

// rosbag2 renamed SerializedBagMessage::time_stamp → recv_timestamp around Jazzy (rclcpp 28+).
// Keep a single-source accessor so the rest of this file reads the same on both distros.
namespace {
inline rcutils_time_point_value_t bag_msg_time(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &m) {
#if RCLCPP_VERSION_GTE(28, 0, 0)
  return m->recv_timestamp;
#else
  return m->time_stamp;
#endif
}
} // namespace

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "ros/ROS2Visualizer.h"
#include "utils/dataset_reader.h"

using namespace ov_msckf;

// Deserialize a raw CDR bag message into a ROS2 message type
template <typename MsgT>
std::shared_ptr<MsgT> deserialize_msg(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &raw) {
  rclcpp::SerializedMessage serialized_msg(*raw->serialized_data);
  rclcpp::Serialization<MsgT> serializer;
  auto msg = std::make_shared<MsgT>();
  serializer.deserialize_message(&serialized_msg, msg.get());
  return msg;
}

// Lightweight bag entry: defer deserialization until the message is actually needed
struct BagEntry {
  std::string topic_name;
  int64_t timestamp_ns;
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> raw;
};

int main(int argc, char **argv) {

  // Launch our ROS2 node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("ros2_serial_msckf", options);

  // Config path (can be overridden via command-line arg or launch parameter)
  std::string config_path = "unset_path_to_config.yaml";
  if (argc > 1) {
    config_path = argv[1];
  }
  node->get_parameter("config_path", config_path);

  // Load the config
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
  parser->set_node(node);

  // Verbosity
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  // Create our VIO system
  VioManagerOptions params;
  params.print_and_load(parser);
  // Serial mode: block inside each callback until the VIO update completes
  // (use_multi_threading_subs=true would detach the update thread and let the
  // main loop rush ahead, defeating the purpose of serial processing)
  params.use_multi_threading_subs = false;
  auto sys = std::make_shared<VioManager>(params);
  // Create visualizer but do NOT call setup_subscribers — we feed it directly
  auto viz = std::make_shared<ROS2Visualizer>(node, sys);

  // Ensure we read in all parameters required
  if (!parser->successful()) {
    PRINT_ERROR(RED "[SERIAL]: unable to parse all parameters, please fix\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // IMU topic — from config, overridable via node parameter
  std::string topic_imu = "/imu0";
  node->get_parameter("topic_imu", topic_imu);
  parser->parse_external("relative_config_imu", "imu0", "rostopic", topic_imu);
  PRINT_DEBUG("[SERIAL]: imu topic: %s\n", topic_imu.c_str());

  // Camera topics — one per camera, from config, overridable via node parameter
  std::vector<std::string> topic_cameras;
  for (int i = 0; i < params.state_options.num_cameras; i++) {
    std::string cam_topic = "/cam" + std::to_string(i) + "/image_raw";
    node->get_parameter("topic_camera" + std::to_string(i), cam_topic);
    parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "rostopic", cam_topic);
    topic_cameras.push_back(cam_topic);
    PRINT_DEBUG("[SERIAL]: cam%d topic: %s\n", i, cam_topic.c_str());
  }

  // Bag path and time window
  std::string path_bag = "";
  node->get_parameter("path_bag", path_bag);
  PRINT_DEBUG("[SERIAL]: bag path: %s\n", path_bag.c_str());

  double bag_start = 0.0;
  double bag_durr = -1.0;
  node->get_parameter("bag_start", bag_start);
  node->get_parameter("bag_durr", bag_durr);
  PRINT_DEBUG("[SERIAL]: bag_start: %.1f\n", bag_start);
  PRINT_DEBUG("[SERIAL]: bag_durr:  %.1f\n", bag_durr);

  // Ground truth file (optional, ASL CSV format)
  std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;
  std::string path_gt = "";
  node->get_parameter("path_gt", path_gt);
  if (!path_gt.empty()) {
    ov_core::DatasetReader::load_gt_file(path_gt, gt_states);
    PRINT_DEBUG("[SERIAL]: gt file: %s\n", path_gt.c_str());
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Open the ROS2 bag
  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = path_bag;
  storage_options.storage_id = ""; // auto-detect from metadata.yaml
  try {
    reader.open(storage_options);
  } catch (const std::exception &e) {
    PRINT_ERROR(RED "[SERIAL]: failed to open bag '%s': %s\n" RESET, path_bag.c_str(), e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  // Compute time window in nanoseconds
  auto meta = reader.get_metadata();
  int64_t bag_t0_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(meta.starting_time.time_since_epoch()).count();
  int64_t bag_end_ns = bag_t0_ns + std::chrono::duration_cast<std::chrono::nanoseconds>(meta.duration).count();
  int64_t time_init_ns = bag_t0_ns + static_cast<int64_t>(bag_start * 1e9);
  int64_t time_finish_ns = (bag_durr < 0) ? bag_end_ns : time_init_ns + static_cast<int64_t>(bag_durr * 1e9);
  PRINT_DEBUG("[SERIAL]: time start  = %.6f\n", time_init_ns * 1e-9);
  PRINT_DEBUG("[SERIAL]: time finish = %.6f\n", time_finish_ns * 1e-9);

  // Seek to the start of the window and filter to only our topics (reduces I/O)
  reader.seek(time_init_ns);
  rosbag2_storage::StorageFilter filter;
  filter.topics.push_back(topic_imu);
  for (const auto &t : topic_cameras) {
    filter.topics.push_back(t);
  }
  reader.set_filter(filter);

  // Collect all messages in the window into memory (raw CDR bytes, not yet deserialized).
  // We need random-access for the stereo look-ahead sync algorithm.
  std::vector<BagEntry> msgs;
  int64_t max_camera_time_ns = -1;
  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    if (bag_msg_time(bag_msg) > time_finish_ns)
      break;
    bool is_imu = (bag_msg->topic_name == topic_imu);
    bool is_cam = false;
    for (const auto &t : topic_cameras) {
      if (bag_msg->topic_name == t) {
        is_cam = true;
        break;
      }
    }
    if (is_imu || is_cam) {
      msgs.push_back({bag_msg->topic_name, bag_msg_time(bag_msg), bag_msg});
      if (is_cam) {
        max_camera_time_ns = std::max(max_camera_time_ns, bag_msg_time(bag_msg));
      }
    }
  }
  PRINT_DEBUG("[SERIAL]: total of %zu messages collected\n", msgs.size());

  if (msgs.empty()) {
    PRINT_ERROR(RED "[SERIAL]: no messages in specified time window — check path_bag and bag_start/bag_durr\n" RESET);
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Process messages serially — same algorithm as ros1_serial_msckf.cpp
  std::set<int> used_index;
  for (int m = 0; m < (int)msgs.size(); m++) {

    // Stop at end of window or once we've passed the last camera frame
    if (msgs[m].timestamp_ns > time_finish_ns)
      break;
    if (max_camera_time_ns >= 0 && msgs[m].timestamp_ns > max_camera_time_ns)
      break;

    // Skip stereo partners already consumed by a previous camera message
    if (used_index.count(m)) {
      used_index.erase(m);
      continue;
    }

    // ---- IMU ----
    if (msgs[m].topic_name == topic_imu) {
      viz->callback_inertial(deserialize_msg<sensor_msgs::msg::Imu>(msgs[m].raw));
      continue;
    }

    // ---- Camera ----
    for (int cam_id = 0; cam_id < params.state_options.num_cameras; cam_id++) {
      if (msgs[m].topic_name != topic_cameras[cam_id])
        continue;

      // Build cam_id -> message-index map; for stereo, look ahead for the matching camera
      std::map<int, int> camid_to_idx;
      double meas_time_sec = msgs[m].timestamp_ns * 1e-9;
      camid_to_idx[cam_id] = m;

      for (int cam_idt = 0; cam_idt < params.state_options.num_cameras; cam_idt++) {
        if (cam_idt == cam_id)
          continue;
        int found = -1;
        for (int mt = m; mt < (int)msgs.size(); mt++) {
          if (msgs[mt].topic_name != topic_cameras[cam_idt])
            continue;
          if (std::abs(msgs[mt].timestamp_ns * 1e-9 - meas_time_sec) < 0.02)
            found = mt;
          break; // stop at the first candidate on this camera topic
        }
        if (found != -1)
          camid_to_idx[cam_idt] = found;
      }

      // Optionally initialise with ground truth
      Eigen::Matrix<double, 17, 1> imustate;
      if (!gt_states.empty() && !sys->initialized() && ov_core::DatasetReader::get_gt_state(meas_time_sec, imustate, gt_states)) {
        sys->initialize_with_gt(imustate);
      }

      // Dispatch to the appropriate callback
      if (params.state_options.num_cameras == 1) {
        if ((int)camid_to_idx.size() == 1) {
          viz->callback_monocular(deserialize_msg<sensor_msgs::msg::Image>(msgs[camid_to_idx.at(0)].raw), 0);
        }
      } else if (params.state_options.num_cameras == 2) {
        if ((int)camid_to_idx.size() == 2) {
          used_index.insert(camid_to_idx.at(0));
          used_index.insert(camid_to_idx.at(1));
          viz->callback_stereo(deserialize_msg<sensor_msgs::msg::Image>(msgs[camid_to_idx.at(0)].raw),
                               deserialize_msg<sensor_msgs::msg::Image>(msgs[camid_to_idx.at(1)].raw), 0, 1);
        } else {
          PRINT_DEBUG(YELLOW "[SERIAL]: no stereo pair at %.2f s into bag — skipping\n" RESET, meas_time_sec - time_init_ns * 1e-9);
        }
      } else {
        PRINT_ERROR(RED "[SERIAL]: only 1 or 2 cameras are supported\n" RESET);
        rclcpp::shutdown();
        return EXIT_FAILURE;
      }
      break; // matched this camera; done with this message
    }
  }

  // Final visualisation and clean shutdown
  viz->visualize_final();
  rclcpp::spin_some(node); // flush any pending publisher output (e.g. to rviz2)
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
