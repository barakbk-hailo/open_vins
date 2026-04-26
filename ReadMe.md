# OpenVINS (Hailo Fork)

> **This is a fork of [rpng/open_vins](https://github.com/rpng/open_vins)** maintained by Hailo for benchmarking VIO performance on embedded platforms (Raspberry Pi 5).
> The upstream README follows below. See [Fork Changes](#fork-changes) for what was added.

[![ROS 1 Workflow](https://github.com/rpng/open_vins/actions/workflows/build_ros1.yml/badge.svg)](https://github.com/rpng/open_vins/actions/workflows/build_ros1.yml)
[![ROS 2 Workflow](https://github.com/rpng/open_vins/actions/workflows/build_ros2.yml/badge.svg)](https://github.com/rpng/open_vins/actions/workflows/build_ros2.yml)
[![ROS Free Workflow](https://github.com/rpng/open_vins/actions/workflows/build.yml/badge.svg)](https://github.com/rpng/open_vins/actions/workflows/build.yml)

## Fork Changes

The following changes were made on top of upstream OpenVINS to support deterministic benchmarking and embedded deployment:

### 1. ROS 2 Serial (Offline) VIO Node
**`ov_msckf/src/ros2_serial_msckf.cpp`** — A new node that reads a ROS 2 bag file and processes frames sequentially (one at a time, blocking updates). Unlike the default subscriber-based node, this guarantees deterministic results regardless of CPU speed — no frames are dropped due to processing delays. This is critical for fair benchmarking across different hardware (x86 baseline vs. RPi5).

Key features:
- Reads bags via `rosbag2_cpp`, filters by topic and time window
- Stereo frame synchronization (20 ms tolerance)
- Optional ground-truth initialization from ASL-format CSV
- Single-threaded, blocking VIO updates (`use_multi_threading_subs=false`)

**`ov_msckf/launch/serial.launch.py`** — Launch file for the serial node with parameters for bag path, duration, topics, and optional RViz display. Configured with `on_exit=Shutdown()` so the launch process exits cleanly when the bag is fully processed (important for automated benchmark loops).

### 2. Customizable RPE Segment Lengths
**`ov_eval/src/error_singlerun.cpp`** — The Relative Pose Error (RPE) evaluation tool now accepts custom segment lengths as command-line arguments:
```
./error_singlerun <align_mode> <gt_file> <est_file> [seg1] [seg2] ... [segN]
```
Default segments remain `{8, 16, 24, 32, 40}` seconds when no arguments are given.

### 3. Persistent Worker Thread (Subscribe Mode)
**`ov_msckf/src/ros/ROS2Visualizer.{h,cpp}`** — Replaces upstream's per-frame `std::thread(...).detach()` dispatch with a single long-lived worker thread that drains a bounded queue. Eliminates a TOCTOU race, dangling-reference UB on captured stack data, and non-deterministic IMU callback triggering. Reduces subscribe-mode overhead from ~2× serial to ~1× serial on x86 and keeps SLAM health stable on RPi5. Details in the companion workspace's `docs/determinism.md` §3.

- Gated by the `multi_threading_subs` YAML knob (default `true` = worker active; `false` = inline dispatch, needed for serial determinism)

### 4. SLAM Recovery (Chi-Squared Gate Relaxation)
When the SLAM feature state drops below 25% of `max_slam`, the chi-squared gate is transiently relaxed by 3× to let more features in and break the empty-state feedback loop that otherwise keeps new SLAM features permanently rejected.

- Gated by the **`slam_chi2_recovery`** YAML knob (default `false`), added to `VioManagerOptions.h` / `VioManager.cpp` / `config/euroc_mav/estimator_config.yaml`. Leave at the default `false` for offline serial replay and the paper Table II/III reproduction — chi2 relaxation interferes with stereo init on dark sequences (MH_05_difficult), and with the knob off all 10 EuRoC sequences × stereo+mono reproduce committed `results/{stereo,mono}/estimate_*.txt` bit-for-bit. Set to `true` for subscribe-mode deployment at >1× realtime on resource-constrained hardware: V1_03_difficult @ rate 2.0 shows worst-case ATE 3.7 m with `true` vs >50 m collapse with `false` in 2/3 runs — the safety net is meaningful under filter overload.

### 5. Three-Clock Timing + Feature Count Recording
`VioManager::track_image_and_update()` records per-frame metrics to three separate CSVs plus a per-frame feature-count CSV. Useful for disentangling wall-clock jitter, process-CPU cost (across all threads), and thread-CPU cost (the VIO thread alone).

- **`record_timing_information`** / `record_timing_filepath` — wall clock (existing upstream)
- **`record_timing_cpu_time`** / `record_timing_cpu_filepath` — process CPU time (fork addition)
- **`record_timing_thread_time`** / `record_timing_thread_filepath` — thread CPU time (fork addition)
- **`record_feature_counts`** / `record_feature_counts_filepath` — per-frame SLAM/MSCKF counts, clone count (fork addition)

### 6. Stereo Sync Bound for Subscribe Mode
**`ov_msckf/src/ros/ROS2Visualizer.cpp`** — `setMaxIntervalDuration(0.02)` on the stereo `Synchronizer` caps cross-camera pair latency at 20 ms. Prevents queue-state race conditions from pairing mismatched frames under scheduler jitter. Zero frame drops on EuRoC's hardware-synced stereo (confirmed at 10 × 2800 frames). The only RPi5 subscribe-mode intervention with a reproducible, measurable effect on accuracy variance — see `docs/determinism.md` §"Recommendation for RPi5 deployment".

### 7. Dual-Distro Support (ROS 2 Humble + Jazzy)
**`ov_msckf/src/ros/ROS2Visualizer.h`**, **`ov_msckf/src/ros/ROSVisualizerHelper.h`**, **`ov_msckf/src/ros2_serial_msckf.cpp`** — `__has_include()` guards pick the correct `image_transport` / `tf2_geometry_msgs` / `cv_bridge` header (`.hpp` on Jazzy, `.h` where Humble has no `.hpp`). An inline `bag_msg_time()` helper gated on `RCLCPP_VERSION_GTE(28, 0, 0)` picks `recv_timestamp` (Jazzy) vs `time_stamp` (Humble) for rosbag2 `SerializedBagMessage`.

- Verified: `colcon build --packages-select ov_msckf` passes on Ubuntu 22.04 / ROS 2 Humble (`rclcpp` 16.x) and Ubuntu 24.04 / ROS 2 Jazzy (`rclcpp` 28.x).

### 8. Docker Images for Embedded Deployment
- **`Dockerfile_ros2_humble_jammy`** — Ubuntu 22.04 + ROS 2 Humble, targeting RPi5 / Debian Trixie. Default-clones the `sync-max-interval-20ms` branch so the 20 ms bound ships enabled.
- **`Dockerfile_ros2_jazzy_noble`** — Ubuntu 24.04 + ROS 2 Jazzy. Works with the `__has_include` / `RCLCPP_VERSION_GTE` guards above.

Both use ccache and limit parallel builds to 2 workers for memory-constrained boards.

### 9. RViz Configuration Updates
- **`ov_msckf/launch/display.rviz`** — Updated plugin names from ROS 1 to ROS 2, simplified for Intel iGPU
- **`ov_msckf/launch/display_minimal.rviz`** — New minimal config with only essential displays (Grid, TF, Paths, Points)

### Summary of Changed Files

| File | Status | Description |
|------|--------|-------------|
| `ov_msckf/src/ros2_serial_msckf.cpp` | Added | Serial (offline) VIO node; dual-distro `bag_msg_time()` |
| `ov_msckf/launch/serial.launch.py` | Added | Launch config for serial node |
| `ov_msckf/launch/display_minimal.rviz` | Added | Minimal RViz config |
| `Dockerfile_ros2_humble_jammy` | Added | Docker for Humble + RPi5; clones `sync-max-interval-20ms` by default |
| `Dockerfile_ros2_jazzy_noble` | Added | Docker for Jazzy (Ubuntu 24.04) |
| `ov_msckf/src/ros/ROS2Visualizer.{h,cpp}` | Modified | Persistent worker thread; 20 ms stereo sync bound; `__has_include` guards |
| `ov_msckf/src/ros/ROSVisualizerHelper.h` | Modified | `__has_include` guards for `tf2_geometry_msgs` |
| `ov_msckf/src/core/VioManager.{h,cpp}` | Modified | SLAM recovery gate; 3-clock timing; feature count recording |
| `ov_msckf/src/core/VioManagerOptions.h` | Modified | `slam_chi2_recovery`, `multi_threading_subs`, timing/feature-count knobs |
| `config/euroc_mav/estimator_config.yaml` | Modified | New fork knobs exposed with defaults |
| `ov_msckf/cmake/ROS2.cmake` | Modified | Added rosbag2_cpp dep and serial target |
| `ov_msckf/package.xml` | Modified | Added rosbag2_cpp dependency |
| `ov_msckf/launch/display.rviz` | Modified | ROS 2 plugin names |
| `ov_eval/src/error_singlerun.cpp` | Modified | Customizable RPE segments |

---

Welcome to the OpenVINS project!
The OpenVINS project houses some core computer vision code along with a state-of-the art filter-based visual-inertial
estimator. The core filter is an [Extended Kalman filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) which
fuses inertial information with sparse visual feature tracks. These visual feature tracks are fused leveraging
the [Multi-State Constraint Kalman Filter (MSCKF)](https://ieeexplore.ieee.org/document/4209642) sliding window
formulation which allows for 3D features to update the state estimate without directly estimating the feature states in
the filter. Inspired by graph-based optimization systems, the included filter has modularity allowing for convenient
covariance management with a proper type-based state system. Please take a look at the feature list below for full
details on what the system supports.

* Github project page - https://github.com/rpng/open_vins
* Documentation - https://docs.openvins.com/
* Getting started guide - https://docs.openvins.com/getting-started.html
* Publication reference - https://pgeneva.com/downloads/papers/Geneva2020ICRA.pdf

## News / Events

* **May 11, 2023** - Inertial intrinsic support released as part of v2.7 along with a few bug fixes and improvements to stereo KLT tracking. Please check out the [release page](https://github.com/rpng/open_vins/releases/tag/v2.7) for details.
* **April 15, 2023** - Minor update to v2.6.3 to support incremental feature triangulation of active features for downstream applications, faster zero-velocity update, small bug fixes, some example realsense configurations, and cached fast state prediction. Please check out the [release page](https://github.com/rpng/open_vins/releases/tag/v2.6.3) for details.
* **April 3, 2023** - We have released a monocular plane-aided VINS, termed [ov_plane](https://github.com/rpng/ov_plane), which leverages the OpenVINS project. Both now support the released [Indoor AR Table](https://github.com/rpng/ar_table_dataset) dataset.
* **July 14, 2022** - Improved feature extraction logic for >100hz tracking, some bug fixes and updated scripts. See v2.6.1 [PR#259](https://github.com/rpng/open_vins/pull/259) and v2.6.2 [PR#264](https://github.com/rpng/open_vins/pull/264).
* **March 14, 2022** - Initial dynamic initialization open sourcing, asynchronous subscription to inertial readings and publishing of odometry, support for lower frequency feature tracking. See v2.6 [PR#232](https://github.com/rpng/open_vins/pull/232) for details.
* **December 13, 2021** - New YAML configuration system, ROS2 support, Docker images, robust static initialization based on disparity, internal logging system to reduce verbosity, image transport publishers, dynamic number of features support, and other small fixes. See
  v2.5 [PR#209](https://github.com/rpng/open_vins/pull/209) for details.
* **July 19, 2021** - Camera classes, masking support, alignment utility, and other small fixes. See
  v2.4 [PR#117](https://github.com/rpng/open_vins/pull/186) for details.
* **December 1, 2020** - Released improved memory management, active feature pointcloud publishing, limiting number of
  features in update to bound compute, and other small fixes. See
  v2.3 [PR#117](https://github.com/rpng/open_vins/pull/117) for details.
* **November 18, 2020** - Released groundtruth generation utility package, [vicon2gt](https://github.com/rpng/vicon2gt)
  to enable creation of groundtruth trajectories in a motion capture room for evaulating VIO methods.
* **July 7, 2020** - Released zero velocity update for vehicle applications and direct initialization when standing
  still. See [PR#79](https://github.com/rpng/open_vins/pull/79) for details.
* **May 18, 2020** - Released secondary pose graph example
  repository [ov_secondary](https://github.com/rpng/ov_secondary) based
  on [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion). OpenVINS now publishes marginalized feature
  track, feature 3d position, and first camera intrinsics and extrinsics.
  See [PR#66](https://github.com/rpng/open_vins/pull/66) for details and discussion.
* **April 3, 2020** - Released [v2.0](https://github.com/rpng/open_vins/releases/tag/v2.0) update to the codebase with
  some key refactoring, ros-free building, improved dataset support, and single inverse depth feature representation.
  Please check out the [release page](https://github.com/rpng/open_vins/releases/tag/v2.0) for details.
* **January 21, 2020** - Our paper has been accepted for presentation in [ICRA 2020](https://www.icra2020.org/). We look
  forward to seeing everybody there! We have also added links to a few videos of the system running on different
  datasets.
* **October 23, 2019** - OpenVINS placed first in the [IROS 2019 FPV Drone Racing VIO Competition
  ](http://rpg.ifi.uzh.ch/uzh-fpv.html). We will be giving a short presentation at
  the [workshop](https://wp.nyu.edu/workshopiros2019mav/) at 12:45pm in Macau on November 8th.
* **October 1, 2019** - We will be presenting at the [Visual-Inertial Navigation: Challenges and Applications
  ](http://udel.edu/~ghuang/iros19-vins-workshop/index.html) workshop at [IROS 2019](https://www.iros2019.org/). The
  submitted workshop paper can be found at [this](http://udel.edu/~ghuang/iros19-vins-workshop/papers/06.pdf) link.
* **August 21, 2019** - Open sourced [ov_maplab](https://github.com/rpng/ov_maplab) for interfacing OpenVINS with
  the [maplab](https://github.com/ethz-asl/maplab) library.
* **August 15, 2019** - Initial release of OpenVINS repository and documentation website!

## Project Features

* Sliding window visual-inertial MSCKF
* Modular covariance type system
* Comprehensive documentation and derivations
* Extendable visual-inertial simulator
    * On manifold SE(3) b-spline
    * Arbitrary number of cameras
    * Arbitrary sensor rate
    * Automatic feature generation
* Five different feature representations
    1. Global XYZ
    2. Global inverse depth
    3. Anchored XYZ
    4. Anchored inverse depth
    5. Anchored MSCKF inverse depth
    6. Anchored single inverse depth
* Calibration of sensor intrinsics and extrinsics
    * Camera to IMU transform
    * Camera to IMU time offset
    * Camera intrinsics
    * Inertial intrinsics (including g-sensitivity)
* Environmental SLAM feature
    * OpenCV ARUCO tag SLAM features
    * Sparse feature SLAM features
* Visual tracking support
    * Monocular camera
    * Stereo camera (synchronized)
    * Binocular cameras (synchronized)
    * KLT or descriptor based
    * Masked tracking
* Static and dynamic state initialization
* Zero velocity detection and updates
* Out of the box evaluation on EuRocMav, TUM-VI, UZH-FPV, KAIST Urban and other VIO datasets
* Extensive evaluation suite (ATE, RPE, NEES, RMSE, etc..)

## Codebase Extensions

* **[ov_plane](https://github.com/rpng/ov_plane)** - A real-time monocular visual-inertial odometry (VIO) system which leverages
  environmental planes. At the core it presents an efficient robust monocular-based plane detection algorithm which does
  not require additional sensing modalities such as a stereo, depth camera or neural network. The plane detection and tracking
  algorithm enables real-time regularization of point features to environmental planes which are either maintained in the state
  vector as long-lived planes, or marginalized for efficiency. Planar regularities are applied to both in-state SLAM and
  out-of-state MSCKF point features, enabling long-term point-to-plane loop-closures due to the large spacial volume of planes.

* **[vicon2gt](https://github.com/rpng/vicon2gt)** - This utility was created to generate groundtruth trajectories using
  a motion capture system (e.g. Vicon or OptiTrack) for use in evaluating visual-inertial estimation systems.
  Specifically we calculate the inertial IMU state (full 15 dof) at camera frequency rate and generate a groundtruth
  trajectory similar to those provided by the EurocMav datasets. Performs fusion of inertial and motion capture
  information and estimates all unknown spacial-temporal calibrations between the two sensors.

* **[ov_maplab](https://github.com/rpng/ov_maplab)** - This codebase contains the interface wrapper for exporting
  visual-inertial runs from [OpenVINS](https://github.com/rpng/open_vins) into the ViMap structure taken
  by [maplab](https://github.com/ethz-asl/maplab). The state estimates and raw images are appended to the ViMap as
  OpenVINS runs through a dataset. After completion of the dataset, features are re-extract and triangulate with
  maplab's feature system. This can be used to merge multi-session maps, or to perform a batch optimization after first
  running the data through OpenVINS. Some example have been provided along with a helper script to export trajectories
  into the standard groundtruth format.

* **[ov_secondary](https://github.com/rpng/ov_secondary)** - This is an example secondary thread which provides loop
  closure in a loosely coupled manner for [OpenVINS](https://github.com/rpng/open_vins). This is a modification of the
  code originally developed by the HKUST aerial robotics group and can be found in
  their [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) repository. Here we stress that this is a
  loosely coupled method, thus no information is returned to the estimator to improve the underlying OpenVINS odometry.
  This codebase has been modified in a few key areas including: exposing more loop closure parameters, subscribing to
  camera intrinsics, simplifying configuration such that only topics need to be supplied, and some tweaks to the loop
  closure detection to improve frequency.


## Demo Videos

<a href="http://www.youtube.com/watch?v=KCX51GvYGss">
   <img src="https://raw.githubusercontent.com/rpng/open_vins/master/docs/youtube/KCX51GvYGss.jpg" width="120" height="90" />
</a>
<a href="http://www.youtube.com/watch?v=Lc7VQHngSuQ">
   <img src="https://raw.githubusercontent.com/rpng/open_vins/master/docs/youtube/Lc7VQHngSuQ.jpg" width="120" height="90" />
</a>
<a href="http://www.youtube.com/watch?v=vaia7iPaRW8">
   <img src="https://raw.githubusercontent.com/rpng/open_vins/master/docs/youtube/vaia7iPaRW8.jpg" width="120" height="90" />
</a>
<a href="http://www.youtube.com/watch?v=MCzTF9ye2zw">
   <img src="https://raw.githubusercontent.com/rpng/open_vins/master/docs/youtube/MCzTF9ye2zw.jpg"  width="120" height="90"/>
</a>
<a href="http://www.youtube.com/watch?v=eSQLWcNrx_I">
   <img src="https://raw.githubusercontent.com/rpng/open_vins/master/docs/youtube/eSQLWcNrx_I.jpg" width="120" height="90" />
</a>
<br/>

<a href="http://www.youtube.com/watch?v=187AXuuGNNw">
   <img src="https://raw.githubusercontent.com/rpng/open_vins/master/docs/youtube/187AXuuGNNw.jpg" width="120" height="90" />
</a>
<a href="http://www.youtube.com/watch?v=oUoLlrFryk0">
   <img src="https://raw.githubusercontent.com/rpng/open_vins/master/docs/youtube/oUoLlrFryk0.jpg" width="120" height="90" />
</a>
<a href="http://www.youtube.com/watch?v=ExPIGwORm4E">
   <img src="https://raw.githubusercontent.com/rpng/open_vins/master/docs/youtube/ExPIGwORm4E.jpg" width="120" height="90" />
</a>
<a href="http://www.youtube.com/watch?v=lXHl-qgLGl8">
   <img src="https://raw.githubusercontent.com/rpng/open_vins/master/docs/youtube/lXHl-qgLGl8.jpg" width="120" height="90" />
</a>



## Credit / Licensing

This code was written by the [Robot Perception and Navigation Group (RPNG)](https://sites.udel.edu/robot/) at the
University of Delaware. If you have any issues with the code please open an issue on our github page with relevant
implementation details and references. For researchers that have leveraged or compared to this work, please cite the
following:

```txt
@Conference{Geneva2020ICRA,
  Title      = {{OpenVINS}: A Research Platform for Visual-Inertial Estimation},
  Author     = {Patrick Geneva and Kevin Eckenhoff and Woosik Lee and Yulin Yang and Guoquan Huang},
  Booktitle  = {Proc. of the IEEE International Conference on Robotics and Automation},
  Year       = {2020},
  Address    = {Paris, France},
  Url        = {\url{https://github.com/rpng/open_vins}}
}
```

The codebase and documentation is licensed under the [GNU General Public License v3 (GPL-3)](https://www.gnu.org/licenses/gpl-3.0.txt).
You must preserve the copyright and license notices in your derivative work and make available the complete source code with modifications under the same license ([see this](https://choosealicense.com/licenses/gpl-3.0/); this is not legal advice).


