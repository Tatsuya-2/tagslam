# tagslam

GTSAM-based fiducial-marker SLAM, vendored from
[berndpfrommer/tagslam](https://github.com/berndpfrommer/tagslam) and adapted
for this project's Pure SLAM Mode pipeline.

Upstream is a multi-camera, multi-body factor-graph SLAM. In this project
it is used in a degenerate single-camera, two-body configuration that
consumes `/tag_detections` from Isaac ROS AprilTag and emits the camera
pose as `/tagslam/odom`. No odometry is fed in.

## Project Modifications vs. Upstream

- Subscribes to AprilTag detections through `isaac_ros_apriltag_interfaces`
  (`AprilTagDetectionArray`) instead of `apriltag_msgs`.
- `tagslam.launch.py` remaps the auto-generated body topics to fixed names:
  - `/odom/body_camera_body` → `/tagslam/odom`
  - `/path/body_camera_body` → `/tagslam/path`
- Per-body `publish_tf` flag added (`body.publish_tf: false` in
  `tagslam.yaml`) so TagSLAM does **not** broadcast `map → camera_body` TF.
  `pose_corrector` is the sole owner of `map → drone` TF.
- `nav_msgs/Odometry` and `nav_msgs/Path` publishers use
  `SensorDataQoS` (BEST_EFFORT, KEEP_LAST 1000).
- The `*_from_bag` executables and the `sync_and_detect` node are not
  built in this fork; only `tagslam_node` is compiled (see
  `CMakeLists.txt`). The `launch/sync_and_detect.launch.py` file ships
  unused and will fail because `sync_and_detect_node` is not built.

## Executable & Launch

| Item                              | Value                                          |
| --------------------------------- | ---------------------------------------------- |
| Executable                        | `tagslam_node` (single C++ node `tagslam::TagSLAM`) |
| Project launch entry point        | `launch/tagslam.launch.py`                     |
| Composable component              | `tagslam::TagSLAM` (registered, library installed to global `lib/`) |
| Bringup (real drone)              | `jetson_prod/scripts/bringup-dji-drone.sh` → `start_tagslam()` |
| Bringup (Gazebo)                  | `jetson_prod/scripts/bringup-dji-drone-gazebo.sh` → `start_tagslam()` (runs inside Docker) |

### Launch arguments (`tagslam.launch.py`)

| Arg                    | Default          | Description                                         |
| ---------------------- | ---------------- | --------------------------------------------------- |
| `cameras`              | `cameras.yaml`   | Path to camera intrinsics / topic config            |
| `camera_poses`         | `camera_poses.yaml` | Path to camera extrinsics (cam0 → rig body)      |
| `tagslam_config`       | `tagslam.yaml`   | Path to SLAM parameters + body/tag definitions      |
| `use_sim_time`         | `False`          | Use `/clock` (`True` when replaying a bag)          |
| `use_approximate_sync` | `True`           | Approximate vs. exact `flex_sync` for inputs        |

The launch file passes all four through to `tagslam_node` parameters.
Additional node parameters declared by the source: `outbag`, `playback_rate`,
`output_directory`, `fixed_frame_id` (default `map`), `max_number_of_frames`,
`publish_ack`.

## Pure SLAM Mode

"Pure SLAM Mode" is the project's deployment configuration (see
`docs/02-architecture.md` §5 and `docs/06-qos-reference.md` §2.4):

- Only AprilTag detections are fed in. No `/odom` topic is provided to
  TagSLAM (the `camera_body` body has no `odom_topic` configured, so no
  `OdometryProcessor` is instantiated).
- The single non-static body `camera_body` represents the camera. Its
  optimized pose is published as `/tagslam/odom` after the launch remap.
- The single static body `indoor_lab` carries the known floor tags.
- `body.publish_tf: false` on `camera_body` — TagSLAM does **not**
  broadcast a `map → camera_body` TF. Tag TFs (`map → tag_<id>`) are
  always broadcast.
- Downstream: `/tagslam/odom` → `pose_corrector` → `/odom/drone` →
  `odometry_fuser` → `/odom/fused` → flight_controller / flight_planner /
  gimbal_controller.

## Topics

### Subscribed

| Topic             | Type                                                  | QoS                       | Source                              |
| ----------------- | ----------------------------------------------------- | ------------------------- | ----------------------------------- |
| `/tag_detections` | `isaac_ros_apriltag_interfaces/AprilTagDetectionArray` | per `flex_sync` (RELIABLE default; see docs/06) | Isaac ROS AprilTag (`isaac_ros_apriltag`) |

The topic name is read from `cameras.yaml` (`cam0.tag_topic`). No image
or `/odom` topic is consumed in Pure SLAM Mode.

### Published

| Topic            | Type                | QoS                                  | Notes                                          |
| ---------------- | ------------------- | ------------------------------------ | ---------------------------------------------- |
| `/tagslam/odom`  | `nav_msgs/Odometry` | BEST_EFFORT, VOLATILE, KEEP_LAST 1000 (`SensorDataQoS`) | `frame_id=map`, `child_frame_id=camera_body`   |
| `/tagslam/path`  | `nav_msgs/Path`     | BEST_EFFORT, VOLATILE, KEEP_LAST 1000 | Trajectory of `camera_body`                    |
| `/acknowledge`   | `std_msgs/Header`   | RELIABLE, depth 10                    | Only if `publish_ack:=true` (default false)    |
| TF: `map → tag_<id>` | `tf2_msgs/TFMessage` | TF default                       | Per-tag TFs are always broadcast               |

`map → camera_body` is **not** broadcast (suppressed by
`publish_tf: false` on the `camera_body` body).

### Services

| Service           | Type                  | Purpose                                              |
| ----------------- | --------------------- | ---------------------------------------------------- |
| `~/replay`        | `std_srvs/Trigger`    | Re-run optimization over buffered frames             |
| `~/dump`          | `std_srvs/Trigger`    | Write poses / calibration / diagnostics to disk      |
| `~/plot`          | `std_srvs/Trigger`    | Dump factor graph to `graph.dot`                     |

## Configuration Files

All three are required and live in
`jetson_prod/config/drone/tagslam/`:

| File              | Contents                                                                 |
| ----------------- | ------------------------------------------------------------------------ |
| `cameras.yaml`    | Single camera `cam0`: pinhole model, intrinsics `[fx, fy, cx, cy]`, radtan distortion, resolution, `image_topic`, `tag_topic` (`/tag_detections`), `rig_body: camera_body`. Intrinsics calibrated with kalibr (2026-01-21, source: `calib_01-camchain.yaml`). |
| `camera_poses.yaml` | Pose of `cam0` relative to its `rig_body` (`camera_body`). Identity transform with information matrix `diag(1e6)` (fixed mount). |
| `tagslam.yaml`    | `tagslam_parameters` (optimizer_mode, sync, noise), one static body `indoor_lab` with five floor tags (`22, 23, 24` at 0.15 m; `10, 21` at 0.20 m), one dynamic body `camera_body` (`publish_tf: false`, no odom). `amnesia: true`. |
| `template_for_calib_tagslam.yaml` | Editable template used during AprilTag map calibration. Optimizer is set to `slow`, `amnesia` disabled. |

### Tag-size invariant

Tag sizes in `tagslam.yaml` **must** match the per-id sizes in
`jetson_prod/config/drone/apriltag.yaml`:

| Tag ID | apriltag.yaml | tagslam.yaml |
| ------ | ------------- | ------------ |
| 22     | 0.15          | 0.15         |
| 23     | 0.15          | 0.15         |
| 24     | 0.15          | 0.15         |
| 10     | 0.20          | 0.20         |
| 21     | 0.20          | 0.20         |

Both files must be updated together. Camera intrinsics are configured
only inside `cameras.yaml` (TagSLAM does **not** consume
`/camera_info`).

## Frames

| Frame         | Defined by                            |
| ------------- | ------------------------------------- |
| `map`         | TagSLAM `fixed_frame_id` parameter (default `map`) |
| `camera_body` | TagSLAM body name (rig body for cam0) |
| `cam0`        | Camera frame, identity to `camera_body` per `camera_poses.yaml` |
| `tag_<id>`    | Broadcast by TagSLAM under `map`      |

`map → drone` is owned by `pose_corrector` (TagSLAM does not publish it).

## How to Run (this project)

The package is launched as part of the bringup scripts. To launch it
standalone for debugging:

```bash
# Inside the isaac-ros-dev container, after sourcing install/setup.bash
ros2 launch tagslam tagslam.launch.py \
    tagslam_config:=/workspaces/isaac_ros-dev/jetson_prod/config/drone/tagslam/tagslam.yaml \
    cameras:=/workspaces/isaac_ros-dev/jetson_prod/config/drone/tagslam/cameras.yaml \
    camera_poses:=/workspaces/isaac_ros-dev/jetson_prod/config/drone/tagslam/camera_poses.yaml \
    use_sim_time:=False \
    use_approximate_sync:=True
```

For replay from a recorded bag, pass `use_sim_time:=True` and start
`ros2 bag play --clock <bag>` in another terminal. Run with
`use_approximate_sync:=True` unless tag detections are guaranteed
hardware-synced (Android H.264 stream is not, hence the default).

## Troubleshooting

### Nothing happens
- Confirm Isaac ROS AprilTag is running and publishing `/tag_detections`.
- Confirm the topic name in `cameras.yaml` (`cam0.tag_topic`) matches.
- Confirm `use_sim_time` is consistent across all nodes when replaying.

### Large reprojection error / SUBGRAPH ERROR
- Wrong tag size (cross-check with `apriltag.yaml`).
- Wrong tag pose in `tagslam.yaml`.
- Bad camera intrinsics in `cameras.yaml`.
- Bad camera extrinsics in `camera_poses.yaml`.

### Pose drifts or jumps
- `amnesia: true` resets graph state between starts — expected on
  bring-up; persistent jumps during flight suggest sync issues
  (`use_approximate_sync`), low-quality detections, or a missing tag in
  the map.

## License

Upstream Apache-2.0. See [`LICENSE`](LICENSE).
