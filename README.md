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
  `CMakeLists.txt`).

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

The launch file passes the five arguments above through to `tagslam_node`
parameters. Additional node parameters declared in the source
(`TagSLAM::readParams`, `tagslam.cpp`) — these are **not** exposed as
launch arguments and use their code defaults unless overridden:

| Parameter              | Type   | Default     | Use                                                        |
| ---------------------- | ------ | ----------- | ---------------------------------------------------------- |
| `outbag`               | string | `out.bag`   | Output bag written during `dump`                           |
| `playback_rate`        | double | `5.0`       | Wall-rate multiplier used by the `replay` service          |
| `output_directory`     | string | `.`         | Where `dump` writes `poses.yaml`, `calibration.yaml`, `camera_poses.yaml`, `error_map.txt`, `tag_diagnostics.txt`, `time_diagnostics.txt` |
| `fixed_frame_id`       | string | `map`       | `frame_id` of published odom/path and the TF fixed frame   |
| `max_number_of_frames` | int    | `0`         | If `>0`, auto-finalize after this many frames (0 = run forever) |
| `publish_ack`          | bool   | `false`     | If true, create the `/acknowledge` publisher and echo each processed header |

`cameras`, `camera_poses`, and `tagslam_config` are also declared as
string parameters and must point at valid YAML files (`tagslam_config`
and `cameras` are mandatory; `camera_poses` may be empty).

## Pure SLAM Mode

"Pure SLAM Mode" is the project's deployment configuration (see
`docs/02-architecture.md` §5 and `docs/06-qos-reference.md` §2.4):

- Only AprilTag detections are fed in. No `/odom` topic is provided to
  TagSLAM (the `camera_body` body has no `odom_topic` configured, so no
  `OdometryProcessor` is instantiated).
- The single non-static body `camera_body` represents the camera. Its
  optimized pose is published on `odom/body_camera_body` (remapped to
  `/tagslam/odom`) with `frame_id=map`; `child_frame_id` is empty because
  `camera_body` has no `odom_frame_id` in `tagslam.yaml`.
- The single static body `indoor_lab` carries the known floor tags.
- `body.publish_tf: false` on `camera_body` — TagSLAM does **not**
  broadcast a `map → camera_body` TF. Tag TFs (`<body_frame> → tag_<id>`,
  parent `indoor_lab` for the floor tags) and the camera extrinsic TF
  (`camera_body → cam0`) are always broadcast regardless of `publish_tf`.
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
| `/odom/body_camera_body` → remapped to `/tagslam/odom` | `nav_msgs/Odometry` | BEST_EFFORT, VOLATILE, KEEP_LAST 1000 (`SensorDataQoS`) | `frame_id=map` (`fixed_frame_id`); `child_frame_id` = body's `odom_frame_id`, which is **empty** for `camera_body` (no `odom_frame_id` set in `tagslam.yaml`). Base topic is `odom/body_<body_name>`, remapped to `/tagslam/odom` in `tagslam.launch.py`. |
| `/path/body_camera_body` → remapped to `/tagslam/path` | `nav_msgs/Path`     | BEST_EFFORT, VOLATILE, KEEP_LAST 1000 (`SensorDataQoS`) | Trajectory of `camera_body`; `header.frame_id=map`. Base topic is `path/body_<body_name>`, remapped to `/tagslam/path`. |
| `/acknowledge`   | `std_msgs/Header`   | depth 10 (default RELIABLE)           | Created only if `publish_ack:=true` (default false) |
| TF: `<body_frame> → tag_<id>` | `tf2_msgs/TFMessage` | `tf2_ros::TransformBroadcaster` default | Per-tag TFs are always broadcast, regardless of the body's `publish_tf` (parent frame is the owning body's frame, e.g. `indoor_lab`) |
| TF: `<rig_frame> → <cam_frame>` (`camera_body → cam0`) | `tf2_msgs/TFMessage` | broadcaster default | Camera extrinsic TF, always broadcast for each camera |

`map → camera_body` is **not** broadcast (suppressed by
`publish_tf: false` on the `camera_body` body).

### Services

All three are created with relative names on the `tagslam` node (no
namespace), so they resolve to `/replay`, `/dump`, `/plot`.

| Service     | Type                  | Purpose                                              |
| ----------- | --------------------- | ---------------------------------------------------- |
| `/replay`   | `std_srvs/Trigger`    | Re-run optimization over buffered frames at `playback_rate` |
| `/dump`     | `std_srvs/Trigger`    | Final optimization + write poses / calibration / diagnostics to `output_directory` and a bag |
| `/plot`     | `std_srvs/Trigger`    | Dump factor graph to `graph.dot`                     |

## Configuration Files

These live in `jetson_prod/config/drone/tagslam/`. The bringup script
(`start_tagslam()`) checks that `tagslam.yaml`, `cameras.yaml`, and
`camera_poses.yaml` all exist and aborts otherwise.
`template_for_calib_tagslam.yaml` is used only for offline map
calibration. (In the node code, `tagslam_config` and `cameras` are
mandatory; `camera_poses` may be empty.)

| File              | Contents                                                                 |
| ----------------- | ------------------------------------------------------------------------ |
| `cameras.yaml`    | Single camera `cam0`: pinhole model, intrinsics `[fx, fy, cx, cy]`, radtan distortion, resolution, `image_topic`, `tag_topic` (`/tag_detections`), `rig_body: camera_body`. Intrinsics calibrated with kalibr (2026-01-21, source: `calib_01-camchain.yaml`). |
| `camera_poses.yaml` | Pose of `cam0` relative to its `rig_body` (`camera_body`). Identity transform with information matrix `diag(1e6)` (fixed mount). |
| `tagslam.yaml`    | `tagslam_parameters` (`optimizer_mode: fast`, `minimum_tag_area: 2000`, `pixel_noise: 1.0`, `use_approximate_sync: true`, plus optimizer noise/error settings), `body_defaults`, one static body `indoor_lab` with five floor tags (`22, 23, 24` at 0.15 m; `10, 21` at 0.20 m), one dynamic body `camera_body` (`publish_tf: false`, no `odom_topic`). `amnesia: true`. |
| `template_for_calib_tagslam.yaml` | Editable template used during AprilTag map calibration. `optimizer_mode: slow`, `amnesia` commented out (disabled), tags 21–24 at 0.15 m. |

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
| `map`         | TagSLAM `fixed_frame_id` parameter (default `map`); the fixed parent frame of all published body TFs (`map → <body_frame>`, e.g. `map → indoor_lab`). Note `camera_body.publish_tf=false`, so the `map → camera_body` link is suppressed; `indoor_lab` has no `publish_tf` key, so it defaults to `true` and `map → indoor_lab` is broadcast. |
| `camera_body` | TagSLAM body frame for the rig body of `cam0` (frame id defaults to the body name) |
| `cam0`        | Camera frame (frame id defaults to camera name `cam0`); broadcast as `camera_body → cam0`, identity per `camera_poses.yaml` |
| `tag_<id>`    | Broadcast by TagSLAM with the owning body's frame as parent (e.g. `indoor_lab → tag_22`), not directly under `map` |

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
