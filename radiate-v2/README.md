# radiate_ros2

ROS 2 Humble Python package that replays the
[RADIATE](http://pro.hw.ac.uk/radiate/) adverse-weather dataset through
standard ROS 2 topics.

RADIATE is distributed as a Python SDK + raw data files (not ROS bags).
This package bridges the two: it wraps the SDK in a thin data-access layer
(`dataset_loader.py`), converts the numpy arrays to ROS messages
(`msg_utils.py`), and drives playback from a timer node
(`radiate_player_node.py`).

---

## Prerequisites

| Requirement | Notes |
|---|---|
| Ubuntu 22.04 | Tested distro for ROS 2 Humble |
| ROS 2 Humble | `ros-humble-desktop` |
| RADIATE Python SDK | Installed from `radiate_sdk/` (see below) |
| Python ≥ 3.10 | Ships with Ubuntu 22.04 |
| `cv_bridge` | `ros-humble-cv-bridge` |
| `tf2_ros` | `ros-humble-tf2-ros` |
| `pandas`, `numpy`, `opencv-python` | Runtime deps of the RADIATE SDK |

### Install the RADIATE SDK

The SDK is **not** on PyPI; install it in editable mode so the package
can find its config files at runtime:

```bash
pip3 install -e /path/to/radiate-test/radiate_sdk
```

---

## Dataset folder layout

Each RADIATE sequence must follow this layout (this is what the dataset
download produces):

```
<dataset_root>/
└── <sequence_name>/             ← sequence_name launch arg
    ├── velo_lidar/              CSV files: x,y,z,intensity,ring  (float)
    ├── Navtech_Cartesian/       PNG files: 1152×1152 BGR uint8
    ├── Navtech_Polar/           PNG files: 576 rows × 400 cols uint8
    ├── zed_left/                PNG files: 672×376 BGR uint8
    ├── zed_right/               PNG files: 672×376 BGR uint8
    ├── GPS_IMU_Twist/           TXT files: 18-line format (see below)
    ├── annotations/             annotations.json
    ├── velo_lidar.txt           frame↔timestamp index
    ├── Navtech_Cartesian.txt    frame↔timestamp index
    ├── Navtech_Polar.txt        frame↔timestamp index
    ├── GPS_IMU_Twist.txt        frame↔timestamp index
    ├── zed_left.txt             frame↔timestamp index
    ├── zed_right.txt            frame↔timestamp index
    └── meta.json
```

`dataset_loader.py` is the single source of truth for all path
conventions; edit it there if your download differs.

### GPS_IMU_Twist file format (18 non-empty lines)

```
0:     lat, lon, alt
1–3:   GPS position covariance 3×3 (one row per line)
4:     qx, qy, qz, qw         (orientation quaternion)
5:     angular_vel x, y, z    (rad/s)
6:     linear_accel x, y, z   (m/s²)
7–9:   IMU orientation covariance 3×3
10–12: IMU angular velocity covariance 3×3
13–15: IMU linear acceleration covariance 3×3
16:    twist linear x, y, z   (m/s)
17:    twist angular x, y, z  (rad/s)
```

---

## Build

```bash
# 1. Source ROS 2
source /opt/ros/humble/setup.bash

# 2. Create (or reuse) a colcon workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# 3. Place the package inside the workspace source tree
cp -r /home/liamyates/Desktop/radiate-v2 src/radiate_ros2
# — or symlink it:
# ln -s /home/liamyates/Desktop/radiate-v2 src/radiate_ros2

# 4. Resolve ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# 5. Build
colcon build --packages-select radiate_ros2

# 6. Source the install overlay
source install/setup.bash
```

---

## Run

### Via launch file (recommended)

```bash
ros2 launch radiate_ros2 radiate_player.launch.py \
    dataset_root:=/data/radiate \
    sequence_name:=tiny_foggy
```

With all options shown:

```bash
ros2 launch radiate_ros2 radiate_player.launch.py \
    dataset_root:=/data/radiate \
    sequence_name:=tiny_foggy \
    playback_rate:=10.0 \
    loop:=true \
    enable_lidar:=true \
    enable_camera_left:=true \
    enable_camera_right:=true \
    enable_radar_cartesian:=true \
    enable_radar_polar:=true \
    enable_imu:=true \
    enable_gps:=true \
    base_frame_id:=base_link \
    lidar_frame_id:=radiate_lidar \
    camera_left_frame_id:=radiate_camera_left \
    camera_right_frame_id:=radiate_camera_right \
    radar_frame_id:=radiate_radar \
    imu_frame_id:=radiate_imu \
    gps_frame_id:=radiate_gps
```

### Via `ros2 run` (manual parameter passing)

```bash
ros2 run radiate_ros2 player \
    --ros-args \
    -p dataset_root:=/data/radiate \
    -p sequence_name:=tiny_foggy \
    -p playback_rate:=5.0 \
    -p loop:=false \
    -p enable_imu:=false
```

---

## Published topics

| Topic | Message type | Description |
|---|---|---|
| `/radiate/lidar/points` | `sensor_msgs/PointCloud2` | Raw Velodyne HDL-32e point cloud (x, y, z, intensity, ring) |
| `/radiate/camera_left/image_raw` | `sensor_msgs/Image` | Left ZED camera, bgr8, 672×376 |
| `/radiate/camera_right/image_raw` | `sensor_msgs/Image` | Right ZED camera, bgr8, 672×376 |
| `/radiate/radar/cartesian` | `sensor_msgs/Image` | Navtech radar Cartesian image, 1152×1152 |
| `/radiate/radar/polar` | `sensor_msgs/Image` | Navtech radar polar image, 576×400 |
| `/radiate/imu/data` | `sensor_msgs/Imu` | Advanced Navigation IMU (orientation + angular vel + linear accel) |
| `/radiate/gps/fix` | `sensor_msgs/NavSatFix` | GPS fix (lat / lon / alt) |

All messages carry a `stamp` from the node's ROS clock and the
appropriate `frame_id` as configured.

### Static TF frames

The node publishes identity static transforms from `base_frame_id` to
every sensor frame at startup.  Override the frame ID parameters if you
want to supply accurate extrinsic calibration via a separate TF publisher.

---

## Viewing in RViz2

```bash
# In a separate terminal (with the workspace sourced):
rviz2
```

Suggested displays:

| Display | Topic / Frame |
|---|---|
| PointCloud2 | `/radiate/lidar/points` |
| Image | `/radiate/camera_left/image_raw` |
| Image | `/radiate/radar/cartesian` |
| Image | `/radiate/radar/polar` |
| Imu | `/radiate/imu/data` |

---

## Package structure

```
radiate_ros2/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── radiate_ros2          # ament resource index marker
├── radiate_ros2/
│   ├── __init__.py
│   ├── dataset_loader.py     # RADIATE SDK wrapper + direct file readers
│   ├── msg_utils.py          # numpy → ROS message conversions (no ROS state)
│   └── radiate_player_node.py # rclpy node: parameters, publishers, timer
└── launch/
    └── radiate_player.launch.py
```

---

## Troubleshooting

**`FileNotFoundError: RADIATE sequence directory not found`**
→ Check that `dataset_root` and `sequence_name` together form a valid path.

**`RuntimeError: RADIATE SDK failed to load`**
→ The SDK chdir-workaround relies on `radiate.__file__` pointing to the
installed SDK.  Confirm with `python3 -c "import radiate; print(radiate.__file__)"`.

**LiDAR / polar radar / GPS topics not appearing**
→ The corresponding timestamp index file (`velo_lidar.txt`, etc.) may be
absent.  The node logs a WARNING and skips that modality gracefully.

**Image topics present but no data in RViz**
→ Verify that `zed_left/` contains PNG files and that the SDK config has
`use_camera_left_rect: True` (the default).  The SDK's `config.yaml` is
read from the installed SDK directory, not from this package.
