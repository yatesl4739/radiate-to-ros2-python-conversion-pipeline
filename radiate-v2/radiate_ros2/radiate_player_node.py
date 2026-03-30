"""
radiate_player_node.py — ROS 2 node that replays a RADIATE dataset sequence.

Parameters
----------
dataset_root        str   Root directory that contains sequence sub-directories.
sequence_name       str   Name of the sequence to load (sub-dir of dataset_root).
playback_rate       float Playback speed in Hz (default: 5.0).
loop                bool  Restart from the beginning when the sequence ends
                          (default: false).

enable_lidar        bool  Publish raw LiDAR PointCloud2 (default: true).
enable_camera_left  bool  Publish left camera Image (default: true).
enable_camera_right bool  Publish right camera Image (default: true).
enable_radar_cartesian bool Publish cartesian radar Image (default: true).
enable_radar_polar  bool  Publish polar radar Image (default: true).
enable_imu          bool  Publish Imu (default: true).
enable_gps          bool  Publish NavSatFix (default: true).

base_frame_id       str   Parent frame for all static TF transforms
                          (default: base_link).
lidar_frame_id      str   (default: radiate_lidar).
camera_left_frame_id str  (default: radiate_camera_left).
camera_right_frame_id str (default: radiate_camera_right).
radar_frame_id      str   Shared frame for both radar topics
                          (default: radiate_radar).
imu_frame_id        str   (default: radiate_imu).
gps_frame_id        str   (default: radiate_gps).

Published topics
----------------
/radiate/lidar/points           sensor_msgs/PointCloud2
/radiate/camera_left/image_raw  sensor_msgs/Image   (bgr8, 672×376)
/radiate/camera_right/image_raw sensor_msgs/Image   (bgr8, 672×376)
/radiate/radar/cartesian        sensor_msgs/Image   (bgr8, 1152×1152)
/radiate/radar/polar            sensor_msgs/Image   (bgr8 or mono8, 576×400)
/radiate/imu/data               sensor_msgs/Imu
/radiate/gps/fix                sensor_msgs/NavSatFix
"""

import os

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, NavSatFix, PointCloud2
from tf2_ros import StaticTransformBroadcaster

from .dataset_loader import RadiateLoader
from .msg_utils import (
    gps_dict_to_msg,
    imu_dict_to_msg,
    ndarray_to_image_msg,
    ndarray_to_pointcloud2,
)


class RadiatePlayerNode(Node):
    """Replays a RADIATE sequence by publishing sensor data at a fixed rate."""

    def __init__(self) -> None:
        super().__init__('radiate_player')

        # ------------------------------------------------------------------ #
        # Parameter declarations
        # ------------------------------------------------------------------ #
        self.declare_parameter('dataset_root', '')
        self.declare_parameter('sequence_name', '')
        self.declare_parameter('playback_rate', 5.0)
        self.declare_parameter('loop', False)

        self.declare_parameter('enable_lidar', True)
        self.declare_parameter('enable_camera_left', True)
        self.declare_parameter('enable_camera_right', True)
        self.declare_parameter('enable_radar_cartesian', True)
        self.declare_parameter('enable_radar_polar', True)
        self.declare_parameter('enable_imu', True)
        self.declare_parameter('enable_gps', True)

        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('lidar_frame_id', 'radiate_lidar')
        self.declare_parameter('camera_left_frame_id', 'radiate_camera_left')
        self.declare_parameter('camera_right_frame_id', 'radiate_camera_right')
        self.declare_parameter('radar_frame_id', 'radiate_radar')
        self.declare_parameter('imu_frame_id', 'radiate_imu')
        self.declare_parameter('gps_frame_id', 'radiate_gps')

        # ------------------------------------------------------------------ #
        # Read parameters
        # ------------------------------------------------------------------ #
        dataset_root = self._str('dataset_root')
        sequence_name = self._str('sequence_name')

        if not dataset_root or not sequence_name:
            raise RuntimeError(
                'Parameters dataset_root and sequence_name are both required. '
                'Pass them on the command line:\n'
                '  ros2 run radiate_ros2 player '
                '--ros-args -p dataset_root:=/data/radiate '
                '-p sequence_name:=tiny_foggy'
            )

        self._rate = float(self._val('playback_rate'))
        self._loop = self._as_bool(self._val('loop'))

        self._en_lidar = self._as_bool(self._val('enable_lidar'))
        self._en_cam_left = self._as_bool(self._val('enable_camera_left'))
        self._en_cam_right = self._as_bool(self._val('enable_camera_right'))
        self._en_radar_cart = self._as_bool(self._val('enable_radar_cartesian'))
        self._en_radar_polar = self._as_bool(self._val('enable_radar_polar'))
        self._en_imu = self._as_bool(self._val('enable_imu'))
        self._en_gps = self._as_bool(self._val('enable_gps'))

        self._base_frame = self._str('base_frame_id')
        self._lidar_frame = self._str('lidar_frame_id')
        self._cam_left_frame = self._str('camera_left_frame_id')
        self._cam_right_frame = self._str('camera_right_frame_id')
        self._radar_frame = self._str('radar_frame_id')
        self._imu_frame = self._str('imu_frame_id')
        self._gps_frame = self._str('gps_frame_id')

        # ------------------------------------------------------------------ #
        # Load dataset
        # ------------------------------------------------------------------ #
        sequence_path = os.path.join(dataset_root, sequence_name)
        self.get_logger().info(f'Loading sequence: {sequence_path}')
        self._loader = RadiateLoader(sequence_path)
        self._current_t = self._loader.init_timestamp
        self.get_logger().info(
            f'Sequence spans [{self._loader.init_timestamp:.3f}, '
            f'{self._loader.end_timestamp:.3f}] s'
        )

        self._bridge = CvBridge()

        # ------------------------------------------------------------------ #
        # Publishers (only created when the modality is enabled)
        # ------------------------------------------------------------------ #
        def _pub(msg_type, topic, enabled):
            return self.create_publisher(msg_type, topic, 10) if enabled else None

        self._pub_lidar = _pub(PointCloud2, '/radiate/lidar/points', self._en_lidar)
        self._pub_cam_left = _pub(
            Image, '/radiate/camera_left/image_raw', self._en_cam_left
        )
        self._pub_cam_right = _pub(
            Image, '/radiate/camera_right/image_raw', self._en_cam_right
        )
        self._pub_radar_cart = _pub(
            Image, '/radiate/radar/cartesian', self._en_radar_cart
        )
        self._pub_radar_polar = _pub(
            Image, '/radiate/radar/polar', self._en_radar_polar
        )
        self._pub_imu = _pub(Imu, '/radiate/imu/data', self._en_imu)
        self._pub_gps = _pub(NavSatFix, '/radiate/gps/fix', self._en_gps)

        enabled_list = [
            name for name, flag in [
                ('lidar', self._en_lidar),
                ('camera_left', self._en_cam_left),
                ('camera_right', self._en_cam_right),
                ('radar_cartesian', self._en_radar_cart),
                ('radar_polar', self._en_radar_polar),
                ('imu', self._en_imu),
                ('gps', self._en_gps),
            ] if flag
        ]
        self.get_logger().info(f'Enabled modalities: {", ".join(enabled_list)}')

        # ------------------------------------------------------------------ #
        # Static TF — identity transforms so all sensor frames are reachable
        # ------------------------------------------------------------------ #
        self._tf_bc = StaticTransformBroadcaster(self)
        self._publish_static_tfs()

        # ------------------------------------------------------------------ #
        # Playback timer
        # ------------------------------------------------------------------ #
        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            f'radiate_player ready  rate={self._rate} Hz  loop={self._loop}'
        )

    # ---------------------------------------------------------------------- #
    # Timer callback
    # ---------------------------------------------------------------------- #

    def _tick(self) -> None:
        if self._current_t > self._loader.end_timestamp:
            if self._loop:
                self.get_logger().info('Sequence ended — looping.')
                self._current_t = self._loader.init_timestamp
            else:
                self.get_logger().info('Sequence ended — stopping timer.')
                self._timer.cancel()
                return

        frame = self._loader.get_frame(self._current_t)

        if not frame:
            # Loader returns empty dict when the SDK signals end-of-sequence.
            if self._loop:
                self.get_logger().info('Empty frame at end — looping.')
                self._current_t = self._loader.init_timestamp
            else:
                self.get_logger().info('Empty frame — stopping timer.')
                self._timer.cancel()
            return

        stamp = self.get_clock().now().to_msg()

        if self._en_lidar and 'lidar' in frame:
            self._pub_lidar.publish(
                ndarray_to_pointcloud2(frame['lidar'], stamp, self._lidar_frame)
            )

        if self._en_cam_left and 'camera_left' in frame:
            msg = ndarray_to_image_msg(
                frame['camera_left'], stamp, self._cam_left_frame, self._bridge
            )
            if msg:
                self._pub_cam_left.publish(msg)

        if self._en_cam_right and 'camera_right' in frame:
            msg = ndarray_to_image_msg(
                frame['camera_right'], stamp, self._cam_right_frame, self._bridge
            )
            if msg:
                self._pub_cam_right.publish(msg)

        if self._en_radar_cart and 'radar_cart' in frame:
            msg = ndarray_to_image_msg(
                frame['radar_cart'], stamp, self._radar_frame, self._bridge
            )
            if msg:
                self._pub_radar_cart.publish(msg)

        if self._en_radar_polar and 'radar_polar' in frame:
            msg = ndarray_to_image_msg(
                frame['radar_polar'], stamp, self._radar_frame, self._bridge
            )
            if msg:
                self._pub_radar_polar.publish(msg)

        if self._en_imu and 'imu' in frame:
            self._pub_imu.publish(
                imu_dict_to_msg(frame['imu'], stamp, self._imu_frame)
            )

        if self._en_gps and 'gps' in frame:
            self._pub_gps.publish(
                gps_dict_to_msg(frame['gps'], stamp, self._gps_frame)
            )

        self._current_t += 1.0 / self._rate

    # ---------------------------------------------------------------------- #
    # Helpers
    # ---------------------------------------------------------------------- #

    def _val(self, name):
        return self.get_parameter(name).value

    def _str(self, name) -> str:
        return str(self._val(name))

    @staticmethod
    def _as_bool(v) -> bool:
        """Coerce the value from either a Python bool or a 'true'/'false' string."""
        if isinstance(v, bool):
            return v
        return str(v).strip().lower() in ('true', '1', 'yes')

    def _publish_static_tfs(self) -> None:
        now = self.get_clock().now().to_msg()
        child_frames = [
            self._lidar_frame,
            self._cam_left_frame,
            self._cam_right_frame,
            self._radar_frame,
            self._imu_frame,
            self._gps_frame,
        ]
        transforms = []
        for child in child_frames:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self._base_frame
            t.child_frame_id = child
            t.transform.rotation.w = 1.0  # identity quaternion
            transforms.append(t)
        self._tf_bc.sendTransform(transforms)


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #

def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RadiatePlayerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
