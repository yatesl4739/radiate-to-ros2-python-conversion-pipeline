"""
msg_utils.py — Pure conversion functions from RADIATE data dicts to ROS 2 messages.

No node state, no rclpy.init, no publishers — just data transformations.
Every function is safe to call from multiple threads.
"""

import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu, NavSatFix, PointCloud2, PointField
from std_msgs.msg import Header


# --------------------------------------------------------------------------- #
# Shared helpers
# --------------------------------------------------------------------------- #

def _header(stamp, frame_id: str) -> Header:
    h = Header()
    h.stamp = stamp
    h.frame_id = frame_id
    return h


# --------------------------------------------------------------------------- #
# LiDAR
# --------------------------------------------------------------------------- #

def ndarray_to_pointcloud2(
    points: np.ndarray,
    stamp,
    frame_id: str,
) -> PointCloud2:
    """
    Convert an (N, 3..5) float32 numpy array to a ``PointCloud2`` message.

    Columns are interpreted in order: x, y, z [, intensity [, ring]].
    Extra columns beyond index 4 are silently dropped.
    Returns a valid but empty cloud when *points* is None or has zero rows.
    """
    msg = PointCloud2()
    msg.header = _header(stamp, frame_id)

    if points is None or points.size == 0:
        msg.height = 1
        msg.width = 0
        msg.fields = []
        msg.is_bigendian = False
        msg.point_step = 0
        msg.row_step = 0
        msg.is_dense = True
        msg.data = b''
        return msg

    points = np.ascontiguousarray(points, dtype=np.float32)
    ncols = min(points.shape[1], 5)

    _FIELD_DEFS = [
        ('x', 0), ('y', 4), ('z', 8), ('intensity', 12), ('ring', 16),
    ]
    fields = [
        PointField(name=name, offset=off, datatype=PointField.FLOAT32, count=1)
        for name, off in _FIELD_DEFS[:ncols]
    ]
    point_step = 4 * ncols

    msg.height = 1
    msg.width = points.shape[0]
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = point_step * points.shape[0]
    msg.is_dense = True
    msg.data = np.ascontiguousarray(points[:, :ncols]).tobytes()
    return msg


# --------------------------------------------------------------------------- #
# Images (camera / radar)
# --------------------------------------------------------------------------- #

def ndarray_to_image_msg(
    arr: np.ndarray,
    stamp,
    frame_id: str,
    bridge: CvBridge,
) -> Image | None:
    """
    Convert a uint8 numpy array to a ``sensor_msgs/Image``.

    Encoding is chosen automatically:
    - (H, W)   → ``mono8``
    - (H, W, 3) → ``bgr8``

    Returns ``None`` for any other shape or if *arr* is ``None``.
    """
    if arr is None:
        return None
    arr = np.asarray(arr, dtype=np.uint8)
    if arr.ndim == 2:
        encoding = 'mono8'
    elif arr.ndim == 3 and arr.shape[2] == 3:
        encoding = 'bgr8'
    else:
        return None
    msg = bridge.cv2_to_imgmsg(arr, encoding=encoding)
    msg.header = _header(stamp, frame_id)
    return msg


# --------------------------------------------------------------------------- #
# IMU
# --------------------------------------------------------------------------- #

def imu_dict_to_msg(imu: dict, stamp, frame_id: str) -> Imu:
    """
    Convert a parsed IMU dict (as returned by ``RadiateLoader``) to
    ``sensor_msgs/Imu``.

    Covariance arrays must have exactly 9 elements (3×3, row-major).
    If a covariance is absent or wrong-length its first element is set to
    ``-1`` to signal *unknown* per REP-145.
    """
    msg = Imu()
    msg.header = _header(stamp, frame_id)

    msg.orientation.x = float(imu.get('qx', 0.0))
    msg.orientation.y = float(imu.get('qy', 0.0))
    msg.orientation.z = float(imu.get('qz', 0.0))
    msg.orientation.w = float(imu.get('qw', 1.0))

    msg.angular_velocity.x = float(imu.get('avx', 0.0))
    msg.angular_velocity.y = float(imu.get('avy', 0.0))
    msg.angular_velocity.z = float(imu.get('avz', 0.0))

    msg.linear_acceleration.x = float(imu.get('ax', 0.0))
    msg.linear_acceleration.y = float(imu.get('ay', 0.0))
    msg.linear_acceleration.z = float(imu.get('az', 0.0))

    msg.orientation_covariance = _cov9(imu.get('orient_cov'))
    msg.angular_velocity_covariance = _cov9(imu.get('angvel_cov'))
    msg.linear_acceleration_covariance = _cov9(imu.get('linacc_cov'))

    return msg


# --------------------------------------------------------------------------- #
# GPS
# --------------------------------------------------------------------------- #

def gps_dict_to_msg(gps: dict, stamp, frame_id: str) -> NavSatFix:
    """
    Convert a parsed GPS dict (as returned by ``RadiateLoader``) to
    ``sensor_msgs/NavSatFix``.
    """
    msg = NavSatFix()
    msg.header = _header(stamp, frame_id)
    msg.latitude = float(gps.get('lat', 0.0))
    msg.longitude = float(gps.get('lon', 0.0))
    msg.altitude = float(gps.get('alt', 0.0))

    pos_cov = gps.get('position_covariance')
    if pos_cov and len(pos_cov) == 9:
        msg.position_covariance = [float(v) for v in pos_cov]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
    else:
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

    return msg


# --------------------------------------------------------------------------- #
# Private
# --------------------------------------------------------------------------- #

def _cov9(cov) -> list:
    """Return a 9-element float list; first element is -1.0 when unknown."""
    if cov and len(cov) == 9:
        return [float(v) for v in cov]
    return [-1.0] + [0.0] * 8
