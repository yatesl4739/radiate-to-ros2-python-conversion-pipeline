"""
dataset_loader.py — RADIATE dataset access layer.

All RADIATE SDK calls and direct file reads are isolated here.
Nothing in this module imports from rclpy or any ROS 2 package.

Folder convention (one sequence directory):

    <sequence_root>/
        velo_lidar/          *.csv  columns: x,y,z,intensity,ring  (float)
        Navtech_Cartesian/   *.png  1152×1152 BGR uint8
        Navtech_Polar/       *.png  576-row × 400-col uint8 (grayscale or BGR)
        zed_left/            *.png  672×376 BGR uint8
        zed_right/           *.png  672×376 BGR uint8
        GPS_IMU_Twist/       *.txt  18-line format (see _read_gps_imu)
        velo_lidar.txt       frame↔timestamp index
        Navtech_Polar.txt    frame↔timestamp index
        GPS_IMU_Twist.txt    frame↔timestamp index
        Navtech_Cartesian.txt frame↔timestamp index (used by SDK internally)
        zed_left.txt / zed_right.txt  (used by SDK internally)
"""

import logging
import os

import cv2
import numpy as np
import pandas as pd

import radiate  # RADIATE Python SDK (pip install from radiate_sdk/)

logger = logging.getLogger(__name__)

_LIDAR_DIR = 'velo_lidar'
_POLAR_DIR = 'Navtech_Polar'
_GPS_IMU_DIR = 'GPS_IMU_Twist'


class RadiateLoader:
    """
    Wraps the RADIATE ``Sequence`` class and adds direct file reading for
    sensors that the SDK does not expose through ``get_from_timestamp()``:
    raw LiDAR point clouds, polar radar images, and GPS/IMU records.

    Parameters
    ----------
    sequence_path : str
        Absolute path to a single RADIATE sequence directory.

    Raises
    ------
    FileNotFoundError
        If *sequence_path* does not exist or is not a directory.
    RuntimeError
        If the RADIATE SDK fails to load the sequence.
    """

    def __init__(self, sequence_path: str) -> None:
        self.sequence_path = os.path.realpath(sequence_path)
        if not os.path.isdir(self.sequence_path):
            raise FileNotFoundError(
                f'RADIATE sequence directory not found: {self.sequence_path!r}'
            )

        # The SDK resolves config files as relative paths ('config/config.yaml'
        # and 'config/default-calib.yaml') from wherever it was installed.
        # We must chdir to the SDK package directory before constructing
        # Sequence so those paths resolve correctly regardless of the CWD.
        sdk_dir = os.path.dirname(os.path.abspath(radiate.__file__))
        orig_dir = os.getcwd()
        os.chdir(sdk_dir)
        try:
            logger.info('Loading RADIATE sequence: %s', self.sequence_path)
            self._seq = radiate.Sequence(self.sequence_path)
        except Exception as exc:
            raise RuntimeError(
                f'RADIATE SDK failed to load {self.sequence_path!r}: {exc}'
            ) from exc
        finally:
            os.chdir(orig_dir)

        self.init_timestamp: float = float(self._seq.init_timestamp)
        self.end_timestamp: float = float(self._seq.end_timestamp)

        # Load per-sensor timestamp indices for sensors not exposed by the SDK.
        self._lidar_ts = self._load_ts('velo_lidar.txt')
        self._polar_ts = self._load_ts('Navtech_Polar.txt')
        self._gps_ts = self._load_ts('GPS_IMU_Twist.txt')

        def _avail(ts):
            return f'{len(ts["frame"])} frames' if ts else 'not found'

        logger.info(
            'Sequence ready  t=[%.3f, %.3f]  '
            'lidar_ts=%s  polar_ts=%s  gps_ts=%s',
            self.init_timestamp,
            self.end_timestamp,
            _avail(self._lidar_ts),
            _avail(self._polar_ts),
            _avail(self._gps_ts),
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get_frame(self, t: float) -> dict:
        """
        Return all available sensor data at dataset time *t*.

        The returned dict contains only the keys for which data was
        successfully loaded; missing or unreadable modalities are silently
        omitted (a DEBUG-level message is logged for file-not-found cases,
        WARNING for parse errors).

        Keys and value types
        --------------------
        ``camera_left``   : ``np.ndarray`` (H, W, 3) uint8 BGR
        ``camera_right``  : ``np.ndarray`` (H, W, 3) uint8 BGR
        ``radar_cart``    : ``np.ndarray`` (H, W, 3) uint8 BGR
        ``radar_polar``   : ``np.ndarray`` (H, W[, C]) uint8
        ``lidar``         : ``np.ndarray`` (N, 5) float32  [x,y,z,intensity,ring]
        ``gps``           : dict  {lat, lon, alt, position_covariance}
        ``imu``           : dict  {qx,qy,qz,qw, avx,avy,avz, ax,ay,az,
                                    orient_cov, angvel_cov, linacc_cov}

        Returns an empty dict when the SDK signals end-of-sequence.
        """
        result: dict = {}

        # SDK provides camera (left/right rect or raw) and radar cartesian.
        try:
            sdk_data = self._seq.get_from_timestamp(t)
        except Exception as exc:
            logger.error('SDK get_from_timestamp failed at t=%.3f: %s', t, exc)
            return result

        # The SDK returns an empty dict when the last radar boundary is reached.
        if not sdk_data:
            return result

        sensors = sdk_data.get('sensors', {})

        # Use explicit None checks — numpy arrays are not safe to use with `or`
        # because evaluating their truth value raises ValueError.
        cam_left = sensors.get('camera_left_rect')
        if cam_left is None:
            cam_left = sensors.get('camera_left_raw')
        if cam_left is not None:
            result['camera_left'] = np.asarray(cam_left, dtype=np.uint8)

        cam_right = sensors.get('camera_right_rect')
        if cam_right is None:
            cam_right = sensors.get('camera_right_raw')
        if cam_right is not None:
            result['camera_right'] = np.asarray(cam_right, dtype=np.uint8)

        radar_cart = sensors.get('radar_cartesian')
        if radar_cart is not None:
            result['radar_cart'] = np.asarray(radar_cart, dtype=np.uint8)

        # Raw LiDAR point cloud (SDK only returns a BEV image; read the CSV).
        lidar_frame = self._nearest_frame(self._lidar_ts, t)
        if lidar_frame is not None:
            pts = self._read_lidar(lidar_frame)
            if pts is not None:
                result['lidar'] = pts

        # Polar radar (SDK does not expose this; read PNG directly).
        polar_frame = self._nearest_frame(self._polar_ts, t)
        if polar_frame is not None:
            img = self._read_polar(polar_frame)
            if img is not None:
                result['radar_polar'] = img

        # GPS / IMU (not exposed by SDK; read text file directly).
        gps_frame = self._nearest_frame(self._gps_ts, t)
        if gps_frame is not None:
            gps, imu = self._read_gps_imu(gps_frame)
            if gps is not None:
                result['gps'] = gps
            if imu is not None:
                result['imu'] = imu

        return result

    # ------------------------------------------------------------------
    # Internal helpers — timestamp indexing
    # ------------------------------------------------------------------

    def _load_ts(self, filename: str):
        """Load a RADIATE timestamp index file.  Returns None if absent."""
        path = os.path.join(self.sequence_path, filename)
        if not os.path.exists(path):
            logger.debug('Timestamp file absent: %s', path)
            return None
        ts = self._seq.load_timestamp(path)
        if not ts or not ts.get('frame'):
            logger.warning('Empty or unreadable timestamp file: %s', path)
            return None
        return ts

    def _nearest_frame(self, timestamps, t: float):
        """Return the frame ID whose timestamp is closest to *t*."""
        if timestamps is None:
            return None
        times = np.asarray(timestamps['time'], dtype=np.float64)
        idx = int(np.argmin(np.abs(times - t)))
        return timestamps['frame'][idx]

    # ------------------------------------------------------------------
    # Internal helpers — direct file readers
    # ------------------------------------------------------------------

    def _read_lidar(self, frame_id: int):
        """Read a LiDAR CSV.  Returns (N,5) float32 or None."""
        path = os.path.join(
            self.sequence_path, _LIDAR_DIR, f'{frame_id:06d}.csv'
        )
        if not os.path.exists(path):
            logger.debug('LiDAR file not found: %s', path)
            return None
        try:
            data = pd.read_csv(path, header=None, dtype=np.float32).values
        except Exception as exc:
            logger.warning('Failed to read LiDAR %s: %s', path, exc)
            return None
        if data.ndim != 2 or data.shape[1] < 3:
            logger.warning(
                'Unexpected LiDAR CSV shape %s in %s — skipping', data.shape, path
            )
            return None
        return data.astype(np.float32)

    def _read_polar(self, frame_id: int):
        """Read a polar radar PNG.  Returns np.ndarray or None."""
        path = os.path.join(
            self.sequence_path, _POLAR_DIR, f'{frame_id:06d}.png'
        )
        if not os.path.exists(path):
            logger.debug('Polar radar file not found: %s', path)
            return None
        img = cv2.imread(path)
        if img is None:
            logger.warning('cv2.imread returned None for polar radar: %s', path)
        return img

    def _read_gps_imu(self, frame_id: int):
        """
        Parse a GPS_IMU_Twist text file.

        File layout (18 non-empty lines, values comma-separated):
          0:     lat, lon, alt                          (degrees / metres)
          1–3:   GPS position covariance 3×3 (row per line)
          4:     qx, qy, qz, qw                        (orientation quaternion)
          5:     angular_vel x, y, z                   (rad/s)
          6:     linear_accel x, y, z                  (m/s²)
          7–9:   IMU orientation covariance 3×3
          10–12: IMU angular velocity covariance 3×3
          13–15: IMU linear acceleration covariance 3×3
          16:    twist linear x, y, z                  (m/s)
          17:    twist angular x, y, z                 (rad/s)

        Returns (gps_dict, imu_dict) or (None, None) on any failure.
        """
        path = os.path.join(
            self.sequence_path, _GPS_IMU_DIR, f'{frame_id:06d}.txt'
        )
        if not os.path.exists(path):
            logger.debug('GPS/IMU file not found: %s', path)
            return None, None
        try:
            with open(path, 'r') as fh:
                lines = [ln.strip() for ln in fh if ln.strip()]

            def row(i):
                return [float(v) for v in lines[i].replace(' ', '').split(',')]

            gps_raw = row(0)
            gps_cov = row(1) + row(2) + row(3)
            q = row(4)
            av = row(5)
            la = row(6)
            orient_cov = row(7) + row(8) + row(9)
            angvel_cov = row(10) + row(11) + row(12)
            linacc_cov = row(13) + row(14) + row(15)

            gps_dict = {
                'lat': gps_raw[0],
                'lon': gps_raw[1],
                'alt': gps_raw[2],
                'position_covariance': gps_cov,
            }
            imu_dict = {
                'qx': q[0], 'qy': q[1], 'qz': q[2], 'qw': q[3],
                'avx': av[0], 'avy': av[1], 'avz': av[2],
                'ax': la[0], 'ay': la[1], 'az': la[2],
                'orient_cov': orient_cov,
                'angvel_cov': angvel_cov,
                'linacc_cov': linacc_cov,
            }
            return gps_dict, imu_dict

        except (IndexError, ValueError) as exc:
            logger.warning('Failed to parse GPS/IMU %s: %s', path, exc)
            return None, None
