"""
Launch file for radiate_ros2.

Mandatory arguments
-------------------
dataset_root    Absolute path to the directory that contains sequence
                sub-directories (e.g. /data/radiate).
sequence_name   Name of the sequence sub-directory to play
                (e.g. tiny_foggy).

Optional arguments (all have sensible defaults)
-----------------------------------------------
playback_rate           Hz at which frames are published (default: 5.0).
loop                    Restart when the sequence ends — true/false
                        (default: false).

enable_lidar            true/false  (default: true)
enable_camera_left      true/false  (default: true)
enable_camera_right     true/false  (default: true)
enable_radar_cartesian  true/false  (default: true)
enable_radar_polar      true/false  (default: true)
enable_imu              true/false  (default: true)
enable_gps              true/false  (default: true)

base_frame_id           (default: base_link)
lidar_frame_id          (default: radiate_lidar)
camera_left_frame_id    (default: radiate_camera_left)
camera_right_frame_id   (default: radiate_camera_right)
radar_frame_id          (default: radiate_radar)
imu_frame_id            (default: radiate_imu)
gps_frame_id            (default: radiate_gps)

Example
-------
ros2 launch radiate_ros2 radiate_player.launch.py \\
    dataset_root:=/data/radiate \\
    sequence_name:=tiny_foggy \\
    playback_rate:=10.0 \\
    loop:=true \\
    enable_imu:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    def arg(name, default='', description=''):
        return DeclareLaunchArgument(name, default_value=default,
                                     description=description)

    def cfg(name):
        return LaunchConfiguration(name)

    return LaunchDescription([
        # Required
        arg('dataset_root', description='Root dir containing sequence sub-dirs'),
        arg('sequence_name', description='Name of the sequence to replay'),

        # Playback control
        arg('playback_rate', default='5.0',
            description='Publish rate in Hz'),
        arg('loop', default='false',
            description='Loop when sequence ends (true/false)'),

        # Per-modality enable flags
        arg('enable_lidar',            default='true'),
        arg('enable_camera_left',      default='true'),
        arg('enable_camera_right',     default='true'),
        arg('enable_radar_cartesian',  default='true'),
        arg('enable_radar_polar',      default='true'),
        arg('enable_imu',              default='true'),
        arg('enable_gps',              default='true'),

        # Frame IDs
        arg('base_frame_id',          default='base_link'),
        arg('lidar_frame_id',         default='radiate_lidar'),
        arg('camera_left_frame_id',   default='radiate_camera_left'),
        arg('camera_right_frame_id',  default='radiate_camera_right'),
        arg('radar_frame_id',         default='radiate_radar'),
        arg('imu_frame_id',           default='radiate_imu'),
        arg('gps_frame_id',           default='radiate_gps'),

        Node(
            package='radiate_ros2',
            executable='player',
            name='radiate_player',
            output='screen',
            parameters=[{
                'dataset_root':          cfg('dataset_root'),
                'sequence_name':         cfg('sequence_name'),
                'playback_rate':         cfg('playback_rate'),
                'loop':                  cfg('loop'),

                'enable_lidar':           cfg('enable_lidar'),
                'enable_camera_left':     cfg('enable_camera_left'),
                'enable_camera_right':    cfg('enable_camera_right'),
                'enable_radar_cartesian': cfg('enable_radar_cartesian'),
                'enable_radar_polar':     cfg('enable_radar_polar'),
                'enable_imu':             cfg('enable_imu'),
                'enable_gps':             cfg('enable_gps'),

                'base_frame_id':          cfg('base_frame_id'),
                'lidar_frame_id':         cfg('lidar_frame_id'),
                'camera_left_frame_id':   cfg('camera_left_frame_id'),
                'camera_right_frame_id':  cfg('camera_right_frame_id'),
                'radar_frame_id':         cfg('radar_frame_id'),
                'imu_frame_id':           cfg('imu_frame_id'),
                'gps_frame_id':           cfg('gps_frame_id'),
            }],
        ),
    ])
