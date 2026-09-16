"""
Launches the euroc_mav_sim Gazebo world and bridges its sensor + ground
truth topics into ROS 2. Does not run any lidar/vision processing -- this
only stands the simulation up.

  ros2 launch sensor_pkg sim.launch.py

Resulting ROS 2 topics (see sim/config/bridge.yaml for the full map):
  /imu, /lidar/points, /lidar/scan,
  /stereo/left/image_raw (+/camera_info), /stereo/right/image_raw (+/camera_info),
  /ground_truth/odom, /clock
  /cmd_vel (subscribed -- publish Twist here to drive the rig)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sensor_pkg')
    world_path = os.path.join(pkg_share, 'sim', 'worlds', 'euroc_mav_sim.sdf')
    bridge_config = os.path.join(pkg_share, 'sim', 'config', 'bridge.yaml')

    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='euroc_sim_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([gz_sim, bridge])
