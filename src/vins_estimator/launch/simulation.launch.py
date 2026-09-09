from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_path = LaunchConfiguration('config_path')
    vins_folder = LaunchConfiguration('vins_folder')
    use_rviz = LaunchConfiguration('use_rviz')
    bringup_dir = get_package_share_directory("vins_estimator")
    launch_dir = os.path.join(bringup_dir, "launch")

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=PathJoinSubstitution([FindPackageShare('vins_estimator'), 'config', 'simulation', 'simulation_config.yaml'])),
        DeclareLaunchArgument('vins_folder', default_value=FindPackageShare('vins_estimator')),
        DeclareLaunchArgument('use_rviz', default_value='True'),
        Node(
            package='data_generator',
            executable='data_generator',
            name='data_generator',
            output='log'),
        Node(
            package='feature_tracker',
            executable='feature_tracker',
            name='feature_tracker',
            output='log',
            parameters=[config_path, {
                'config_file': config_path,
                'vins_folder': vins_folder
                }]),
        Node(
            package='vins_estimator',
            executable='vins_estimator',
            name='vins_estimator',
            output='screen',
            parameters=[config_path, {
                'config_file': config_path,
                'vins_folder': vins_folder
                }]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "vins_rviz.launch.py")),
            condition=IfCondition(use_rviz),
        )
    ])
