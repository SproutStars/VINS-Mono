from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = LaunchConfiguration('config_path')
    vins_folder = LaunchConfiguration('vins_folder')

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=PathJoinSubstitution([FindPackageShare('vins_estimator'), 'config', '3dm', '3dm_config.yaml'])),
        DeclareLaunchArgument('vins_folder', default_value=FindPackageShare('vins_estimator')),
        Node(package='feature_tracker', executable='feature_tracker', name='feature_tracker', output='log', parameters=[{'config_file': config_path, 'vins_folder': vins_folder}]),
        Node(package='vins_estimator', executable='vins_estimator', name='vins_estimator', output='screen', parameters=[{'config_file': config_path, 'vins_folder': vins_folder}]),
    ])
