from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sequence_name = LaunchConfiguration('sequence_name')
    data_name = PathJoinSubstitution([FindPackageShare('benchmark_publisher'), 'config', sequence_name, 'data.csv'])

    return LaunchDescription([
        DeclareLaunchArgument('sequence_name', default_value='MH_01_easy'),
        Node(
            package='benchmark_publisher',
            executable='benchmark_publisher',
            name='benchmark_publisher',
            output='screen',
            remappings=[('estimated_odometry', '/vins_estimator/odometry')],
            parameters=[{'data_name': data_name}],
        ),
    ])
