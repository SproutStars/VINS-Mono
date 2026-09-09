from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution([FindPackageShare('vins_estimator'), 'config', 'AR_demo.rviz'])
    return LaunchDescription([
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log', arguments=['-d', rviz_config]),
    ])
