from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vins_launch = PathJoinSubstitution([FindPackageShare('vins_estimator'), 'launch', '3dm.launch.py'])
    calib_file = PathJoinSubstitution([FindPackageShare('vins_estimator'), 'config', '3dm', '3dm_config.yaml'])

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(vins_launch)),
        Node(
            package='ar_demo',
            executable='ar_demo_node',
            name='ar_demo_node',
            output='screen',
            remappings=[
                ('image_raw', '/mv_25001498/image_raw'),
                ('camera_pose', '/vins_estimator/camera_pose'),
                ('pointcloud', '/vins_estimator/point_cloud'),
            ],
            parameters=[{'calib_file': calib_file, 'use_undistored_img': False}],
        ),
    ])
