from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

def generate_launch_description():
    px4_ns = LaunchConfiguration('px4_ns')
    gz_world_name = LaunchConfiguration('gz_world_name')
    gz_model_name = LaunchConfiguration('gz_model_name')

    px4_ns_arg = DeclareLaunchArgument(
        'px4_ns',
        default_value='px4_1'
    )
    gz_world_name_arg = DeclareLaunchArgument(
        'gz_world_name',
        default_value='AbuDhabi'
    )
    gz_model_name_arg = DeclareLaunchArgument(
        'gz_model_name',
        default_value='x500_1'
    )
    
    gz_true_pos_publisher = Node(
        package='offboard_detector',
        executable='gz_true_pos_pub',
        parameters=[
            {'px4_ns': px4_ns},
            {'gz_world_name': gz_world_name},
            {'gz_model_name': gz_model_name},
        ]
    )
    
    offboard_detector = Node(
        package='offboard_detector',
        executable='observer_detector.py',
        parameters=[
            {'px4_ns': px4_ns},
        ]
    )

    return LaunchDescription([
        px4_ns_arg,
        gz_world_name_arg,
        gz_model_name_arg,
        offboard_detector,
        gz_true_pos_publisher,
    ])