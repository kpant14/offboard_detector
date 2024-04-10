from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    px4_ns = LaunchConfiguration('px4_ns')
    mocap_rigid_body = LaunchConfiguration('mocap_rigid_body')

    px4_ns_arg = DeclareLaunchArgument(
        'px4_ns',
        default_value=''
    )
    
    mocap_rigid_body_arg = DeclareLaunchArgument(
        'mocap_rigid_body',
        default_value='drone162'
    )
    
    offboard_detector = Node(
        package='offboard_detector',
        executable='observer_detector_real.py',
        parameters=[
            {'px4_ns': px4_ns},
            {'mocap_rigid_body': mocap_rigid_body},
        ]
    )

    return LaunchDescription([
        px4_ns_arg,
        mocap_rigid_body_arg,
        offboard_detector,
    ])