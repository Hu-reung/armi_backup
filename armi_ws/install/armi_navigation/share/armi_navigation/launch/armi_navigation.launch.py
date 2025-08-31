from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='armi_navigation',
            executable='nav_to_pose_client',
            name='nav_to_pose_client',
            output='screen'
        ),
        Node(
            package='armi_navigation',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen'
        ),
        Node(
            package='armi_navigation',
            executable='camera_node',
            name='camera_node',
            output='screen'
        )
    ])
