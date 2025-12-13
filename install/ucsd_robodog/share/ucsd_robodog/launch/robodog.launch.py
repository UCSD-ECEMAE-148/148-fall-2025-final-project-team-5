from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ucsd_robodog',
            executable='gesture_detector_node',
            name='gesture_detector',
            output='screen',
        ),
        Node(
            package='ucsd_robodog',
            executable='gesture_cmd_node',
            name='gesture_cmd',
            output='screen',
        ),
    ])
