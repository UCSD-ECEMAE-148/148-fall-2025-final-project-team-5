from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # -------------------------------
        # Gesture Detection Node (Camera + MediaPipe)
        # -------------------------------
        Node(
            package='ucsd_robodog',
            executable='gesture_detector_node',
            name='gesture_detector_node',
            output='screen'
        ),

        # -------------------------------
        # Gesture → cmd_vel Control Node
        # -------------------------------
        Node(
            package='ucsd_robodog',
            executable='gesture_cmd_node',
            name='gesture_cmd_node',
            output='screen'
        ),

    ])
