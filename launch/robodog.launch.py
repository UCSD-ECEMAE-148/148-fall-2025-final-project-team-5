from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ------------------------------------------------------
    # Load VESC calibration YAML file
    # ------------------------------------------------------
    vesc_config = os.path.join(
        get_package_share_directory('ucsd_robocar_actuator2_pkg'),
        'config',
        'vesc_twsit_calibration.yaml'  # <-- MUST MATCH EXACT FILE NAME
    )

    return LaunchDescription([

        # ------------------------------------------------------
        # Gesture Detector Node (camera + mediapipe)
        # ------------------------------------------------------
        Node(
            package='ucsd_robodog',
            executable='gesture_detector_node',
            name='gesture_detector_node',
            output='screen'
        ),

        # ------------------------------------------------------
        # Gesture Command Node (state machine that drives)
        # ------------------------------------------------------
        Node(
            package='ucsd_robodog',
            executable='gesture_cmd_node',
            name='gesture_cmd_node',
            output='screen'
        ),

        # ------------------------------------------------------
        # Speaker Command Node (state machine that drives)
        # ------------------------------------------------------
        Node(
            package='ucsd_robodog',
            executable='speaker_node',
            name='speaker_node',
            output='screen'
        ),




        # ------------------------------------------------------
        # VESC Twist Node (translates /cmd_vel → motor commands)
        # + loads calibration YAML
        # ------------------------------------------------------
        Node(
            package='ucsd_robocar_actuator2_pkg',
            executable='vesc_twist_node',
            name='vesc_twist_node',
            output='screen',
            parameters=[vesc_config]  # <-- THIS LINE FIXES THE DEFAULT PARAM ISSUE
        ),
	Node(
	    package='ucsd_robodog',
            executable='speaker_node',
            name='vesc_twist_node',
            output='screen',
	),
        # ------------------------------------------------------
        # Gesture Command Node (state machine that drives)
        # ------------------------------------------------------
        Node(
            package='ucsd_robodog',
            executable='gun_detection_node',
            name='gun_detection_node',
            output='screen'
        ),


    ])
