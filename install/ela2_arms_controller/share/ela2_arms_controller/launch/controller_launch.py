from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the URDF/Xacro file
    robot_description_path = os.path.join(
        get_package_share_directory('ela2_arms'),
        'urdf',
        'ela2arms.urdf.xacro'
    )

    # Robot description parameter
    robot_description = ParameterValue(
        Command(['xacro ', robot_description_path]),
        value_type=str
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )

    # Left Arm Controller Spawner (CLAIMS left arm joints)
    left_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller', '--controller-manager', '/controller_manager']
    )

    # Right Arm Controller Spawner (CLAIMS right arm joints)
    right_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller', '--controller-manager', '/controller_manager']
    )


    

    return LaunchDescription([
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        left_arm_controller_spawner,
        right_arm_controller_spawner
    ])
