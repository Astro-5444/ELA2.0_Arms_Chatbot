import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
)
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Replace with the name of your package containing the URDF/xacro
    package_name = "ela2_arms"

    # Get share and prefix directories
    share_dir = get_package_share_directory(package_name)
    prefix_dir = get_package_prefix(package_name)

    # Set GAZEBO_MODEL_PATH so Gazebo can find any models in your package
    model_path = os.path.join(share_dir, "models")
    model_path += pathsep + os.path.join(prefix_dir, "share")
    set_model_path = SetEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=model_path)

    # Launch argument to specify the path to your URDF .xacro file
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(share_dir, "urdf", "ela2arms.urdf.xacro"),
        description="Absolute path to robot URDF (xacro) file",
    )

    # Generate the robot description from xacro
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str,
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # Launch Gazebo server (physics backend)
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
        )
    )

    # Launch Gazebo client (GUI)
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
        )
    )

    # Spawn your robot into Gazebo
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "ela2_arms", "-topic", "robot_description"],
        output="screen",
    )

    

    return LaunchDescription([
        set_model_path,
        model_arg,
        robot_state_publisher_node,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot,
    ])
