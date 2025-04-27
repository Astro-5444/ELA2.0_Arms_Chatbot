from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the argument for the URDF file
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            get_package_share_directory("ela2_arms"),
            "urdf",
            "ela2arms.urdf.xacro",
        ),
        description="Absolute path to the robot URDF file"
    )

    # Parse the xacro file and set it as the robot description
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")])
        
    )

    # Robot State Publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # Joint State Publisher GUI node
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("ela2_arms"),
                "rviz",
                "display.rviz",
            ),
        ],
    )

    # Return the launch description
    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node,
    ])
