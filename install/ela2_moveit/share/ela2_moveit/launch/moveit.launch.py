from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Declare simulation argument
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    # Retrieve the argument value
    is_sim = LaunchConfiguration("is_sim")

    # MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("ela2", package_name="ela2_moveit")  # CHANGE "ela2" and "ela2_moveit" to your robot and package names
        .robot_description(file_path=os.path.join(get_package_share_directory("ela2_arms"), "urdf", "ela2arms.urdf.xacro"))  # Update the paths for your URDF
        .robot_description_semantic(file_path="config/ela2.srdf")  # Update the SRDF file name
        .trajectory_execution(file_path="config/moveit_controllers.yaml")  # Ensure your controllers file is correctly referenced
        .to_moveit_configs()
    )

    # Move group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": is_sim}, {"publish_robot_description_semantic": True}],
        arguments=["--ros-args", "--log-level", "info"]
    )

    # RViz configuration
    rviz_config = os.path.join(get_package_share_directory("ela2_moveit"), "config", "moveit.rviz")  # Ensure the path to RViz config is correct
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ]
    )

    return LaunchDescription([
        is_sim_arg,
        move_group_node,
        rviz_node
    ])
