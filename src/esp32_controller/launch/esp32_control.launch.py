from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='esp32_controller',
            executable='esp32_control_node',
            name='esp32_control_node',
            output='screen',
        ),
    ])
