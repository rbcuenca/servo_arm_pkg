from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='servo_arm_pkg',
            executable='servo_arm',
            output='screen'),
    ])