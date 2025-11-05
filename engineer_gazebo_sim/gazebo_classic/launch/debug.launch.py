from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
        package="engineer_gazebo_sim",
        executable="image_raw_process",
        output="screen",
    )

    return LaunchDescription([node])
