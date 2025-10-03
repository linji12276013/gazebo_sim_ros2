from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    keyboard_node = Node(
        package="moveit2_tutorials",
        executable="servo_keyboard_input",
        remappings=[("/servo_node/delta_joint_cmds", "/engineer/engineer_servo_node/delta_joint_cmds"),
                    ("/servo_node/delta_twist_cmds", "/engineer/engineer_servo_node/delta_twist_cmds")],
        output="screen",
    )

    launch_description = LaunchDescription([keyboard_node])
    return launch_description
