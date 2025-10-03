""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-IN-HAND: link7 -> engineer_depth_camera_optical_link """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "link7",
                "--child-frame-id",
                "engineer_depth_camera_optical_link",
                "--x",
                "0.476788",
                "--y",
                "0.0511815",
                "--z",
                "0.0054438",
                "--qx",
                "0.0222263",
                "--qy",
                "0.823273",
                "--qz",
                "0.554501",
                "--qw",
                "-0.119402",
                # "--roll",
                # "1.20045",
                # "--pitch",
                # "-2.96878",
                # "--yaw",
                # "0.172422",
            ],
        ),
    ]
    return LaunchDescription(nodes)
