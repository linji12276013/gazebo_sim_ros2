from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os, yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    engineer_moveit_config = MoveItConfigsBuilder("engineer", package_name="engineer_moveit_config").to_moveit_configs()

    servo_yaml = load_yaml("engineer_gazebo_sim", "gazebo_classic/config/engineer_simulated_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="engineer_moveit_servo_container",
        namespace="/engineer",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                namespace="/engineer",
                parameters=[{"child_frame_id": "/link1", "frame_id": "/base_link"}],
            ),
            ComposableNode(
                package="engineer_gazebo_sim",
                plugin="engineer::JoyToServoPub",
                name="engineer_controller_to_servo_node",
                # namespace="/engineer",
                remappings=[("/servo_node/delta_joint_cmds", "/engineer/engineer_servo_node/delta_joint_cmds"),
                            ("/servo_node/delta_twist_cmds", "/engineer/engineer_servo_node/delta_twist_cmds"),
            ]),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="engineer_joy_node",
            ),
        ],
        output="screen",
    )

    node = Node(
        package="engineer_gazebo_sim",
        executable="engineer_servo_control",
        parameters=[
            servo_params,
            engineer_moveit_config.robot_description,
            engineer_moveit_config.robot_description_semantic,
            engineer_moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    launch_description = LaunchDescription([node, container])
    return launch_description
