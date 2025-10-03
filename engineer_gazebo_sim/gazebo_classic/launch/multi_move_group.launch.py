from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_param_builder import ParameterBuilder
import os, yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def make_move_group_node(ns, moveit_config, joint_states_topic):

    mg_conf = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "capabilities": "",
        "disable_capabilities": "",
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }

    params = [
        moveit_config.to_dict(), 
        mg_conf,
        {"use_sim_time": True},
    ]

    node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        namespace=ns,
        output="screen",
        parameters=params,
        remappings=[
            ("/joint_states", joint_states_topic),
            ("/tf", "/tf"),
            ("/tf_static", "/tf_static"),
        ]
    )
    return node


def generate_launch_description():
    engineer_moveit_config = MoveItConfigsBuilder("engineer", package_name="engineer_moveit_config").to_moveit_configs()
    exchanger_moveit_config = MoveItConfigsBuilder("exchanger", package_name="exchanger_moveit_config").to_moveit_configs()

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("engineer_js", default_value="/engineer/joint_states",
                                        description="engineer joint_states topic"))
    ld.add_action(DeclareLaunchArgument("exchanger_js", default_value="/exchanger/joint_states",
                                        description="exchanger joint_states topic"))
    ld.add_action(DeclareLaunchArgument("rviz_config", default_value="",
                                        description="rviz config file (optional)"))

    engineer_js = LaunchConfiguration("engineer_js")
    exchanger_js = LaunchConfiguration("exchanger_js")

    mg_engineer = make_move_group_node("engineer", engineer_moveit_config, engineer_js)
    mg_exchanger = make_move_group_node("exchanger", exchanger_moveit_config, exchanger_js)

    ld.add_action(mg_engineer)
    ld.add_action(mg_exchanger)


    # Get parameters for the Servo node
    servo_yaml = load_yaml("engineer_gazebo_sim", "gazebo_classic/config/engineer_simulated_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = Node(
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

    ld.add_action(servo_node)


    pkg_share = FindPackageShare(package="engineer_gazebo_sim").find("engineer_gazebo_sim")
    engineer_rviz = os.path.join(pkg_share, "gazebo_classic", "config", "engineer_moveit.rviz")
    exchanger_rviz = os.path.join(pkg_share, "gazebo_classic", "config", "exchanger_moveit.rviz")

    engineer_rviz_parameters = [
        engineer_moveit_config.planning_pipelines,
        engineer_moveit_config.robot_description_kinematics,
        engineer_moveit_config.robot_description,
        engineer_moveit_config.robot_description_semantic,
    ]
    engineer_rviz_parameters.append({"use_sim_time": True})

    exchanger_rviz_parameters = [
        exchanger_moveit_config.planning_pipelines,
        exchanger_moveit_config.robot_description_kinematics,
        exchanger_moveit_config.robot_description,
        exchanger_moveit_config.robot_description_semantic,
    ]
    exchanger_rviz_parameters.append({"use_sim_time": True})

    engineer_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="engineer_rviz2",
        namespace="engineer",
        output="screen",
        arguments=["-d", engineer_rviz],
        parameters=engineer_rviz_parameters,
    )

    exchanger_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="exchanger_rviz2",
        namespace="exchanger",
        output="screen",
        arguments=["-d", exchanger_rviz],
        parameters=exchanger_rviz_parameters,
    )
    ld.add_action(engineer_rviz_node)
    # ld.add_action(exchanger_rviz_node)

    return ld
