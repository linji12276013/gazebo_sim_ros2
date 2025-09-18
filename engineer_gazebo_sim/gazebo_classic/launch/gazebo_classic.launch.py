import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
import xacro
import re

def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

def generate_launch_description():
    description_package_name = 'engineer_description'
    gazebo_package_name = 'engineer_gazebo_sim'
    robot1_name_in_model = 'engineer'
    robot2_name_in_model = 'exchanger'
    description_pkg_share = FindPackageShare(package=description_package_name).find(description_package_name)
    gazebo_pkg_share = FindPackageShare(package=gazebo_package_name).find(gazebo_package_name)
    robot1_urdf_model_path = os.path.join(description_pkg_share, f'urdf/robot/engineer_robot.urdf.xacro')
    robot2_urdf_model_path = os.path.join(description_pkg_share, f'urdf/exchanger/engineer_exchanger.urdf.xacro')
    world_path = os.path.join(gazebo_pkg_share, 'gazebo_classic', 'worlds', 'robot.world')

    doc1 = xacro.parse(open(robot1_urdf_model_path))
    xacro.process_doc(doc1)
    params1 = {'robot_description': remove_comments(doc1.toxml())}

    doc2 = xacro.parse(open(robot2_urdf_model_path))
    xacro.process_doc(doc2)
    params2 = {'robot_description': remove_comments(doc2.toxml())}

    # Use gazebo launcher recommended in many examples
    gazebo =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '-world', world_path],
        output='screen')

    # robot_state_publisher for both robots in their namespaces
    node_robot_state_publisher1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='engineer',
        parameters=[{'use_sim_time': True}, params1, {"publish_frequency":15.0}],
        output='screen'
    )

    node_robot_state_publisher2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='exchanger',
        parameters=[{'use_sim_time': True}, params2, {"publish_frequency":15.0}],
        output='screen'
    )

    spawn_entity1 = Node(package='gazebo_ros', executable='spawn_entity.py',
                        # namespace='engineer',
                        arguments=['-topic', '/engineer/robot_description',
                                   '-entity', f'{robot1_name_in_model}',
                                   '-z', '0.0886',
                                   ], 
                        output='screen')
    
    spawn_entity2 = Node(package='gazebo_ros', executable='spawn_entity.py',
                        # namespace='exchanger',
                        arguments=['-topic', '/exchanger/robot_description',
                                   '-entity', f'{robot2_name_in_model}',
                                   '-x', '2.5',
                                   '-Y', '3.1415926',
                                   ],
                        output='screen')

    load_joint_state_controller2 = Node(
        package='controller_manager',
        executable='spawner',
        namespace='exchanger',
        arguments=[
            'joint_state_broadcaster',
            '-c', '/exchanger/controller_manager',
            '--controller-manager-timeout', '60',
            '--ros-args',
            '-r', '/joint_states:=/exchanger/joint_states'
        ],
        output='screen'
    )

    load_exchanger_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace='exchanger',
        arguments=[
            'exchanger_controller',
            '-c', '/exchanger/controller_manager',
            '--controller-manager-timeout', '60',
            # '--inactive',
        ],
        output='screen'
    )

    # Engineer controllers (namespaced under /engineer)
    load_joint_state_controller1 = Node(
        package='controller_manager',
        executable='spawner',
        namespace='engineer',
        arguments=[
            'joint_state_broadcaster',
            '-c', '/engineer/controller_manager',
            '--controller-manager-timeout', '60',
            '--ros-args',
            '-r', '/joint_states:=/engineer/joint_states'
        ],
        output='screen'
    )

    load_boom_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace='engineer',
        arguments=['boom_controller', '-c', '/engineer/controller_manager', '--controller-manager-timeout', '60'],
        output='screen'
    )

    load_left_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace='engineer',
        arguments=['left_controller', '-c', '/engineer/controller_manager', '--controller-manager-timeout', '60'],
        output='screen'
    )

    load_right_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace='engineer',
        arguments=['right_controller', '-c', '/engineer/controller_manager', '--controller-manager-timeout', '60'],
        output='screen'
    )

    # 目前是一个一个加载，否则会有控制器加载失败
    # === Event handlers to ensure correct startup order ===
    close_evt1 =  RegisterEventHandler( 
            event_handler=OnProcessExit(
                target_action=spawn_entity2,
                on_exit=[load_joint_state_controller2],
            )
    )

    close_evt2 =  RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller2,
                on_exit=[load_exchanger_joint_trajectory_controller],
            )
    )

    close_evt4 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_exchanger_joint_trajectory_controller,
                on_exit=[spawn_entity1],
            )
    )

    close_evt5 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity1,
                on_exit=[load_joint_state_controller1],
            )
    )

    close_evt6 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller1,
                on_exit=[load_boom_joint_trajectory_controller,
                         load_left_joint_trajectory_controller,
                         load_right_joint_trajectory_controller],
            )
    )

    ld = LaunchDescription([
        close_evt1,
        close_evt2,
        close_evt4,
        close_evt5,
        close_evt6,
        gazebo,
        node_robot_state_publisher2,
        node_robot_state_publisher1,
        spawn_entity2,
    ])

    return ld
