import os
import re
import tempfile
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import actions as L
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch.conditions import UnlessCondition  # <-- correct module
from launch_ros.actions import Node
import xacro

def _strip_xml_comments(xml: str) -> str:
    # remove <!-- ... --> blocks (multi-line)
    return re.sub(r'<!--.*?-->', '', xml, flags=re.DOTALL)

def generate_launch_description():
    # ---- Config ----
    xacro_file = "truck_urdf.urdf.xacro"
    package_description = "project1phase1"

    position = [0.0, 0.0, 1.5]     # x y z
    orientation = [3.14, 0.0, 0.0] # R P Y
    robot_base_name = "base_link"
    entity_name = f"{robot_base_name}-{random.random()}"

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='True')

    # ---- Resolve paths ----
    pkg_share = get_package_share_directory(package_description)
    xacro_path = os.path.join(pkg_share, "urdf", xacro_file)

    # ---- Render URDF and sanitize ----
    urdf_xml = xacro.process_file(xacro_path).toxml()
    urdf_xml = _strip_xml_comments(urdf_xml)

    # ---- Write URDF to a temp file for Gazebo spawn (avoid /robot_description topic) ----
    tmp_xml_path = tempfile.NamedTemporaryFile(prefix="truck_", suffix=".urdf", delete=False).name
    with open(tmp_xml_path, "w", encoding="utf-8") as f:
        f.write(urdf_xml)

    # ---- Nodes ----

    # Robot State Publisher (set robot_description as a PARAM, not CLI)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': urdf_xml}]
    )

    # Spawn the robot from file (no /robot_description topic needed)
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', entity_name,
            '-file', tmp_xml_path,
            '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),
            '-R', str(orientation[0]), '-P', str(orientation[1]), '-Y', str(orientation[2]),
        ]
    )

    # Joint State Publisher (only when gui==False)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(gui)  # <-- fix here
    )

    # Static TF (map -> dummy_link)
    tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['1', '0', '0', '0', '0', '0', '1', '/map', '/dummy_link'],
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output='screen'
    )
    robot_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
        output='screen'
    )
    robot_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
        output='screen'
    )

    # Start velocity & position controllers after JSB comes up
    delay_pos_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_position_controller_spawner],
        )
    )
    delay_vel_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_velocity_controller_spawner],
        )
    )

    return LaunchDescription([
        L.DeclareLaunchArgument('gui', default_value='True',
                                description='Enable joint_state_publisher GUI'),
        L.DeclareLaunchArgument('use_sim_time', default_value='True',
                                description='Use /clock from simulation'),

        # Order: RSP -> spawn -> controllers
        robot_state_publisher,
        joint_state_publisher_node,
        spawn_robot,
        joint_state_broadcaster_spawner,
        delay_pos_after_jsb,
        delay_vel_after_jsb,
        tf,
    ])


# import os

# import launch
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.substitutions import LaunchConfiguration
# from launch.actions import IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler
# from launch.event_handlers import OnProcessExit
# import launch_ros
# from launch_ros.actions import Node
# import xacro
# import random

# # this is the function launch  system will look for


# def generate_launch_description():

#     ####### DATA INPUT ##########
#     xacro_file = "truck_urdf.urdf.xacro"

#     package_description = "project1phase1"

#     # Position and orientation
#     # [X, Y, Z]
#     position = [0.0, 0.0, 1.5]
#     # [Roll, Pitch, Yaw]
#     orientation = [-3.14, 0.0, 0.0]
#     # Base Name or robot
#     robot_base_name = "base_link"
#     ####### DATA INPUT END ##########

#     # Path to robot model XACRO File
#     robot_desc_path = os.path.join(get_package_share_directory(
#         package_description), "urdf", xacro_file)


#     # Robot Description in XACRO Format
#     robot_desc = xacro.process_file(robot_desc_path)

#     # Robot Description in XML Format
#     xml = robot_desc.toxml()
    
#     # Entity Name
#     entity_name = robot_base_name+"-"+str(random.random())

   

#     # Spawn ROBOT Set Gazebo (Does not spwan robot only communicates with the Gazebo Client)
#     spawn_robot = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         name='spawn_entity',
#         output='screen',
#         arguments=['-entity',
#                    entity_name,
#                    '-x', str(position[0]), '-y', str(position[1]
#                                                      ), '-z', str(position[2]),
#                    '-R', str(orientation[0]), '-P', str(orientation[1]
#                                                         ), '-Y', str(orientation[2]),
#                    '-topic', '/robot_description'
#                    ]
#     )

#     # Publish Robot Desciption in String form in the topic /robot_description
#     publish_robot_description = Node(
#         package='project1phase1',
#         executable='robot_description_publisher.py',
#         name='robot_description_publisher',
#         output='screen',
#         arguments=['-xml_string', xml,
#                    '-robot_description_topic', '/robot_description'
#                    ]
#     )

#     # Launch Config for Simulation Time
#     use_sim_time = LaunchConfiguration('use_sim_time', default='true')

#     # Robot State Publisher Node
#     robot_state_publisher= Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         parameters=[{'use_sim_time': use_sim_time, 'robot_description': xml}],
#         output="screen"
#     )

#     # Joint State Publisher Node

#     joint_state_publisher_node = launch_ros.actions.Node(
#         package='joint_state_publisher',
#         executable='joint_state_publisher',
#         name='joint_state_publisher',
#         condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
#     )


#     # Static TF Transform
#     tf=Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='static_transform_publisher',
#         output='screen',
#         arguments=['1', '0', '0', '0', '0', '0', '1', '/map',  '/dummy_link'  ],
#     )

#     # create and return launch description object
#     return LaunchDescription(
#         [   
#             launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
#                                             description='Flag to enable joint_state_publisher_gui'),
#             launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
#                                             description='Flag to enable use_sim_time'),
#             publish_robot_description,
#             joint_state_publisher_node,
#             robot_state_publisher,
#             spawn_robot,
#             tf
#         ]
#     )