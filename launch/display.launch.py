from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os
import launch
import xacro

def generate_launch_description():
    pkg = "project1phase1"
    xacro_file = "truck_urdf.urdf.xacro"
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # --- robot description ---
    robot_path = os.path.join(get_package_share_directory(pkg), "urdf", xacro_file)
    xml = xacro.process_file(robot_path).toxml()

    publish_robot_description = Node(
        package=pkg,
        executable="robot_description_publisher.py",
        name="robot_description_publisher",
        output="screen",
        arguments=["-xml_string", xml, "-robot_description_topic", "/robot_description"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": xml}],
        output="screen",
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=launch.conditions.UnlessCondition(LaunchConfiguration("gui")),
    )

    joint_state_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # ✅ Correct TF fix: rotate +π around Y, child of base_link
    tf_pitch_fix = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_pitch_fix",
        output="screen",
        arguments=["0", "0", "0", "0", "3.14159", "0", "base_link", "base_link_fixed"],
    )

    # --- RViz ---
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare(pkg), "rviz", "display_default.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["-d", rviz_cfg],
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument("gui", default_value="True"),
        launch.actions.DeclareLaunchArgument("use_sim_time", default_value="True"),
        publish_robot_description,
        joint_state_publisher,
        robot_state_publisher,
        tf_pitch_fix,      # <-- the only fix you need
        rviz_node,
        joint_state_gui,
    ])
