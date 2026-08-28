"""Start the native ROS 2 Jazzy / Gazebo Panda tic-tac-toe simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    package_share = get_package_share_directory("nuovo_progetto")
    panda_share = get_package_share_directory("moveit_resources_panda_description")
    controllers_path = os.path.join(package_share, "config", "controllers.yaml")
    xacro_path = os.path.join(package_share, "urdf", "panda.urdf.xacro")
    world_path = os.path.join(package_share, "worlds", "tic_tac_toe.sdf")
    robot_description = xacro.process_file(
        xacro_path, mappings={"controllers_file": controllers_path}
    ).toxml().replace('<mimic joint="panda_finger_joint1"/>', "")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/world/tic_tac_toe/set_pose@ros_gz_interfaces/srv/SetEntityPose",
        ] + [
            mapping
            for mark in ("x", "o")
            for index in range(1, 6)
            for mapping in (
                f"/{mark}_piece_{index}/attach@std_msgs/msg/Empty]gz.msgs.Empty",
                f"/{mark}_piece_{index}/detach@std_msgs/msg/Empty]gz.msgs.Empty",
                f"/{mark}_piece_{index}/state@std_msgs/msg/String[gz.msgs.StringMsg",
            )
        ],
    )
    delayed_robot_and_controllers = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_sim", executable="create", output="screen",
                arguments=["-topic", "robot_description", "-name", "tic_tac_toe_panda", "-allow_renaming", "false"],
            ),
            Node(
                package="controller_manager", executable="spawner", output="screen",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "60", "--param-file", controllers_path],
            ),
            Node(
                package="controller_manager", executable="spawner", output="screen",
                arguments=["arm_controller", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "60", "--param-file", controllers_path],
            ),
            Node(
                package="controller_manager", executable="spawner", output="screen",
                arguments=["eef_controller", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "60", "--param-file", controllers_path],
            ),
        ],
    )
    return LaunchDescription(
        [
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(panda_share)),
            gazebo,
            state_publisher,
            bridge,
            delayed_robot_and_controllers,
        ]
    )
