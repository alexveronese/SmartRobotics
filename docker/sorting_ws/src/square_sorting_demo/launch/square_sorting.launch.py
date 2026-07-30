"""Launch the complete two-table square-sorting simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    demo_share = get_package_share_directory("square_sorting_demo")
    panda_share = get_package_share_directory("panda_description")
    realsense_share = get_package_share_directory("realsense2_description")
    ros_gz_share = get_package_share_directory("ros_gz_sim")

    world_path = os.path.join(demo_share, "worlds", "square_sorting.sdf")
    panda_xacro = os.path.join(panda_share, "urdf", "panda.urdf.xacro")
    controller_config = os.path.join(
        panda_share, "config", "ros2_controllers.yaml"
    )
    gz_launch = os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")

    headless = LaunchConfiguration("headless")
    auto_start = LaunchConfiguration("auto_start")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", panda_xacro]),
        value_type=str,
    )

    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            "gz_args": f"-r -s -v 3 {world_path}",
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(headless),
    )
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            "gz_args": f"-r -v 3 {world_path}",
            "on_exit_shutdown": "true",
        }.items(),
        condition=UnlessCondition(headless),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="square_sorting_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/sorting_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            (
                "/sorting_camera/camera_info"
                "@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
            ),
            (
                "/world/square_sorting/set_pose"
                "@ros_gz_interfaces/srv/SetEntityPose"
            ),
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    spawn_panda = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_panda",
        output="screen",
        arguments=[
            "-name",
            "panda",
            "-topic",
            "robot_description",
            "-allow_renaming",
            "false",
        ],
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--param-file",
            controller_config,
        ],
        output="screen",
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--param-file",
            controller_config,
        ],
        output="screen",
    )
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "eef_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--param-file",
            controller_config,
        ],
        output="screen",
    )

    detector = Node(
        package="square_sorting_demo",
        executable="shape_detector",
        name="shape_detector",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    sorter = Node(
        package="square_sorting_demo",
        executable="square_sorter",
        name="square_sorter",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "auto_start": ParameterValue(auto_start, value_type=bool),
                "robot_description": robot_description,
            }
        ],
    )

    delayed_robot_and_controllers = TimerAction(
        period=2.0,
        actions=[
            spawn_panda,
            joint_state_spawner,
            arm_spawner,
            gripper_spawner,
        ],
    )
    delayed_sorter = TimerAction(period=7.0, actions=[sorter])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="Run only the Gazebo server (no GUI).",
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="true",
                description="Start sorting automatically after stable detections.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo simulation clock.",
            ),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                os.pathsep.join(
                    [
                        os.path.dirname(panda_share),
                        os.path.dirname(realsense_share),
                    ]
                ),
            ),
            gazebo_headless,
            gazebo_gui,
            bridge,
            robot_state_publisher,
            detector,
            delayed_robot_and_controllers,
            delayed_sorter,
        ]
    )
