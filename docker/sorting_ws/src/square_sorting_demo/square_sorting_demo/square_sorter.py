"""Recognize squares and command the Chapter 9 Panda to sort them."""

import json
import math
import threading
import time

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from square_sorting_demo.kinematics import SerialChain


class SquareSorter(Node):
    """Coordinate perception, IK, ros2_control and simulated attachment."""

    ARM_JOINTS = [f"joint{index}" for index in range(1, 8)]
    GRIPPER_JOINTS = ["finger_joint1", "finger_joint2"]
    HOME = np.array([0.0, -0.78, 0.0, -2.35, 0.0, 1.57, 0.78])
    SOURCE_MODELS = {
        "square_1": np.array([0.44, 0.24]),
        "square_2": np.array([0.61, 0.35]),
    }
    DESTINATIONS = [
        np.array([0.44, -0.31]),
        np.array([0.61, -0.31]),
    ]

    def __init__(self):
        super().__init__("square_sorter")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("robot_description", "")
        robot_description = str(
            self.get_parameter("robot_description").get_parameter_value().string_value
        )
        if not robot_description.strip():
            raise RuntimeError("robot_description is required for Panda IK")

        self._chain = SerialChain(
            robot_description, base_link="world", tip_link="hand_tcp"
        )
        if self._chain.joint_names != self.ARM_JOINTS:
            raise RuntimeError(
                "Unexpected Panda chain: " + ", ".join(self._chain.joint_names)
            )
        self._grasp_rotation = self._chain.forward(self.HOME)[:3, :3]

        self._arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )
        self._gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/eef_controller/follow_joint_trajectory",
        )
        self._pose_client = self.create_client(
            SetEntityPose, "/world/square_sorting/set_pose"
        )
        self._status_pub = self.create_publisher(
            String, "/square_sorting/status", 10
        )
        self.create_subscription(
            String, "/shape_detections", self._detection_callback, 10
        )
        self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 20
        )
        self.create_subscription(Bool, "/square_sorting/start", self._start_callback, 5)

        self._lock = threading.Lock()
        self._current_positions = None
        self._latest_detections = []
        self._stable_frames = 0
        self._last_signature = None
        self._worker = None
        self._held_object = None
        self._held_yaw = 0.0
        self._pose_future = None
        self.create_timer(0.08, self._follow_held_object)
        self.create_timer(0.5, self._auto_start_if_ready)
        self._status("WAITING: camera detections and ros2_control")

    def _status(self, message):
        self.get_logger().info(message)
        self._status_pub.publish(String(data=message))

    def _joint_state_callback(self, message):
        values = dict(zip(message.name, message.position))
        if all(name in values for name in self.ARM_JOINTS):
            with self._lock:
                self._current_positions = np.array(
                    [values[name] for name in self.ARM_JOINTS], dtype=float
                )

    def _detection_callback(self, message):
        try:
            payload = json.loads(message.data)
            detections = payload["detections"]
        except (ValueError, KeyError, TypeError) as error:
            self.get_logger().warning(f"Invalid detection payload: {error}")
            return

        source_squares = [
            item
            for item in detections
            if item.get("shape") == "square"
            and 0.20 <= float(item["position"][0]) <= 0.85
            and 0.05 <= float(item["position"][1]) <= 0.62
        ]
        signature = tuple(
            sorted(
                (
                    item["shape"],
                    round(float(item["position"][0]), 2),
                    round(float(item["position"][1]), 2),
                )
                for item in detections
            )
        )
        with self._lock:
            self._latest_detections = source_squares
            if signature == self._last_signature and len(source_squares) >= 2:
                self._stable_frames += 1
            else:
                self._stable_frames = 1 if len(source_squares) >= 2 else 0
            self._last_signature = signature

    def _start_callback(self, message):
        if message.data:
            self._start_worker()

    def _auto_start_if_ready(self):
        if not bool(self.get_parameter("auto_start").value):
            return
        with self._lock:
            ready = (
                self._stable_frames >= 3
                and self._current_positions is not None
            )
        if ready:
            self._start_worker()

    def _start_worker(self):
        if self._worker is not None and self._worker.is_alive():
            return
        with self._lock:
            if len(self._latest_detections) < 2:
                self._status("WAITING: two stable square detections are required")
                return
        self._worker = threading.Thread(target=self._run_sorting, daemon=True)
        self._worker.start()

    @staticmethod
    def _wait_future(future, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError("ROS future timed out")
            time.sleep(0.04)
        if not future.done():
            raise RuntimeError("ROS shutdown while waiting for a future")
        return future.result()

    def _send_trajectory(self, client, joint_names, positions, duration):
        trajectory = JointTrajectory()
        trajectory.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.time_from_start = Duration(seconds=duration).to_msg()
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()
        goal_handle = self._wait_future(client.send_goal_async(goal), 10.0)
        if not goal_handle.accepted:
            raise RuntimeError("Controller rejected trajectory")
        result = self._wait_future(goal_handle.get_result_async(), duration + 8.0)
        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"Trajectory failed with error {result.result.error_code}: "
                f"{result.result.error_string}"
            )

    def _move_joints(self, positions, duration=2.0):
        self._send_trajectory(
            self._arm_client, self.ARM_JOINTS, positions, duration
        )

    def _move_gripper(self, opening, duration=0.8):
        self._send_trajectory(
            self._gripper_client,
            self.GRIPPER_JOINTS,
            [opening, opening],
            duration,
        )

    def _move_tcp(self, x, y, z, duration=2.0):
        with self._lock:
            seed = (
                self.HOME.copy()
                if self._current_positions is None
                else self._current_positions.copy()
            )
        solution = self._chain.inverse(
            [x, y, z],
            self._grasp_rotation,
            seed,
            preferred=self.HOME,
        )
        self._move_joints(solution, duration)

    def _make_pose_request(self, name, position, yaw=0.0):
        request = SetEntityPose.Request()
        request.entity = Entity()
        request.entity.name = name
        request.entity.type = Entity.MODEL
        request.pose = Pose()
        request.pose.position.x = float(position[0])
        request.pose.position.y = float(position[1])
        request.pose.position.z = float(position[2])
        request.pose.orientation.z = math.sin(yaw / 2.0)
        request.pose.orientation.w = math.cos(yaw / 2.0)
        return request

    def _set_model_pose(self, name, position, yaw=0.0, wait=False):
        future = self._pose_client.call_async(
            self._make_pose_request(name, position, yaw)
        )
        if wait:
            response = self._wait_future(future, 5.0)
            if not response.success:
                raise RuntimeError(f"Gazebo refused pose update for {name}")
        return future

    def _follow_held_object(self):
        with self._lock:
            held_object = self._held_object
            positions = (
                None
                if self._current_positions is None
                else self._current_positions.copy()
            )
            yaw = self._held_yaw
        if held_object is None or positions is None:
            return
        if self._pose_future is not None and not self._pose_future.done():
            return
        tcp = self._chain.forward(positions)[:3, 3]
        self._pose_future = self._set_model_pose(
            held_object, tcp, yaw=yaw, wait=False
        )

    def _assign_square_models(self, detections):
        remaining = dict(self.SOURCE_MODELS)
        assignments = []
        for detection in sorted(
            detections, key=lambda item: float(item["position"][0])
        ):
            measured = np.array(detection["position"], dtype=float)
            model = min(
                remaining,
                key=lambda name: np.linalg.norm(measured - remaining[name]),
            )
            nominal = remaining.pop(model)
            distance = float(np.linalg.norm(measured - nominal))
            if distance > 0.13:
                raise RuntimeError(
                    f"Detection {measured} is too far from model {model} "
                    f"at {nominal}; check camera calibration"
                )
            assignments.append((model, measured))
        return assignments

    def _run_sorting(self):
        try:
            self._status("STARTING: waiting for controller actions and Gazebo service")
            if not self._arm_client.wait_for_server(timeout_sec=35.0):
                raise TimeoutError("arm_controller action is unavailable")
            if not self._gripper_client.wait_for_server(timeout_sec=35.0):
                raise TimeoutError("eef_controller action is unavailable")
            if not self._pose_client.wait_for_service(timeout_sec=35.0):
                raise TimeoutError("Gazebo SetEntityPose service is unavailable")

            with self._lock:
                detections = list(self._latest_detections[:2])
            assignments = self._assign_square_models(detections)
            self._status(
                "RECOGNIZED: "
                + ", ".join(
                    f"{name} square at ({position[0]:.3f}, {position[1]:.3f})"
                    for name, position in assignments
                )
                + "; triangles ignored"
            )

            self._move_joints(self.HOME, duration=2.5)
            self._move_gripper(0.035)

            for index, ((model, measured), destination) in enumerate(
                zip(assignments, self.DESTINATIONS), start=1
            ):
                self._status(
                    f"PICK {index}/{len(assignments)}: {model} "
                    f"from ({measured[0]:.3f}, {measured[1]:.3f})"
                )
                self._move_gripper(0.035)
                self._move_tcp(measured[0], measured[1], 0.24, duration=2.2)
                self._move_tcp(measured[0], measured[1], 0.047, duration=1.6)
                self._move_gripper(0.011)

                with self._lock:
                    self._held_object = model
                    self._held_yaw = 0.0
                time.sleep(0.25)
                self._move_tcp(measured[0], measured[1], 0.24, duration=1.8)
                self._status(
                    f"PLACE {index}/{len(assignments)}: {model} "
                    f"to ({destination[0]:.3f}, {destination[1]:.3f})"
                )
                self._move_tcp(
                    destination[0], destination[1], 0.24, duration=2.4
                )
                self._move_tcp(
                    destination[0], destination[1], 0.047, duration=1.7
                )

                with self._lock:
                    self._held_object = None
                self._set_model_pose(
                    model,
                    [destination[0], destination[1], 0.021],
                    wait=True,
                )
                self._move_gripper(0.035)
                self._move_tcp(
                    destination[0], destination[1], 0.24, duration=1.7
                )

            self._move_joints(self.HOME, duration=2.5)
            self._status(
                "COMPLETED: both squares are on the destination table; "
                "triangles remain on the source table"
            )
        except Exception as error:
            with self._lock:
                self._held_object = None
            self.get_logger().error(f"Square sorting failed: {error}")
            self._status(f"FAILED: {error}")


def main(args=None):
    rclpy.init(args=args)
    node = SquareSorter()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
