#!/usr/bin/env python3
"""Interactive ROS 2 node coordinating vision, game play, and manipulation."""

from __future__ import annotations

import sys
import threading
import time
from collections import deque
from typing import Iterable, Optional, Sequence, Tuple

from ament_index_python.packages import get_package_share_directory
import rclpy
from control_msgs.action import FollowJointTrajectory
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import xacro
import numpy as np

from tic_tac_toe.game import choose_best_move, game_result
from tic_tac_toe.kinematics import SerialChain
from tic_tac_toe.vision import detect_board


HOME = np.array([0.0, -0.78, 0.0, -2.35, 0.0, 1.57, 0.78])
ARM_JOINTS = [f"panda_joint{index}" for index in range(1, 8)]
GRIPPER_JOINTS = ["panda_finger_joint1", "panda_finger_joint2"]

# Camera numbering is top-left to bottom-right. With the supplied camera pose,
# image rows run from +world-X to -world-X and columns from +world-Y to -world-Y.
X_SUPPLY_PIECE_Z = 0.007
X_BOARD_PIECE_Z = 0.023
O_SUPPLY_PIECE_Z = 0.015
O_BOARD_PIECE_Z = 0.031
SUPPLY_GRASP_Z = O_SUPPLY_PIECE_Z
BOARD_GRASP_Z = O_BOARD_PIECE_Z
GRIPPER_OPEN_POSITION = 0.040
# The O has a 68 mm outside diameter. A 66 mm commanded aperture gives the
# simulated fingers 1 mm of contact on each side without closing through it.
O_GRASP_FINGER_POSITION = 0.033

CELL_POSITIONS = tuple(
    (0.67 - row * 0.10, 0.10 - col * 0.10)
    for row in range(3) for col in range(3)
)
X_SUPPLY = ((0.31, 0.26), (0.41, 0.26), (0.51, 0.26), (0.61, 0.26), (0.71, 0.26))
O_SUPPLY = ((0.31, -0.26), (0.41, -0.26), (0.51, -0.26), (0.61, -0.26), (0.71, -0.26))


def render_board(board: Sequence[str]) -> str:
    labels = [value.upper() if value else str(index + 1) for index, value in enumerate(board)]
    return "\n---+---+---\n".join(
        " {} | {} | {} ".format(*labels[start:start + 3]) for start in (0, 3, 6)
    )


class TicTacToeRobot(Node):
    def __init__(self) -> None:
        super().__init__("tic_tac_toe_robot")
        self.declare_parameter("stable_frames", 3)
        self.declare_parameter("perception_timeout", 8.0)
        self.declare_parameter("motion_duration", 1.25)
        self.declare_parameter("gripper_duration", 0.45)

        self.bridge = CvBridge()
        package_share = get_package_share_directory("nuovo_progetto")
        robot_description = xacro.process_file(
            str(package_share + "/urdf/panda.urdf.xacro"),
            mappings={"controllers_file": str(package_share + "/config/controllers.yaml")},
        ).toxml().replace('<mimic joint="panda_finger_joint1"/>', "")
        self.chain = SerialChain(robot_description)
        if self.chain.joint_names != ARM_JOINTS:
            raise RuntimeError("Unexpected Panda chain: " + ", ".join(self.chain.joint_names))
        self.grasp_rotation = self.chain.forward(HOME)[:3, :3]
        self.positions_lock = threading.Lock()
        self.current_positions: Optional[np.ndarray] = None
        self.attachment_condition = threading.Condition()
        self.attachment_states = {f"o_piece_{index}": None for index in range(1, 6)}
        self.board_condition = threading.Condition()
        self.recent_boards: deque[Tuple[str, ...]] = deque(
            maxlen=int(self.get_parameter("stable_frames").value)
        )
        self.stable_board: Optional[Tuple[str, ...]] = None
        self.started = False
        self.stopping = False

        self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, qos_profile_sensor_data
        )
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 20)
        self.annotated_pub = self.create_publisher(Image, "/tic_tac_toe/annotated", 1)
        self.board_pub = self.create_publisher(String, "/tic_tac_toe/board_state", 10)
        self.status_pub = self.create_publisher(String, "/tic_tac_toe/status", 10)
        self.pose_client = self.create_client(
            SetEntityPose, "/world/tic_tac_toe/set_pose"
        )
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory, "/eef_controller/follow_joint_trajectory"
        )
        self.attach_pubs = {}
        self.detach_pubs = {}
        for index in range(1, 6):
            name = f"o_piece_{index}"
            self.attach_pubs[name] = self.create_publisher(Empty, f"/{name}/attach", 1)
            self.detach_pubs[name] = self.create_publisher(Empty, f"/{name}/detach", 1)
            self.create_subscription(
                String,
                f"/{name}/state",
                lambda message, piece=name: self.attachment_state_callback(piece, message),
                1,
            )
        self.get_logger().info("Waiting for a stable overhead-camera view...")

    def joint_state_callback(self, message: JointState) -> None:
        values = dict(zip(message.name, message.position))
        if all(name in values for name in ARM_JOINTS):
            with self.positions_lock:
                self.current_positions = np.array([values[name] for name in ARM_JOINTS])

    def attachment_state_callback(self, name: str, message: String) -> None:
        with self.attachment_condition:
            self.attachment_states[name] = message.data == "attached"
            self.attachment_condition.notify_all()

    def publish_status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def image_callback(self, message: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            detection = detect_board(image)
        except Exception as exc:  # malformed image should not kill the game node
            self.get_logger().warning(f"Camera frame could not be processed: {exc}")
            return
        if detection is None:
            return

        annotated = self.bridge.cv2_to_imgmsg(detection.annotated, encoding="bgr8")
        annotated.header = message.header
        self.annotated_pub.publish(annotated)
        self.board_pub.publish(String(data="".join(cell or "-" for cell in detection.board)))

        with self.board_condition:
            self.recent_boards.append(detection.board)
            if len(self.recent_boards) == self.recent_boards.maxlen and len(set(self.recent_boards)) == 1:
                changed = self.stable_board != detection.board
                self.stable_board = detection.board
                self.board_condition.notify_all()
                if changed and not self.started:
                    self.started = True
                    threading.Thread(target=self.game_loop, daemon=True).start()

    def wait_for_board(
        self, expected: Optional[Tuple[str, ...]] = None, timeout: Optional[float] = None
    ) -> Tuple[str, ...]:
        deadline = time.monotonic() + (
            timeout if timeout is not None else float(self.get_parameter("perception_timeout").value)
        )
        with self.board_condition:
            while rclpy.ok() and not self.stopping:
                if self.stable_board is not None and (expected is None or self.stable_board == expected):
                    return self.stable_board
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError("camera did not confirm the expected board state")
                self.board_condition.wait(min(remaining, 0.25))
        raise RuntimeError("node is shutting down")

    @staticmethod
    def _wait_future(future, timeout: float, operation: str = "ROS operation"):
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout):
            raise TimeoutError(f"{operation} timed out after {timeout:.1f} seconds")
        exception = future.exception()
        if exception is not None:
            raise exception
        return future.result()

    @staticmethod
    def piece_pose_request(name: str, xyz: Iterable[float]) -> SetEntityPose.Request:
        request = SetEntityPose.Request()
        request.entity.name = name
        request.entity.type = Entity.MODEL
        x, y, z = xyz
        request.pose.position.x = float(x)
        request.pose.position.y = float(y)
        request.pose.position.z = float(z)
        request.pose.orientation.w = 1.0
        return request

    def set_piece_pose(self, name: str, xyz: Iterable[float]) -> None:
        if not self.pose_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Gazebo set_pose service is unavailable")
        response = self._wait_future(
            self.pose_client.call_async(self.piece_pose_request(name, xyz)),
            8.0,
            f"Gazebo pose update for {name}",
        )
        if not response.success:
            raise RuntimeError(f"Gazebo rejected pose update for {name}")

    def set_piece_attachment(self, name: str, attached: bool) -> None:
        publisher = self.attach_pubs[name] if attached else self.detach_pubs[name]
        connection_deadline = time.monotonic() + 5.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < connection_deadline:
            time.sleep(0.05)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError(f"Gazebo attachment bridge is unavailable for {name}")

        def publish_and_wait(command_publisher, expected: bool, timeout: float) -> bool:
            with self.attachment_condition:
                self.attachment_states[name] = None
            command_publisher.publish(Empty())
            deadline = time.monotonic() + timeout
            with self.attachment_condition:
                while self.attachment_states[name] is not expected:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        return False
                    self.attachment_condition.wait(min(remaining, 0.1))
                return True

        if publish_and_wait(publisher, attached, 1.0):
            return

        # The plugin reports only transitions, not its current state. On a game
        # restart the requested state may already be active, so force one
        # opposite transition and then retry the requested command.
        opposite = self.detach_pubs[name] if attached else self.attach_pubs[name]
        if not publish_and_wait(opposite, not attached, 2.0):
            raise TimeoutError(f"Gazebo attachment state is unavailable for {name}")
        if not publish_and_wait(publisher, attached, 2.0):
            raise TimeoutError(
                f"Gazebo did not {'attach' if attached else 'detach'} {name}"
            )

    def send_trajectory(
        self, client: ActionClient, joints: Sequence[str], positions: Sequence[float], duration: float
    ) -> None:
        if not client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("ros2_control trajectory action is unavailable")
        trajectory = JointTrajectory(joint_names=list(joints))
        point = JointTrajectoryPoint(positions=list(positions))
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        trajectory.points.append(point)
        goal = FollowJointTrajectory.Goal(trajectory=trajectory)
        handle = self._wait_future(client.send_goal_async(goal), 10.0, "trajectory acceptance")
        if not handle.accepted:
            raise RuntimeError("controller rejected trajectory")
        # Rendering can make Gazebo run well below real time. The controller
        # evaluates trajectory time in simulation time, so allow ample wall time.
        result = self._wait_future(
            handle.get_result_async(), duration + 35.0, "trajectory execution"
        )
        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f"trajectory failed with code {result.result.error_code}")

    def move_arm(self, xyz: Optional[Tuple[float, float, float]] = None, duration: Optional[float] = None) -> None:
        if xyz is None:
            positions = HOME
        else:
            with self.positions_lock:
                seed = HOME.copy() if self.current_positions is None else self.current_positions.copy()
            # Prefer the current posture. Pulling each waypoint toward HOME made
            # the redundant seventh axis rotate the wrist during lateral moves.
            positions = self.chain.inverse(
                xyz,
                self.grasp_rotation,
                seed,
                preferred=seed,
                preference_weights=np.array([1.0, 1.0, 1.0, 1.0, 3.0, 1.0, 12.0]),
            )
        self.send_trajectory(
            self.arm_client, ARM_JOINTS, positions,
            duration or float(self.get_parameter("motion_duration").value),
        )

    def move_gripper(self, opened: bool) -> None:
        value = GRIPPER_OPEN_POSITION if opened else O_GRASP_FINGER_POSITION
        duration = float(self.get_parameter("gripper_duration").value)
        self.send_trajectory(self.gripper_client, GRIPPER_JOINTS, [value, value], duration)

    def reset_pieces(self) -> None:
        for index, (x, y) in enumerate(X_SUPPLY, start=1):
            self.set_piece_pose(f"x_piece_{index}", (x, y, X_SUPPLY_PIECE_Z))
        for index, (x, y) in enumerate(O_SUPPLY, start=1):
            name = f"o_piece_{index}"
            self.set_piece_attachment(name, False)
            self.set_piece_pose(name, (x, y, O_SUPPLY_PIECE_Z))

    def place_robot_piece(self, piece_index: int, cell: int) -> None:
        # Consume the row from its far end: O5, O4, O3, O2, then O1.
        supply_index = len(O_SUPPLY) - 1 - piece_index
        piece_name = f"o_piece_{supply_index + 1}"
        sx, sy = O_SUPPLY[supply_index]
        tx, ty = CELL_POSITIONS[cell]
        tz = O_BOARD_PIECE_Z
        self.publish_status(f"Robot is placing O in cell {cell + 1}")
        base_duration = float(self.get_parameter("motion_duration").value)

        def motion_time(scale: float) -> float:
            return max(0.6, base_duration * scale)

        try:
            self.move_gripper(True)
            self.move_arm((sx, sy, 0.24))
            self.move_arm((sx, sy, SUPPLY_GRASP_Z), motion_time(0.75))
            self.move_gripper(False)
            self.set_piece_attachment(piece_name, True)
            self.move_arm((sx, sy, 0.24), motion_time(0.80))
            self.move_arm((tx, ty, 0.24))
            self.move_arm((tx, ty, BOARD_GRASP_Z), motion_time(0.75))
            self.move_gripper(True)
            self.set_piece_attachment(piece_name, False)
            # Correct only the final resting pose; transport itself is handled
            # by Gazebo's native fixed joint without asynchronous pose updates.
            self.set_piece_pose(piece_name, (tx, ty, tz))
            self.move_arm((tx, ty, 0.24), motion_time(0.80))
        finally:
            # HOME is a safety invariant after every robot turn, including failures.
            if self.attachment_states[piece_name]:
                self.set_piece_attachment(piece_name, False)
            self.move_arm(None, base_duration)

    def prompt_move(self, board: Tuple[str, ...]) -> int:
        while rclpy.ok():
            print("\n" + render_board(board), flush=True)
            try:
                raw = input("Your move (X), choose cell 1-9: ").strip()
            except EOFError as exc:
                raise RuntimeError("standard input closed; run this node in an interactive terminal") from exc
            if raw.isdigit() and 1 <= int(raw) <= 9 and not board[int(raw) - 1]:
                return int(raw) - 1
            print("Invalid or occupied cell. Please enter an available number from 1 to 9.", flush=True)
        raise RuntimeError("ROS is shutting down")

    def game_loop(self) -> None:
        try:
            self.publish_status("Initialising pieces and controllers")
            self.reset_pieces()
            self.move_gripper(True)
            self.move_arm(None, 1.0)
            board = self.wait_for_board(tuple("" for _ in range(9)), timeout=12.0)
            x_count = 0
            o_count = 0
            self.publish_status("New game: you are X and move first")

            while rclpy.ok() and not self.stopping:
                human_move = self.prompt_move(board)
                expected = list(board)
                expected[human_move] = "x"
                x, y = CELL_POSITIONS[human_move]
                z = X_BOARD_PIECE_Z
                self.set_piece_pose(f"x_piece_{x_count + 1}", (x, y, z))
                x_count += 1
                board = self.wait_for_board(tuple(expected))
                result = game_result(board)
                if result is not None:
                    break

                robot_move = choose_best_move(board)
                expected = list(board)
                expected[robot_move] = "o"
                self.place_robot_piece(o_count, robot_move)
                o_count += 1
                board = self.wait_for_board(tuple(expected))
                print("\n" + render_board(board), flush=True)
                result = game_result(board)
                if result is not None:
                    break

            result = game_result(board)
            message = {"x": "You win!", "o": "Robot wins!", "draw": "Draw game."}.get(result, "Game stopped.")
            self.publish_status(f"GAME OVER: {message}")
            print(f"\n{render_board(board)}\n{message}", flush=True)
        except Exception as exc:
            self.get_logger().error(f"Game aborted: {exc}")
            self.publish_status(f"ERROR: {exc}")

    def destroy_node(self):
        self.stopping = True
        with self.board_condition:
            self.board_condition.notify_all()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TicTacToeRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
