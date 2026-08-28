from functools import lru_cache
from pathlib import Path
import runpy
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro

from tic_tac_toe.game import choose_best_move, game_result
from tic_tac_toe.kinematics import SerialChain


def _panda_description():
    root = Path(__file__).parents[1]
    return xacro.process_file(
        str(root / "urdf" / "panda.urdf.xacro"),
        mappings={"controllers_file": "/tmp/controllers.yaml"},
    ).toxml().replace('<mimic joint="panda_finger_joint1"/>', "")


def test_panda_is_fixed_to_world():
    root = ET.fromstring(_panda_description())
    mount = root.find("joint[@name='panda_mount_joint']")
    assert mount is not None
    assert mount.attrib["type"] == "fixed"
    assert mount.find("parent").attrib["link"] == "world"
    assert mount.find("child").attrib["link"] == "panda_link0"


def test_panda_uses_native_detachable_joints_for_all_pieces():
    root = ET.fromstring(_panda_description())
    plugins = [
        plugin
        for plugin in root.findall("gazebo/plugin")
        if plugin.attrib["filename"] == "gz-sim-detachable-joint-system"
    ]

    assert len(plugins) == 10
    configured = {
        (plugin.findtext("child_model"), plugin.findtext("child_link"))
        for plugin in plugins
    }
    for mark in ("x", "o"):
        for index in range(1, 6):
            assert (f"{mark}_piece_{index}", f"{mark}_piece_link_{index}") in configured
    assert all(
        plugin.attrib["name"] == "gz::sim::systems::DetachableJoint"
        and plugin.findtext("parent_link") == "panda_link7"
        for plugin in plugins
    )


def test_result_detection():
    assert game_result(("x", "x", "x", "", "o", "", "o", "", "")) == "x"
    assert game_result(("x", "o", "x", "x", "o", "o", "o", "x", "x")) == "draw"
    assert game_result(("",) * 9) is None


def test_robot_takes_immediate_win_and_blocks():
    assert choose_best_move(("o", "o", "", "x", "x", "", "", "", "")) == 2
    assert choose_best_move(("x", "x", "", "", "o", "", "", "", "")) == 2


def test_minimax_robot_cannot_be_beaten():
    @lru_cache(maxsize=None)
    def human_turn(board):
        result = game_result(board)
        if result:
            return result != "x"
        for move, value in enumerate(board):
            if value:
                continue
            candidate = list(board)
            candidate[move] = "x"
            candidate = tuple(candidate)
            if game_result(candidate) == "x":
                return False
            if game_result(candidate) is None:
                robot_move = choose_best_move(candidate)
                reply = list(candidate)
                reply[robot_move] = "o"
                if not human_turn(tuple(reply)):
                    return False
        return True

    assert human_turn(("",) * 9)


def test_all_pick_and_place_coordinates_are_reachable():
    description = _panda_description()
    chain = SerialChain(description)
    home = np.array([0.0, -0.78, 0.0, -2.35, 0.0, 1.57, 0.78])
    grasp_rotation = chain.forward(home)[:3, :3]
    points = [
        (0.67 - row * 0.10, 0.10 - col * 0.10, 0.031)
        for row in range(3) for col in range(3)
    ]
    points.append((0.57, 0.0, 0.340))
    points += [
        (x, y, 0.015)
        for x in (0.39, 0.48, 0.57, 0.66, 0.75)
        for y in (-0.26, 0.26)
    ]
    for x, y, grasp_z in points:
        above = chain.inverse([x, y, 0.24], grasp_rotation, home, preferred=home)
        grasp = chain.inverse([x, y, grasp_z], grasp_rotation, above, preferred=home)
        assert len(grasp) == 7
        assert np.all(grasp > chain.lower)
        assert np.all(grasp < chain.upper)


def test_invalid_or_terminal_board_is_rejected():
    with pytest.raises(ValueError):
        choose_best_move(("x",) * 9)
    with pytest.raises(ValueError):
        game_result(("",) * 8)


def test_cleanup_restores_piece_one_last_and_normalizes_after_home():
    script = runpy.run_path(
        str(Path(__file__).parents[1] / "scripts" / "tic_tac_toe_game.py")
    )
    robot_class = script["TicTacToeRobot"]
    placed = [("x", 0, 0), ("o", 0, 4), ("x", 1, 6), ("x", 4, 7)]

    class FakeRobot:
        def __init__(self):
            self.events = []

        def publish_status(self, message):
            self.events.append(("status", message))

        def return_game_piece(self, mark, turn_index, cell, return_home=True):
            self.events.append(("return", mark, turn_index, cell, return_home))

        def move_arm(self, xyz=None, duration=None):
            self.events.append(("move", xyz))

        def get_parameter(self, _name):
            return type("Parameter", (), {"value": 1.25})()

        def set_piece_pose(self, name, xyz):
            self.events.append(("pose", name, xyz))

    robot = FakeRobot()
    robot_class.clear_board(robot, placed)

    returns = [event for event in robot.events if event[0] == "return"]
    assert [(event[1], event[2]) for event in returns] == [
        ("x", 0), ("o", 0), ("x", 1), ("x", 4)
    ]
    assert returns[-1][1:3] == ("x", 4)  # x_piece_1 is restored last.

    home_index = max(
        index for index, event in enumerate(robot.events)
        if event == ("move", None)
    )
    pose_events = [
        (index, event) for index, event in enumerate(robot.events)
        if event[0] == "pose"
    ]
    assert pose_events
    assert all(index > home_index for index, _ in pose_events)
    assert pose_events[-1][1] == ("pose", "x_piece_1", (0.39, 0.26, 0.015))
