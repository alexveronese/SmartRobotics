from functools import lru_cache
from pathlib import Path
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
        (0.63 - row * 0.10, 0.10 - col * 0.10, 0.032)
        for row in range(3) for col in range(3)
    ]
    points += [(x, y, 0.016) for x in (0.31, 0.39, 0.47) for y in (-0.29, -0.21)]
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
