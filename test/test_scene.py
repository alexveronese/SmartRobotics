from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


def _numbers(text):
    return [float(value) for value in text.split()]


def test_table_height_and_panda_base_clearance():
    root = ET.parse(Path(__file__).parents[1] / "worlds" / "tic_tac_toe.sdf").getroot()
    world = root.find("world")
    models = {model.attrib["name"]: model for model in world.findall("model")}

    table = models["table"]
    table_pose = _numbers(table.findtext("pose"))
    table_size = _numbers(table.find("link/collision/geometry/box/size").text)
    table_top = table_pose[2] + table_size[2] / 2.0
    table_near_edge = table_pose[0] - table_size[0] / 2.0
    panda_base_radius = float(models["robot_pedestal"].findtext("link/collision/geometry/cylinder/radius"))

    assert table_top == pytest.approx(0.0)
    assert table_near_edge == pytest.approx(0.205)
    assert panda_base_radius == pytest.approx(0.16)
    assert table_near_edge - panda_base_radius == pytest.approx(0.045)


def test_board_and_piece_heights_follow_table_surface():
    root = ET.parse(Path(__file__).parents[1] / "worlds" / "tic_tac_toe.sdf").getroot()
    world = root.find("world")
    models = {model.attrib["name"]: model for model in world.findall("model")}
    board_collision = models["board"].find("link/collision")
    board_z = _numbers(board_collision.findtext("pose"))[2]
    board_thickness = _numbers(board_collision.findtext("geometry/box/size"))[2]
    piece_z = _numbers(models["o_piece_1"].findtext("pose"))[2]

    assert board_z + board_thickness / 2.0 == pytest.approx(0.016)
    assert piece_z == pytest.approx(0.007)


def test_o_pieces_are_physical_rings_with_clear_supply_spacing():
    root = ET.parse(Path(__file__).parents[1] / "worlds" / "tic_tac_toe.sdf").getroot()
    models = {model.attrib["name"]: model for model in root.find("world").findall("model")}
    pieces = [models[f"o_piece_{index}"] for index in range(1, 6)]

    for piece in pieces:
        link = piece.find("link")
        collisions = link.findall("collision")
        visuals = link.findall("visual")
        assert len(collisions) == 12
        assert len(visuals) == 12
        assert piece.findtext("static") == "true"
        assert all(item.find("geometry/box") is not None for item in collisions + visuals)
        assert link.find("visual[@name='inner']") is None

    positions = [_numbers(piece.findtext("pose"))[:2] for piece in pieces]
    minimum_distance = min(
        ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5
        for index, (ax, ay) in enumerate(positions)
        for bx, by in positions[index + 1:]
    )
    assert minimum_distance == pytest.approx(0.08)
