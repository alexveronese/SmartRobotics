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
    pedestal = models["robot_pedestal"]
    pedestal_x = _numbers(pedestal.findtext("pose"))[0]
    panda_base_radius = float(pedestal.findtext("link/collision/geometry/cylinder/radius"))
    pedestal_far_edge = pedestal_x + panda_base_radius
    ground_size = _numbers(models["ground_plane"].findtext("link/collision/geometry/box/size"))
    ground_colour = _numbers(models["ground_plane"].findtext("link/visual/material/diffuse"))

    assert table_top == pytest.approx(0.0)
    assert table_near_edge == pytest.approx(0.245)
    assert panda_base_radius == pytest.approx(0.12)
    assert table_near_edge - pedestal_far_edge == pytest.approx(0.165)
    assert ground_size == pytest.approx([2.0, 2.0, 0.04])
    assert ground_colour == pytest.approx([0.22, 0.42, 0.25, 1.0])


def test_board_and_piece_heights_follow_table_surface():
    root = ET.parse(Path(__file__).parents[1] / "worlds" / "tic_tac_toe.sdf").getroot()
    world = root.find("world")
    models = {model.attrib["name"]: model for model in world.findall("model")}
    board_collision = models["board"].find("link/collision")
    board_z = _numbers(board_collision.findtext("pose"))[2]
    board_thickness = _numbers(board_collision.findtext("geometry/box/size"))[2]
    o_piece_z = _numbers(models["o_piece_1"].findtext("pose"))[2]
    x_piece_z = _numbers(models["x_piece_1"].findtext("pose"))[2]

    assert board_z + board_thickness / 2.0 == pytest.approx(0.016)
    assert o_piece_z == pytest.approx(0.015)
    assert x_piece_z == pytest.approx(0.015)


def test_pieces_use_matching_rows_and_graspable_geometry():
    root = ET.parse(Path(__file__).parents[1] / "worlds" / "tic_tac_toe.sdf").getroot()
    models = {model.attrib["name"]: model for model in root.find("world").findall("model")}
    pieces = [models[f"o_piece_{index}"] for index in range(1, 6)]

    for piece in pieces:
        link = piece.find("link")
        collisions = link.findall("collision")
        visuals = link.findall("visual")
        assert len(collisions) == 1
        assert len(visuals) == 1
        assert piece.findtext("static") in (None, "false")
        assert float(link.findtext("inertial/mass")) == pytest.approx(0.05)
        for item in collisions + visuals:
            cylinder = item.find("geometry/cylinder")
            assert float(cylinder.findtext("radius")) == pytest.approx(0.034)
            assert float(cylinder.findtext("length")) == pytest.approx(0.030)

    positions = [_numbers(piece.findtext("pose"))[:2] for piece in pieces]
    x_positions = [
        _numbers(models[f"x_piece_{index}"].findtext("pose"))[:2]
        for index in range(1, 6)
    ]
    for index in range(1, 6):
        x_piece = models[f"x_piece_{index}"]
        link = x_piece.find("link")
        assert x_piece.findtext("static") in (None, "false")
        assert _numbers(link.findtext("collision/geometry/box/size")) == pytest.approx(
            [0.068, 0.068, 0.030]
        )
        assert float(link.findtext("inertial/mass")) == pytest.approx(0.05)
        visuals = {visual.attrib["name"]: visual for visual in link.findall("visual")}
        assert set(visuals) == {"square", "x_a", "x_b"}
        assert _numbers(visuals["square"].findtext("material/diffuse")) == pytest.approx(
            [0.92, 0.91, 0.80, 1.0]
        )
        for name in ("x_a", "x_b"):
            assert _numbers(visuals[name].findtext("geometry/box/size")) == pytest.approx(
                [0.055, 0.009, 0.001]
            )
            assert _numbers(visuals[name].findtext("pose"))[2] == pytest.approx(0.0155)
            assert _numbers(visuals[name].findtext("material/diffuse")) == pytest.approx(
                [0.9, 0.03, 0.03, 1.0]
            )
    assert [position[0] for position in x_positions] == pytest.approx(
        [0.39, 0.48, 0.57, 0.66, 0.75]
    )
    assert [position[0] for position in positions] == pytest.approx(
        [0.39, 0.48, 0.57, 0.66, 0.75]
    )
    assert [position[1] for position in x_positions] == pytest.approx([0.26] * 5)
    assert [position[1] for position in positions] == pytest.approx([-0.26] * 5)
    minimum_distance = min(
        ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5
        for index, (ax, ay) in enumerate(positions)
        for bx, by in positions[index + 1:]
    )
    assert minimum_distance == pytest.approx(0.09)
