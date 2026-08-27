import cv2
import numpy as np

from tic_tac_toe.vision import detect_board


def test_detects_synthetic_red_x_and_blue_o():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    image[:] = (30, 75, 130)  # brown table
    cv2.rectangle(image, (180, 100), (460, 380), (205, 220, 225), -1)
    for offset in (273, 367):
        cv2.line(image, (offset, 100), (offset, 380), (20, 20, 20), 6)
    for offset in (193, 287):
        cv2.line(image, (180, offset), (460, offset), (20, 20, 20), 6)
    cv2.line(image, (202, 122), (252, 172), (0, 0, 255), 12)
    cv2.line(image, (252, 122), (202, 172), (0, 0, 255), 12)
    cv2.circle(image, (320, 240), 28, (255, 0, 0), 12)

    detection = detect_board(image)

    assert detection is not None
    assert detection.board[0] == "x"
    assert detection.board[4] == "o"
    assert sum(bool(cell) for cell in detection.board) == 2


def test_returns_none_without_a_board():
    assert detect_board(np.zeros((240, 320, 3), dtype=np.uint8)) is None
