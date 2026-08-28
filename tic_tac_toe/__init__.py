"""Core logic for the camera-guided tic-tac-toe demo."""

from .game import (
    Board,
    best_of_three_result,
    best_of_three_starter,
    choose_best_move,
    game_result,
)

__all__ = [
    "Board",
    "best_of_three_result",
    "best_of_three_starter",
    "choose_best_move",
    "game_result",
]
