"""Core logic for the camera-guided tic-tac-toe demo."""

from .game import Board, choose_best_move, game_result

__all__ = ["Board", "choose_best_move", "game_result"]
