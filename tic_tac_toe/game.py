"""Pure tic-tac-toe rules and an unbeatable minimax player."""

from __future__ import annotations

from typing import Iterable, Optional, Sequence, Tuple

Board = Tuple[str, ...]
WINNING_LINES = (
    (0, 1, 2), (3, 4, 5), (6, 7, 8),
    (0, 3, 6), (1, 4, 7), (2, 5, 8),
    (0, 4, 8), (2, 4, 6),
)
# This order makes equally-good play look natural and deterministic.
MOVE_PREFERENCE = (4, 0, 2, 6, 8, 1, 3, 5, 7)


def normalise_board(cells: Iterable[str]) -> Board:
    board = tuple(str(cell).lower() for cell in cells)
    if len(board) != 9 or any(cell not in ("", "x", "o") for cell in board):
        raise ValueError("a board must contain exactly nine '', 'x', or 'o' cells")
    return board


def game_result(cells: Sequence[str]) -> Optional[str]:
    """Return 'x', 'o', 'draw', or None while play can continue."""
    board = normalise_board(cells)
    for a, b, c in WINNING_LINES:
        if board[a] and board[a] == board[b] == board[c]:
            return board[a]
    return "draw" if all(board) else None


def _minimax(board: Board, turn: str, depth: int) -> int:
    result = game_result(board)
    if result == "o":
        return 10 - depth
    if result == "x":
        return depth - 10
    if result == "draw":
        return 0

    scores = []
    for move in MOVE_PREFERENCE:
        if board[move]:
            continue
        candidate = list(board)
        candidate[move] = turn
        scores.append(_minimax(tuple(candidate), "x" if turn == "o" else "o", depth + 1))
    return max(scores) if turn == "o" else min(scores)


def choose_best_move(cells: Sequence[str]) -> int:
    """Choose the robot's zero-based move. Raises if the position is terminal."""
    board = normalise_board(cells)
    if game_result(board) is not None:
        raise ValueError("cannot choose a move on a finished board")

    best_move = -1
    best_score = -100
    for move in MOVE_PREFERENCE:
        if board[move]:
            continue
        candidate = list(board)
        candidate[move] = "o"
        score = _minimax(tuple(candidate), "x", 0)
        if score > best_score:
            best_score = score
            best_move = move
    return best_move
