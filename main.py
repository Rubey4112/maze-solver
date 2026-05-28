from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

DIRS: List[Tuple[str, int, int]] = [
	("N", -1, 0),
	("E", 0, 1),
	("S", 1, 0),
	("W", 0, -1),
]
OPPOSITE: Dict[str, str] = {"N": "S", "S": "N", "E": "W", "W": "E"}


@dataclass
class Cell:
	walls: Dict[str, bool] = field(default_factory=lambda: {"N": False, "E": False, "S": False, "W": False})


class Maze:
	def __init__(self, size: int) -> None:
		self.size = size
		self.grid: List[List[Cell]] = [[Cell() for _ in range(size)] for _ in range(size)]
		self._add_boundary_walls()

	def __str__(self) -> str:
		lines: List[str] = []
		for r in range(self.size):
			top = []
			mid = []
			for c in range(self.size):
				cell = self.grid[r][c]
				top.append("+")
				top.append("---" if cell.walls["N"] else "   ")
				mid.append("|" if cell.walls["W"] else " ")
				mid.append("   ")
			top.append("+")
			mid.append("|" if self.grid[r][self.size - 1].walls["E"] else " ")
			lines.append("".join(top))
			lines.append("".join(mid))
		bottom = []
		for c in range(self.size):
			bottom.append("+")
			bottom.append("---" if self.grid[self.size - 1][c].walls["S"] else "   ")
		bottom.append("+")
		lines.append("".join(bottom))
		return "\n".join(lines)

	def _add_boundary_walls(self) -> None:
		for r in range(self.size):
			self.grid[r][0].walls["W"] = True
			self.grid[r][self.size - 1].walls["E"] = True
		for c in range(self.size):
			self.grid[0][c].walls["N"] = True
			self.grid[self.size - 1][c].walls["S"] = True

	def in_bounds(self, r: int, c: int) -> bool:
		return 0 <= r < self.size and 0 <= c < self.size

	def set_wall(self, r: int, c: int, direction: str, exists: bool = True) -> None:
		if direction not in OPPOSITE:
			raise ValueError(f"Invalid direction: {direction}")
		self.grid[r][c].walls[direction] = exists
		dr, dc = next((dr, dc) for d, dr, dc in DIRS if d == direction)
		nr, nc = r + dr, c + dc
		if self.in_bounds(nr, nc):
			self.grid[nr][nc].walls[OPPOSITE[direction]] = exists

	def neighbors(self, r: int, c: int) -> List[Tuple[int, int]]:
		result: List[Tuple[int, int]] = []
		for d, dr, dc in DIRS:
			if self.grid[r][c].walls[d]:
				continue
			nr, nc = r + dr, c + dc
			if self.in_bounds(nr, nc):
				result.append((nr, nc))
		return result

	def flood_fill(self, goal: Tuple[int, int]) -> List[List[int]]:
		dist = [[99 for _ in range(self.size)] for _ in range(self.size)]
		gr, gc = goal
		dist[gr][gc] = 0

		queue: deque[Tuple[int, int]] = deque([(gr, gc)])
		while queue:
			r, c = queue.popleft()
			for nr, nc in self.neighbors(r, c):
				if dist[nr][nc] > dist[r][c] + 1:
					dist[nr][nc] = dist[r][c] + 1
					queue.append((nr, nc))
		return dist

	def solve(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
		dist = self.flood_fill(goal)
		sr, sc = start
		if dist[sr][sc] == 99:
			return None

		path = [start]
		current = start
		while current != goal:
			r, c = current
			next_cell = None
			for nr, nc in self.neighbors(r, c):
				if dist[nr][nc] < dist[r][c]:
					next_cell = (nr, nc)
					break
			if next_cell is None:
				return None
			path.append(next_cell)
			current = next_cell
		return path


def _print_dist(dist: List[List[int]]) -> None:
	for row in dist:
		print(" ".join(f"{v:2}" for v in row))


if __name__ == "__main__":
	maze = Maze(5)
	maze.set_wall(1, 1, "E")
	maze.set_wall(1, 2, "S")
	maze.set_wall(2, 2, "E")
	maze.set_wall(3, 1, "N")

	start = (4, 0)
	goal = (0, 4)
	distances = maze.flood_fill(goal)
	path = maze.solve(start, goal)

	print("Distance grid:")
	_print_dist(distances)
	print("Path:", path)
	print(maze)
