from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import logging
from typing import Dict, Iterable, List, Optional, Tuple


class Direction(str, Enum):
	NORTH = "N"
	EAST = "E"
	SOUTH = "S"
	WEST = "W"


DIR_ORDER: List[Direction] = [Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST]
DIR_TO_VEC: Dict[Direction, Tuple[int, int]] = {
	Direction.NORTH: (0, -1),
	Direction.EAST: (1, 0),
	Direction.SOUTH: (0, 1),
	Direction.WEST: (-1, 0),
}
OPPOSITE: Dict[Direction, Direction] = {
	Direction.NORTH: Direction.SOUTH,
	Direction.EAST: Direction.WEST,
	Direction.SOUTH: Direction.NORTH,
	Direction.WEST: Direction.EAST,
}


@dataclass
class Pose:
	x: int
	y: int
	direction: Direction


class RobotController:
	"""
	Minimal controller for a grid maze robot.

	- Front sonar only: call sense_front(is_wall) each step.
	- Cardinal orientation: tracked internally and updated on turns.
	- Turn in-place + move forward: use turn_left/right/back + move_forward.
	"""

	def __init__(self, width: int, height: int, start: Tuple[int, int], direction: Direction) -> None:
		self.width = width
		self.height = height
		self.pose = Pose(start[0], start[1], direction)

		# walls[y][x][dir] -> None unknown, True wall, False open
		self.walls: List[List[Dict[Direction, Optional[bool]]]] = [
			[self._new_cell() for _ in range(width)] for _ in range(height)
		]
		self._logger = logging.getLogger(__name__)

	def _new_cell(self) -> Dict[Direction, Optional[bool]]:
		return {d: None for d in DIR_ORDER}

	def in_bounds(self, x: int, y: int) -> bool:
		return 0 <= x < self.width and 0 <= y < self.height

	def sense_front(self, is_wall: bool) -> None:
		"""Update the map using a front sonar reading."""
		x, y, direction = self.pose.x, self.pose.y, self.pose.direction
		self.walls[y][x][direction] = is_wall

		dx, dy = DIR_TO_VEC[direction]
		nx, ny = x + dx, y + dy
		if self.in_bounds(nx, ny):
			self.walls[ny][nx][OPPOSITE[direction]] = is_wall

		self._logger.info(
			"Front sensor at (%d,%d) facing %s: wall=%s",
			x,
			y,
			direction.value,
			is_wall,
		)
		self._logger.info("Current map:\n%s", self._format_map())

	def turn_left(self) -> None:
		idx = (DIR_ORDER.index(self.pose.direction) - 1) % len(DIR_ORDER)
		self.pose.direction = DIR_ORDER[idx]

	def turn_right(self) -> None:
		idx = (DIR_ORDER.index(self.pose.direction) + 1) % len(DIR_ORDER)
		self.pose.direction = DIR_ORDER[idx]

	def turn_back(self) -> None:
		idx = (DIR_ORDER.index(self.pose.direction) + 2) % len(DIR_ORDER)
		self.pose.direction = DIR_ORDER[idx]

	def move_forward(self, allow_unknown: bool = True) -> bool:
		"""Move one cell forward if possible. Returns True if moved."""
		x, y, direction = self.pose.x, self.pose.y, self.pose.direction
		wall_state = self.walls[y][x][direction]
		if wall_state is True:
			return False
		if wall_state is None and not allow_unknown:
			return False

		dx, dy = DIR_TO_VEC[direction]
		nx, ny = x + dx, y + dy
		if not self.in_bounds(nx, ny):
			return False

		self.pose.x = nx
		self.pose.y = ny
		return True

	def neighbor_states(self) -> Iterable[Tuple[Direction, int, int, Optional[bool]]]:
		x, y = self.pose.x, self.pose.y
		for direction in DIR_ORDER:
			dx, dy = DIR_TO_VEC[direction]
			nx, ny = x + dx, y + dy
			if self.in_bounds(nx, ny):
				yield direction, nx, ny, self.walls[y][x][direction]

	def choose_next_move(
		self,
		distances: List[List[Optional[int]]],
		allow_unknown: bool = True,
		tie_break: Tuple[str, str, str, str] = ("F", "R", "L", "B"),
	) -> Optional[Direction]:
		"""
		Pick a neighbor direction based on floodfill distances.

		distances[y][x] should be an int or None for unreachable/unknown.
		"""
		x, y = self.pose.x, self.pose.y
		candidates: List[Tuple[int, int, Direction]] = []

		for direction, nx, ny, wall_state in self.neighbor_states():
			if wall_state is True:
				continue
			if wall_state is None and not allow_unknown:
				continue

			cell_distance = distances[ny][nx]
			if cell_distance is None:
				continue

			score = self._tie_break_score(direction, tie_break)
			candidates.append((cell_distance, score, direction))

		if not candidates:
			return None

		candidates.sort(key=lambda item: (item[0], item[1]))
		return candidates[0][2]

	def _tie_break_score(self, direction: Direction, tie_break: Tuple[str, str, str, str]) -> int:
		rel = self._relative_direction(direction)
		return tie_break.index(rel)

	def _relative_direction(self, direction: Direction) -> str:
		current_idx = DIR_ORDER.index(self.pose.direction)
		target_idx = DIR_ORDER.index(direction)
		delta = (target_idx - current_idx) % len(DIR_ORDER)
		if delta == 0:
			return "F"
		if delta == 1:
			return "R"
		if delta == 3:
			return "L"
		return "B"

	def turn_towards(self, direction: Direction) -> None:
		rel = self._relative_direction(direction)
		if rel == "R":
			self.turn_right()
		elif rel == "L":
			self.turn_left()
		elif rel == "B":
			self.turn_back()

	def _format_map(self) -> str:
		def wall_char(state: Optional[bool], horizontal: bool) -> str:
			if state is True:
				return "-" if horizontal else "|"
			if state is False:
				return " "
			return "?"

		def pose_marker(x: int, y: int) -> str:
			if self.pose.x == x and self.pose.y == y:
				if self.pose.direction == Direction.NORTH:
					return "^"
				if self.pose.direction == Direction.EAST:
					return ">"
				if self.pose.direction == Direction.SOUTH:
					return "v"
				return "<"
			return " "

		lines: List[str] = []
		for y in range(self.height):
			top = "+"
			for x in range(self.width):
				top += wall_char(self.walls[y][x][Direction.NORTH], True) * 3 + "+"
			lines.append(top)

			mid = ""
			for x in range(self.width):
				mid += wall_char(self.walls[y][x][Direction.WEST], False)
				mid += " " + pose_marker(x, y) + " "
			mid += wall_char(self.walls[y][self.width - 1][Direction.EAST], False)
			lines.append(mid)

		bottom = "+"
		for x in range(self.width):
			bottom += wall_char(self.walls[self.height - 1][x][Direction.SOUTH], True) * 3 + "+"
		lines.append(bottom)
		return "\n".join(lines)

