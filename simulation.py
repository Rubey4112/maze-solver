from __future__ import annotations

from collections import deque
from typing import List, Optional, Tuple

from maze import Maze
from robot import DIR_ORDER, DIR_TO_VEC, Direction, RobotController


def compute_distances(
	robot: RobotController,
	goal_xy: Tuple[int, int],
	allow_unknown: bool = True,
) -> List[List[Optional[int]]]:
	width, height = robot.width, robot.height
	dist: List[List[Optional[int]]] = [[None for _ in range(width)] for _ in range(height)]
	gx, gy = goal_xy
	if not robot.in_bounds(gx, gy):
		return dist

	dist[gy][gx] = 0
	queue: deque[Tuple[int, int]] = deque([(gx, gy)])
	while queue:
		x, y = queue.popleft()
		for direction in DIR_ORDER:
			wall_state = robot.walls[y][x][direction]
			if wall_state is True:
				continue
			if wall_state is None and not allow_unknown:
				continue

			dx, dy = DIR_TO_VEC[direction]
			nx, ny = x + dx, y + dy
			if not robot.in_bounds(nx, ny):
				continue

			if dist[ny][nx] is None:
				dist[ny][nx] = dist[y][x] + 1
				queue.append((nx, ny))
	return dist


def render_known_map(
	robot: RobotController,
	goal_xy: Optional[Tuple[int, int]] = None,
	path_xy: Optional[List[Tuple[int, int]]] = None,
	planned_path: Optional[List[Tuple[int, int]]] = None,
) -> str:
	lines: List[str] = []
	path_set = set(path_xy or [])
	planned_set = set(planned_path or [])
	for y in range(robot.height):
		top = []
		mid = []
		for x in range(robot.width):
			cell = robot.walls[y][x]
			top.append("+")
			if y == 0 or cell[Direction.NORTH] is True:
				top.append("---")
			elif cell[Direction.NORTH] is False:
				top.append("   ")
			else:
				top.append("   ")

			if x == 0 or cell[Direction.WEST] is True:
				mid.append("|")
			elif cell[Direction.WEST] is False:
				mid.append(" ")
			else:
				mid.append(" ")

			if (x, y) == (robot.pose.x, robot.pose.y):
				mid.append(f" {robot.pose.direction.value} ")
			elif (x, y) in planned_set:
				mid.append(" * ")
			elif goal_xy is not None and (x, y) == goal_xy:
				mid.append(" G ")
			elif (x, y) in path_set:
				mid.append(" . ")
			else:
				mid.append("   ")
		top.append("+")

		mid.append("|")

		lines.append("".join(top))
		lines.append("".join(mid))

	bottom = []
	for x in range(robot.width):
		bottom.append("+")
		bottom.append("---")
	bottom.append("+")
	lines.append("".join(bottom))
	return "\n".join(lines)


def render_side_by_side(left: str, right: str, gap: int = 4) -> str:
	left_lines = left.splitlines()
	right_lines = right.splitlines()
	left_width = max((len(line) for line in left_lines), default=0)
	space = " " * gap
	max_lines = max(len(left_lines), len(right_lines))
	result: List[str] = []

	for idx in range(max_lines):
		l = left_lines[idx] if idx < len(left_lines) else ""
		r = right_lines[idx] if idx < len(right_lines) else ""
		result.append(l.ljust(left_width) + space + r)
	return "\n".join(result)


def planned_path_to_goal(
	robot: RobotController,
	distances: List[List[Optional[int]]],
	goal_xy: Tuple[int, int],
) -> List[Tuple[int, int]]:
	path: List[Tuple[int, int]] = []
	start = (robot.pose.x, robot.pose.y)
	current = start
	visited = set([current])

	while current != goal_xy:
		x, y = current
		current_dist = distances[y][x]
		if current_dist is None:
			break

		next_cell: Optional[Tuple[int, int]] = None
		for direction in DIR_ORDER:
			wall_state = robot.walls[y][x][direction]
			if wall_state is True:
				continue

			dx, dy = DIR_TO_VEC[direction]
			nx, ny = x + dx, y + dy
			if not robot.in_bounds(nx, ny):
				continue
			if distances[ny][nx] is None:
				continue
			if distances[ny][nx] >= current_dist:
				continue

			next_cell = (nx, ny)
			break

		if next_cell is None or next_cell in visited:
			break

		path.append(next_cell)
		visited.add(next_cell)
		current = next_cell

	return path


def simulate() -> None:
	size = 5
	maze = Maze(size)
	maze.set_wall(1, 1, "E")
	maze.set_wall(1, 2, "S")
	maze.set_wall(2, 2, "E")
	maze.set_wall(3, 1, "N")
	maze.set_wall(0, 0, "S")
	maze.set_wall(0, 2, "S")
	maze.set_wall(1, 3, "S")
	maze.set_wall(2, 3, "E")
	maze.set_wall(3, 3, "E")
	maze.set_wall(4, 1, "E")
	maze.set_wall(3, 0, "E")
	maze.set_wall(2, 1, "S")
	maze.set_wall(4, 2, "N")
	maze.set_wall(3, 0, "N")

	start_rc = (4, 0)
	goal_rc = (0, 4)

	robot = RobotController(
		width=size,
		height=size,
		start=(start_rc[1], start_rc[0]),
		direction=Direction.NORTH,
	)
	goal_xy = (goal_rc[1], goal_rc[0])

	path_rc: List[Tuple[int, int]] = [start_rc]
	path_xy: List[Tuple[int, int]] = [(start_rc[1], start_rc[0])]
	max_steps = size * size * 4

	for step in range(1, max_steps + 1):
		if (robot.pose.x, robot.pose.y) == goal_xy:
			break

		distances = compute_distances(robot, goal_xy, allow_unknown=True)
		next_dir = robot.choose_next_move(distances, allow_unknown=True)
		if next_dir is None:
			print("No reachable move from current knowledge.")
			break

		robot.turn_towards(next_dir)

		r, c = robot.pose.y, robot.pose.x
		is_wall = maze.grid[r][c].walls[robot.pose.direction.value]
		robot.sense_front(is_wall)

		if not robot.move_forward(allow_unknown=False):
			continue

		path_rc.append((robot.pose.y, robot.pose.x))
		path_xy.append((robot.pose.x, robot.pose.y))

		print(f"\nStep {step}: pose=({robot.pose.y}, {robot.pose.x}) heading={robot.pose.direction.value}")
		lookahead = compute_distances(robot, goal_xy, allow_unknown=True)
		planned_path = planned_path_to_goal(robot, lookahead, goal_xy)
		known = render_known_map(robot, goal_xy=goal_xy, path_xy=path_xy, planned_path=planned_path)
		real = maze.render(start=(robot.pose.y, robot.pose.x), goal=goal_rc, path=path_rc)
		print(render_side_by_side("Known map:\n" + known, "Real maze:\n" + real))

	print("Simulated path:", path_rc)
	print(maze.render(start=start_rc, goal=goal_rc, path=path_rc))


if __name__ == "__main__":
	simulate()

