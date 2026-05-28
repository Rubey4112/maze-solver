from __future__ import annotations

import time
import sys
from collections import deque
from typing import List, Optional, Tuple

from robot import DIR_ORDER, DIR_TO_VEC, Direction, RobotController

sys.path.append('/home/pi/TurboPi/')

try:
	from HiwonderSDK.Sonar import Sonar
	from HiwonderSDK.mecanum import MecanumChassis
except Exception as exc:
	raise RuntimeError("Run this file on the TurboPi with the SDK installed.") from exc


CELL_MM = 180
FRONT_WALL_MM = 120
FORWARD_SPEED = 120
FORWARD_TIME_SEC = 0.9
TURN_RATE = 2.2
TURN_90_TIME_SEC = 0.55


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


def relative_direction(current: Direction, target: Direction) -> str:
	current_idx = DIR_ORDER.index(current)
	target_idx = DIR_ORDER.index(target)
	delta = (target_idx - current_idx) % len(DIR_ORDER)
	if delta == 0:
		return "F"
	if delta == 1:
		return "R"
	if delta == 3:
		return "L"
	return "B"


class TurboPiHardware:
	def __init__(self) -> None:
		self.sonar = Sonar()
		self.chassis = MecanumChassis()

	def read_front_wall(self) -> bool:
		dist_mm = self.sonar.getDistance()
		return dist_mm <= FRONT_WALL_MM

	def move_forward_one_cell(self) -> None:
		self.chassis.set_velocity(FORWARD_SPEED, 90, 0)
		time.sleep(FORWARD_TIME_SEC)
		self.chassis.reset_motors()

	def turn_left_90(self) -> None:
		self.chassis.set_velocity(0, 0, -TURN_RATE)
		time.sleep(TURN_90_TIME_SEC)
		self.chassis.reset_motors()

	def turn_right_90(self) -> None:
		self.chassis.set_velocity(0, 0, TURN_RATE)
		time.sleep(TURN_90_TIME_SEC)
		self.chassis.reset_motors()

	def turn_back_180(self) -> None:
		self.chassis.set_velocity(0, 0, TURN_RATE)
		time.sleep(TURN_90_TIME_SEC * 2)
		self.chassis.reset_motors()


class TurboPiRunner:
	def __init__(self, robot: RobotController) -> None:
		self.robot = robot
		self.hw = TurboPiHardware()

	def step(self, goal_xy: Tuple[int, int]) -> bool:
		if (self.robot.pose.x, self.robot.pose.y) == goal_xy:
			return False

		distances = compute_distances(self.robot, goal_xy, allow_unknown=True)
		next_dir = self.robot.choose_next_move(distances, allow_unknown=True)
		if next_dir is None:
			return False

		turn = relative_direction(self.robot.pose.direction, next_dir)
		if turn == "L":
			self.hw.turn_left_90()
			self.robot.turn_left()
		elif turn == "R":
			self.hw.turn_right_90()
			self.robot.turn_right()
		elif turn == "B":
			self.hw.turn_back_180()
			self.robot.turn_back()

		is_wall = self.hw.read_front_wall()
		self.robot.sense_front(is_wall)
		if is_wall:
			return True

		self.hw.move_forward_one_cell()
		self.robot.move_forward(allow_unknown=False)
		return True


if __name__ == "__main__":
	size = 5
	start_xy = (0, 4)
	goal_xy = (4, 0)

	robot = RobotController(
		width=size,
		height=size,
		start=start_xy,
		direction=Direction.NORTH,
	)
	runner = TurboPiRunner(robot)

	while runner.step(goal_xy):
		time.sleep(0.1)
