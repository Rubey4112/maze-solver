import time
from ucollections import deque

from robot import DIR_ORDER, DIR_TO_VEC, Direction, RobotController

try:
	from XRPLib.defaults import drivetrain, rangefinder, board
except Exception as exc:
	raise RuntimeError("Run this file on the XRP with XRPLib installed.") from exc


CELL_CM = 54.0
FRONT_WALL_CM = 30.2
STRAIGHT_EFFORT = 0.6
TURN_EFFORT = 0.5
STRAIGHT_TIMEOUT_SEC = 3.0
TURN_TIMEOUT_SEC = 2.0


def compute_distances(robot, goal_xy, allow_unknown=True):
	width, height = robot.width, robot.height
	dist = [[None for _ in range(width)] for _ in range(height)]
	gx, gy = goal_xy
	if not robot.in_bounds(gx, gy):
		return dist

	dist[gy][gx] = 0
	queue = deque([(gx, gy)], width * height)
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


def relative_direction(current, target):
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


class XRPHardware:
	def __init__(self):
		self.drivetrain = drivetrain
		self.rangefinder = rangefinder

	def read_front_wall(self):
		dist_cm = self.rangefinder.distance()
		return dist_cm <= FRONT_WALL_CM

	def move_forward_one_cell(self):
		self.drivetrain.straight(
			CELL_CM,
			max_effort=STRAIGHT_EFFORT,
			timeout=STRAIGHT_TIMEOUT_SEC,
		)

	def turn_left_90(self):
		self.drivetrain.turn(
			90,
			max_effort=TURN_EFFORT,
			timeout=TURN_TIMEOUT_SEC,
		)

	def turn_right_90(self):
		self.drivetrain.turn(
			-90,
			max_effort=TURN_EFFORT,
			timeout=TURN_TIMEOUT_SEC,
		)

	def turn_back_180(self):
		self.drivetrain.turn(
			180,
			max_effort=TURN_EFFORT,
			timeout=TURN_TIMEOUT_SEC,
		)


class XRPRunner:
	def __init__(self, robot):
		self.robot = robot
		self.hw = XRPHardware()

	def step(self, goal_xy):
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
		moved = self.robot.move_forward(allow_unknown=False)
		return True


if __name__ == "__main__":
	# Wait for button to avoid immediate motion on boot.
	board.wait_for_button()

	size = 3
	start_xy = (0, 2)
	goal_xy = (2, 0)

	robot = RobotController(
		width=size,
		height=size,
		start=start_xy,
		direction=Direction.NORTH,
	)
	runner = XRPRunner(robot)

	while runner.step(goal_xy):
		time.sleep(0.1)
