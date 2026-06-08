from ucollections import deque

DIRS = [
	("N", -1, 0),
	("E", 0, 1),
	("S", 1, 0),
	("W", 0, -1),
]
OPPOSITE = {"N": "S", "S": "N", "E": "W", "W": "E"}


class Cell:
	def __init__(self, walls=None):
		self.walls = walls or {"N": False, "E": False, "S": False, "W": False}


class Maze:
	def __init__(self, size):
		self.size = size
		self.grid = [[Cell() for _ in range(size)] for _ in range(size)]
		self._add_boundary_walls()

	def __str__(self):
		return self.render()

	def render(
		self,
		start=None,
		goal=None,
		path=None,
	):
		path_set = set(path or [])
		lines = []
		for r in range(self.size):
			top = []
			mid = []
			for c in range(self.size):
				cell = self.grid[r][c]
				top.append("+")
				top.append("---" if cell.walls["N"] else "   ")
				mid.append("|" if cell.walls["W"] else " ")
				if start == (r, c):
					mid.append(" S ")
				elif goal == (r, c):
					mid.append(" G ")
				elif (r, c) in path_set:
					mid.append(" . ")
				else:
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

	def _add_boundary_walls(self):
		for r in range(self.size):
			self.grid[r][0].walls["W"] = True
			self.grid[r][self.size - 1].walls["E"] = True
		for c in range(self.size):
			self.grid[0][c].walls["N"] = True
			self.grid[self.size - 1][c].walls["S"] = True

	def in_bounds(self, r, c):
		return 0 <= r < self.size and 0 <= c < self.size

	def set_wall(self, r, c, direction, exists=True):
		if direction not in OPPOSITE:
			raise ValueError(f"Invalid direction: {direction}")
		self.grid[r][c].walls[direction] = exists
		dr, dc = next((dr, dc) for d, dr, dc in DIRS if d == direction)
		nr, nc = r + dr, c + dc
		if self.in_bounds(nr, nc):
			self.grid[nr][nc].walls[OPPOSITE[direction]] = exists

	def neighbors(self, r, c):
		result = []
		for d, dr, dc in DIRS:
			if self.grid[r][c].walls[d]:
				continue
			nr, nc = r + dr, c + dc
			if self.in_bounds(nr, nc):
				result.append((nr, nc))
		return result

	def flood_fill(self, goal):
		dist = [[99 for _ in range(self.size)] for _ in range(self.size)]
		gr, gc = goal
		dist[gr][gc] = 0

		queue = deque([(gr, gc)], self.size * self.size)
		while queue:
			r, c = queue.popleft()
			for nr, nc in self.neighbors(r, c):
				if dist[nr][nc] > dist[r][c] + 1:
					dist[nr][nc] = dist[r][c] + 1
					queue.append((nr, nc))
		return dist

	def solve(self, start, goal):
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


	def _print_dist(dist):
	for row in dist:
		print(" ".join(f"{v:2}" for v in row))


if __name__ == "__main__":
	maze = Maze(5)
	maze.set_wall(1, 1, "E")
	maze.set_wall(1, 2, "S")
	maze.set_wall(2, 2, "E")
	maze.set_wall(3, 1, "N")
	maze.set_wall(0, 0, "S")
	

	start = (4, 0)
	goal = (0, 4)
	distances = maze.flood_fill(goal)
	path = maze.solve(start, goal)

	print("Distance grid:")
	_print_dist(distances)
	print("Path:", path)
	print(maze.render(start=start, goal=goal, path=path))
