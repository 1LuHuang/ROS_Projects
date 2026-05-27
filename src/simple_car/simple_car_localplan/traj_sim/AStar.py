import math
import heapq


class AStarPlanner:
    def __init__(self, resolution=0.05, safety_margin=0.2):
        self.resolution = resolution
        self.safety_margin = safety_margin

        self.car_radius = math.sqrt(0.6 ** 2 + 0.4 ** 2) / 2.0
        self.obs_radius = math.sqrt(1.0 ** 2 + 1.0 ** 2) / 2.0
        self.collision_radius = self.car_radius + self.obs_radius + self.safety_margin

        self.motion = [
            (1, 0, 1.0), (-1, 0, 1.0),
            (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
        ]

    def replan_local_segment(self, original_path, obstacles, buffer_distance=1.2):
        collision_indices = []

        for i, (x, y) in enumerate(original_path):
            for ox, oy in obstacles:
                if math.hypot(x - ox, y - oy) <= self.collision_radius:
                    collision_indices.append(i)
                    break

        if not collision_indices:
            print("No collision detected on original path.")
            return original_path, None, None

        first_hit = min(collision_indices)
        last_hit = max(collision_indices)

        path_resolution = self.estimate_path_resolution(original_path)
        buffer_idx = int(buffer_distance / path_resolution)

        start_idx = max(0, first_hit - buffer_idx)
        end_idx = min(len(original_path) - 1, last_hit + buffer_idx)

        start = original_path[start_idx]
        goal = original_path[end_idx]

        astar_segment = self.plan(start, goal, obstacles)

        if not astar_segment:
            print("A* failed. Return original path.")
            return original_path, start_idx, end_idx

        new_path = original_path[:start_idx] + astar_segment + original_path[end_idx:]

        print(f"collision radius = {self.collision_radius:.3f} m")
        print(f"first_hit = {first_hit}, last_hit = {last_hit}")
        print(f"start_idx = {start_idx}, end_idx = {end_idx}")
        print(f"A* segment length = {len(astar_segment)}")
        print(f"new path length = {len(new_path)}")

        return new_path, start_idx, end_idx

    def estimate_path_resolution(self, path):
        distances = []

        for i in range(1, len(path)):
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
            d = math.hypot(dx, dy)

            if d > 1e-6:
                distances.append(d)

        if not distances:
            return self.resolution

        return sum(distances) / len(distances)

    def plan(self, start, goal, obstacles, margin=2.0):
        min_x = min(start[0], goal[0], *[o[0] for o in obstacles]) - margin
        max_x = max(start[0], goal[0], *[o[0] for o in obstacles]) + margin
        min_y = min(start[1], goal[1], *[o[1] for o in obstacles]) - margin
        max_y = max(start[1], goal[1], *[o[1] for o in obstacles]) + margin

        width = int((max_x - min_x) / self.resolution) + 1
        height = int((max_y - min_y) / self.resolution) + 1

        start_grid = self.world_to_grid(start[0], start[1], min_x, min_y)
        goal_grid = self.world_to_grid(goal[0], goal[1], min_x, min_y)

        obstacle_grid = self.create_obstacle_grid(width, height, min_x, min_y, obstacles)

        return [
            self.grid_to_world(gx, gy, min_x, min_y)
            for gx, gy in self.a_star_search(start_grid, goal_grid, obstacle_grid, width, height)
        ]

    def world_to_grid(self, x, y, min_x, min_y):
        return (
            int(round((x - min_x) / self.resolution)),
            int(round((y - min_y) / self.resolution)),
        )

    def grid_to_world(self, gx, gy, min_x, min_y):
        return (
            gx * self.resolution + min_x,
            gy * self.resolution + min_y,
        )

    def create_obstacle_grid(self, width, height, min_x, min_y, obstacles):
        grid = [[0 for _ in range(width)] for _ in range(height)]

        for gy in range(height):
            for gx in range(width):
                x, y = self.grid_to_world(gx, gy, min_x, min_y)

                for ox, oy in obstacles:
                    if math.hypot(x - ox, y - oy) <= self.collision_radius:
                        grid[gy][gx] = 1
                        break

        return grid

    def a_star_search(self, start, goal, obstacle_grid, width, height):
        open_set = []
        heapq.heappush(open_set, (0.0, start))

        came_from = {}
        g_cost = {start: 0.0}
        closed_set = set()

        while open_set:
            _, current = heapq.heappop(open_set)

            if current in closed_set:
                continue

            if current == goal:
                return self.reconstruct_path(came_from, current)

            closed_set.add(current)

            for dx, dy, move_cost in self.motion:
                neighbor = (current[0] + dx, current[1] + dy)
                nx, ny = neighbor

                if nx < 0 or nx >= width or ny < 0 or ny >= height:
                    continue

                if obstacle_grid[ny][nx] == 1:
                    continue

                new_g = g_cost[current] + move_cost * self.resolution

                if neighbor not in g_cost or new_g < g_cost[neighbor]:
                    g_cost[neighbor] = new_g
                    h = self.heuristic(neighbor, goal)
                    f = new_g + h
                    came_from[neighbor] = current
                    heapq.heappush(open_set, (f, neighbor))

        return []

    def heuristic(self, node, goal):
        dx = (node[0] - goal[0]) * self.resolution
        dy = (node[1] - goal[1]) * self.resolution
        return math.hypot(dx, dy)

    def reconstruct_path(self, came_from, current):
        path = [current]

        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()
        return path


if __name__ == "__main__":
    planner = AStarPlanner(resolution=0.05, safety_margin=0.2)

    # example original path: straight line
    original_path = [(x * 0.05 , 0.0) for x in range(81)]

    obstacles = [
        (-2.0, 0.0),
        (2.0, 0.0),
        (-1.0, -2.0),
    ]

    new_path, start_idx, end_idx = planner.replan_local_segment(
        original_path,
        obstacles,
        buffer_distance=1.2,
    )

    print("start_idx:", start_idx)
    print("end_idx:", end_idx)
    print("new_path length:", len(new_path))