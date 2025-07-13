import numpy as np
import pygame
import math
from collections import deque

class OccupancyGrid:
    def __init__(self, screen, resolution, mode="minimap", position=(10, 10), size=(200, 200), robot_width=40, safety_margin=2):
        """
        Args:
            width (int): map width in pixels
            height (int): map height in pixels
            resolution (int): size of each grid cell in pixels
            mode (str): 'minimap' or 'full' or 'none' size display
        """
        self.screen = screen
        self.width, self.height = screen.get_size()
        self.resolution = resolution
        self.mode = mode  # 'minimap' or 'full'

        self.size = size
        self.position = position
        
        self.grid_width = self.width // resolution
        self.grid_height = self.height // resolution

        # 0 = free, 127 = unknown, 255 = occupied
        self.grid = np.full((self.grid_width, self.grid_height), 127, dtype=np.uint8)
        
        # Path planning grid (inflated obstacles for safety)
        self.planning_grid = np.full((self.grid_width, self.grid_height), 0, dtype=np.uint8)
        
        # Goal and path storage
        self.goal = None
        self.path = []
        self.current_path_index = 0
        
        # Exploration tracking
        self.explored_cells = set()
        self.exploration_percentage = 0.0

        self.robot_width = robot_width  # in pixels
        self.safety_margin = safety_margin  # in grid cells

        # Create surface for visualization
        self.surface = pygame.Surface((self.grid_width, self.grid_height))
        self.surface.fill((127, 127, 127))  # unknown = gray
        self.surface.set_alpha(100)
        
        # Path visualization surface
        self.path_surface = pygame.Surface((self.grid_width, self.grid_height))
        self.path_surface.set_colorkey((0, 0, 0))  # Transparent background

    def world_to_grid(self, x, y):
        """Convert world (pixel) coordinates to grid indices."""
        i = int(x // self.resolution)
        j = int(y // self.resolution)
        return i, j

    def grid_to_world(self, i, j):
        """Convert grid indices to world (pixel) coordinates."""
        x = i * self.resolution + self.resolution // 2
        y = j * self.resolution + self.resolution // 2
        return x, y

    def mark_occupied(self, x, y):
        """Mark a (world) point as occupied."""
        i, j = self.world_to_grid(x, y)
        if 0 <= i < self.grid_width and 0 <= j < self.grid_height:
            self.grid[i, j] = 255
            # Inflate obstacles for safer path planning
            self.inflate_obstacle(i, j)
            self.surface.set_at((i, j), (100, 0, 0))  # occupied = red
    
    def inflate_obstacle(self, i, j):
        """Inflate obstacles to account for robot size and safety margin."""
        # Calculate inflation radius dynamically
        inflation_radius = math.ceil((self.robot_width / 2) / self.resolution) + self.safety_margin
        for di in range(-inflation_radius, inflation_radius + 1):
            for dj in range(-inflation_radius, inflation_radius + 1):
                ni, nj = i + di, j + dj
                if 0 <= ni < self.grid_width and 0 <= nj < self.grid_height:
                    self.planning_grid[ni, nj] = 255

    def mark_free(self, x, y):
        """Mark a (world) point as free."""
        i, j = self.world_to_grid(x, y)
        if 0 <= i < self.grid_width and 0 <= j < self.grid_height:
            self.grid[i, j] = 0
            # Don't mark planning grid as free here - let inflation handle it
            self.surface.set_at((i, j), (255, 255, 255))  # free = white
            self.explored_cells.add((i, j))

    def set_goal(self, world_x, world_y):
        """Set a goal point in world coordinates."""
        i, j = self.world_to_grid(world_x, world_y)
        if 0 <= i < self.grid_width and 0 <= j < self.grid_height:
            self.goal = (i, j)
            return True
        return False

    def find_path_to_goal(self, start_pos):
        """Find path from current position to goal using A* algorithm."""
        if self.goal is None:
            return []
        
        start_i, start_j = self.world_to_grid(start_pos[0], start_pos[1])
        
        # A* pathfinding using planning grid (with inflated obstacles)
        path = self.a_star((start_i, start_j), self.goal, use_planning_grid=True)
        if path:
            self.path = path
            self.current_path_index = 0
            return path
        return []

    def a_star(self, start, goal, use_planning_grid=False):
        """A* pathfinding algorithm."""
        grid_to_use = self.planning_grid if use_planning_grid else self.grid
        if grid_to_use[goal] == 255:  # Goal is occupied
            return []
            
        open_set = {start}
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while open_set:
            current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
            
            if current == goal:
                return self.reconstruct_path(came_from, current)
            
            open_set.remove(current)
            
            for neighbor in self.get_neighbors(current):
                if grid_to_use[neighbor] == 255:  # Occupied
                    continue
                    
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    open_set.add(neighbor)
        
        return []  # No path found

    def heuristic(self, a, b):
        """Manhattan distance heuristic."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, pos):
        """Get valid neighbors for pathfinding."""
        i, j = pos
        neighbors = []
        for di, dj in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            ni, nj = i + di, j + dj
            if 0 <= ni < self.grid_width and 0 <= nj < self.grid_height:
                neighbors.append((ni, nj))
        return neighbors

    def reconstruct_path(self, came_from, current):
        """Reconstruct path from A* results."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def get_next_waypoint(self, current_pos):
        """Get the next waypoint in the path."""
        if not self.path or self.current_path_index >= len(self.path):
            return None
        
        waypoint = self.path[self.current_path_index]
        world_x, world_y = self.grid_to_world(waypoint[0], waypoint[1])
        
        # Check if we're close enough to current waypoint
        current_i, current_j = self.world_to_grid(current_pos[0], current_pos[1])
        if self.heuristic((current_i, current_j), waypoint) < 2:  # Within 2 grid cells
            self.current_path_index += 1
        
        return (world_x, world_y)

    def update_exploration_percentage(self):
        """Update the percentage of explored area."""
        total_cells = self.grid_width * self.grid_height
        self.exploration_percentage = len(self.explored_cells) / total_cells * 100

    def update(self, all_data, sensors, robot_position=None):
        """Mark points as occupied based on full LiDAR data."""
        lidar_data = all_data["2D Lidar"]
        for sensor in sensors:
            if sensor.__name__() == '2D Lidar':
                lidar_range = sensor.range
                lidar_res = sensor.resolution

        # Clear planning grid at start of update
        self.planning_grid.fill(0)
        
        # Get robot position for ray tracing
        robot_pos = robot_position if robot_position is not None else (0, 0)
        
        for distance, x, y in lidar_data:
            if (distance < (lidar_range - lidar_res)):  # Hit detected
                self.mark_occupied(x, y)
                # Mark all cells along the ray as free (except the hit point)
                self.mark_ray_free(robot_pos[0], robot_pos[1], x, y, distance)
            else:
                # No hit, mark entire ray as free
                self.mark_ray_free(robot_pos[0], robot_pos[1], x, y, distance)
        
        self.update_exploration_percentage()

    def mark_ray_free(self, start_x, start_y, end_x, end_y, max_distance):
        """Mark all cells along a ray as free using Bresenham's line algorithm."""
        # Convert to grid coordinates
        start_i, start_j = self.world_to_grid(start_x, start_y)
        end_i, end_j = self.world_to_grid(end_x, end_y)
        
        # Use Bresenham's line algorithm to trace the ray
        dx = abs(end_i - start_i)
        dy = abs(end_j - start_j)
        sx = 1 if start_i < end_i else -1
        sy = 1 if start_j < end_j else -1
        err = dx - dy
        
        x, y = start_i, start_j
        distance_traveled = 0
        
        while True:
            # Mark current cell as free (if within bounds)
            if 0 <= x < self.grid_width and 0 <= y < self.grid_height:
                # Only mark as free if it's not already marked as occupied
                if self.grid[x, y] != 255:
                    self.grid[x, y] = 0
                    self.surface.set_at((x, y), (255, 255, 255))  # free = white
                    self.explored_cells.add((x, y))
            
            # Check if we've reached the end point
            if x == end_i and y == end_j:
                break
                
            # Check if we've exceeded the maximum distance
            distance_traveled += 1
            if distance_traveled > max_distance / self.resolution:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
  
    def draw(self):
        """Draw the map onto the main screen."""
        # Draw base map
        if self.mode == "minimap":
            mini_map = pygame.transform.scale(self.surface, self.size)
            self.screen.blit(mini_map, self.position)
            
            # Draw goal if set
            if self.goal:
                goal_x, goal_y = self.grid_to_world(self.goal[0], self.goal[1])
                scaled_x = (goal_x / self.grid_width) * self.size[0] + self.position[0]
                scaled_y = (goal_y / self.grid_height) * self.size[1] + self.position[1]
                pygame.draw.circle(self.screen, (0, 255, 0), (int(scaled_x), int(scaled_y)), 5)
            
            # Draw path
            if self.path:
                for i, waypoint in enumerate(self.path):
                    if i < len(self.path) - 1:  # Don't draw line from last waypoint
                        # Convert grid coordinates directly to minimap coordinates
                        scaled_x1 = (waypoint[0] / self.grid_width) * self.size[0] + self.position[0]
                        scaled_y1 = (waypoint[1] / self.grid_height) * self.size[1] + self.position[1]
                        scaled_x2 = (self.path[i+1][0] / self.grid_width) * self.size[0] + self.position[0]
                        scaled_y2 = (self.path[i+1][1] / self.grid_height) * self.size[1] + self.position[1]
                        
                        pygame.draw.line(self.screen, (0, 0, 255), 
                                       (int(scaled_x1), int(scaled_y1)), 
                                       (int(scaled_x2), int(scaled_y2)), 2)
            
            # Draw exploration percentage
            font = pygame.font.Font(None, 24)
            text = font.render(f"Explored: {self.exploration_percentage:.1f}%", True, (0, 0, 0))
            self.screen.blit(text, (self.position[0], self.position[1] + self.size[1] + 5))
            
        elif self.mode == "full":
            full_map = pygame.transform.scale(self.surface, (self.width, self.height))
            self.screen.blit(full_map, (0, 0))
        elif self.mode == 'none':
            pass # No map drawn
        else:
            raise ValueError(f"Invalid mode. Choose 'minimap' or 'full' or 'none' not {self.mode}.")

    def reset(self):
        """Clear the map to unknown."""
        self.grid.fill(127)
        self.planning_grid.fill(0)
        self.surface.fill((127, 127, 127))
        self.path = []
        self.goal = None
        self.explored_cells.clear()
        self.exploration_percentage = 0.0

    def __name__(self):
        return 'OccupancyGrid'