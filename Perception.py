import numpy as np
import pygame
import math


class OccupancyGrid:
    def __init__(self, screen, resolution, mode="minimap", position=(10, 10), size=(200, 200)):
        """
        Args:
            width (int): map width in pixels
            height (int): map height in pixels
            resolution (int): size of each grid cell in pixels
            mode (str): 'minimap' or 'full' size display
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

        # Create surface for visualization
        self.surface = pygame.Surface((self.grid_width, self.grid_height))
        self.surface.fill((127, 127, 227))  # unknown = gray
        self.surface.set_alpha(100)

    def world_to_grid(self, x, y):
        """Convert world (pixel) coordinates to grid indices."""
        i = int(x // self.resolution)
        j = int(y // self.resolution)
        return i, j

    def mark_occupied(self, x, y):
        """Mark a (world) point as occupied."""
        i, j = self.world_to_grid(x, y)
        if 0 <= i < self.grid_width and 0 <= j < self.grid_height:
            self.grid[i, j] = 255
            self.surface.set_at((i, j), (100, 0, 0))  # occupied = black
    
    def mark_occupied_routine(self, lidar_data, lidar_res, lidar_range):
        """Mark points as occupied based on full LiDAR data."""
        for distance, x, y in lidar_data:
            if (distance < (lidar_range - lidar_res)):  # Only mark hits
                self.mark_occupied(x, y)
  

    def mark_free(self, x, y):
        """Mark a (world) point as free."""
        i, j = self.world_to_grid(x, y)
        if 0 <= i < self.grid_width and 0 <= j < self.grid_height:
            self.grid[i, j] = 0
            self.surface.set_at((i, j), (255, 255, 255))  # free = white

    def draw(self):
        """Draw the map onto the main screen.

        - If mode == 'minimap', it draws a small version in the corner.
        - If mode == 'full', it scales to the entire window size.
        """
        if self.mode == "minimap":
            mini_map = pygame.transform.scale(self.surface, self.size)
            self.screen.blit(mini_map, self.position)
        elif self.mode == "full":
            full_map = pygame.transform.scale(self.surface, (self.width, self.height))
            self.screen.blit(full_map, (0, 0))
        else:
            raise ValueError("Invalid mode. Choose 'minimap' or 'full'.")

    def reset(self):
        """Clear the map to unknown."""
        self.grid.fill(127)
        self.surface.fill((127, 127, 127))