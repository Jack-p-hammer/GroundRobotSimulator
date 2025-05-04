import numpy as np
import pygame
import math


# First, build a general sensor class that can be used for different types of sensors.
# This class will be the base class for all sensors in the simulation.
# It will contain the basic properties and methods that all sensors will share.
# Some sensors may include: Lidar, Camera, GPS, IMU, etc.

class sensor(object):
    def __init__(self, num_points, screen = None):
        self.num_points = num_points
        self.screen = screen


    def get_readings(self):
        raise NotImplementedError("This method should be overridden by subclasses")


class Lidar(sensor):
    def __init__(self, num_points, screen, range, resolution=1):
        super().__init__(num_points, screen)
        self.range = range
        self.data = np.zeros((num_points,3), dtype=float)
        self.resolution = resolution  # in pixels

    def get_readings(self, collision_map, position, angle):
        """
        Uses a precomputed collision map (2D boolean array) to simulate
        fast LiDAR raycasting. Returns both distance and hit point.
        """
        NUM_LIDAR_POINTS = self.num_points
        LIDAR_RANGE = self.range
        height, width = collision_map.shape[1], collision_map.shape[0]  # x = width, y = height

        # Precompute angle values
        angles = [
            math.radians(angle + i * (360 / NUM_LIDAR_POINTS))
            for i in range(NUM_LIDAR_POINTS)
        ]
        cos_vals = [math.cos(a) for a in angles]
        sin_vals = [math.sin(a) for a in angles]

        for i in range(NUM_LIDAR_POINTS):
            cos_a = cos_vals[i]
            sin_a = sin_vals[i]

            for dist in range(0, LIDAR_RANGE, 2):  # Step by 2 for speed
                x = int(position[0] + dist * cos_a)
                y = int(position[1] + dist * sin_a)

                if x < 0 or x >= width or y < 0 or y >= height:
                    self.data[i, :] = dist, x, y
                    break

                if collision_map[x, y]:
                    self.data[i, :] = dist, x, y
                    break
            else:
                x = int(position[0] + LIDAR_RANGE * cos_a)
                y = int(position[1] + LIDAR_RANGE * sin_a)
                self.data[i, :] = LIDAR_RANGE, x, y
        # adding noise
        noise = np.random.normal(0, self.resolution, (self.num_points,2))
        self.data[:, 1:] += noise[:, :] * 0.1

        return self.data
    
    def draw(self):
        for distance, x,y  in self.data:
            point = (int(x), int(y))
            normalized = distance / self.range
            r = int(255 * normalized)
            g = 0
            b = int(255 * (1 - normalized))
            pygame.draw.circle(self.screen, (r, g, b), point, 5)