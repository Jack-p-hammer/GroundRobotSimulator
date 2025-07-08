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
    
    def __name__(self):
        raise NotImplementedError("This method should be overridden by subclasses")

class Lidar(sensor):
    def __init__(self, num_points, screen, range, resolution=1):
        super().__init__(num_points, screen)
        self.range = range
        self.data = np.zeros((num_points,3), dtype=float)
        self.resolution = resolution  # in pixels

    def get_readings(self, collision_map, body_data):
        """
        Uses a precomputed collision map (2D boolean array) to simulate
        fast LiDAR raycasting. 
        
        Returns numpy array with num_points rows with distance in first column 
        and hit points (x and y) in 2nd and 3rd columns.
        """
        position = body_data[0,:]
        angle = body_data[2,0]
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
    
    def __name__(self):
        return "2D Lidar"


class Distance(sensor):
    """
    A simple distance sensor that returns the distance infront of the vehicle/object.
    It uses a precomputed collision map (2D boolean array) to simulate fast raycasting.
    """
    def __init__(self,  screen, range, resolution=1):
        num_points = 1
        super().__init__(num_points, screen)
        self.range = range
        self.data = np.zeros((1,3), dtype=float)
        self.resolution = resolution  # in pixels

    def get_readings(self, collision_map, body_data):
        """"
        uses a precomputed collision map (2D boolean array) to simulate a single distance sensor

        returns a single value 
        """
        position = body_data[0,:]
        angle = body_data[2,0]
        LIDAR_RANGE = self.range
        height, width = collision_map.shape[1], collision_map.shape[0]  # x = width, y = height

        # Precompute angle values
        angles = math.radians(angle)
    
        cos_a = math.cos(angles)
        sin_a = math.sin(angles)
        for dist in range(0, LIDAR_RANGE, 2):  # Step by 2 for speed
            x = int(position[0] + dist * cos_a)
            y = int(position[1] + dist * sin_a)
            if x < 0 or x >= width or y < 0 or y >= height:
                self.data = dist, x, y
                break
            if collision_map[x, y]:
                self.data = dist, x, y
                break
        else:
            x = int(position[0] + LIDAR_RANGE * cos_a)
            y = int(position[1] + LIDAR_RANGE * sin_a)
            self.data = LIDAR_RANGE, x, y
        
        return self.data

    def draw(self):
        point = self.data[1:]
        r = 255
        g = 255
        b = 255
        pygame.draw.circle(self.screen, (r, g, b), point, 10)

    def __name__(self):
        return "DistanceSensor"

class IMU(sensor):
    """
    A simple IMU sensor that returns the angle of the vehicle/object.
    """
    def __init__(self, screen, resolution):
        num_points = 1
        super().__init__(num_points, screen)
        self.data = np.zeros((1,3), dtype=float)
        self.resolution = resolution
        self.prev_lin_vel = 0
        self.prev_rot_vel = 0
        

    def get_readings(self, collision_map, body_data):
        """
        returns a noised linear velocity and acceleration
        and rotational velocity and acceleration
        """
   
        position = body_data[0,:]
        velocity = body_data[1,:]
        angle_pos = body_data[2,0]
        angle_vel = body_data[3,0]        
        lin_velocity_noise = np.random.normal(0, self.resolution, body_data[1,0].shape)
        rot_velocity_noise = np.random.normal(0, self.resolution, body_data[3,0].shape)

        lin_accel = (velocity - self.prev_lin_vel) / (1/60)
        rot_accel = (angle_vel - self.prev_rot_vel) / (1/60)
        return lin_accel, rot_accel
    
    def __name__(self):
        return "IMU"
    
    def draw(self):
        pass
    

    
class GPS(sensor):
    """
    A simple GPS sensor that returns the position of the vehicle/object.
    """
    def __init__(self, screen, range, resolution=1):
        num_points = 1
        super().__init__(num_points, screen)
        self.range = range
        self.data = np.zeros((1,2), dtype=float)
        self.resolution = resolution 

    def get_readings(self):
        raise NotImplementedError("This method should be overridden by subclasses")