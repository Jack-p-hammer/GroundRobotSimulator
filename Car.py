import pygame
import numpy as np
import math
from Perception import OccupancyGrid 
from Sensors import Lidar
from Sensors import Distance

# Constants
CAR_SIZE = (40, 20)
LIDAR_RANGE = 300
NUM_LIDAR_POINTS = 120
LIDAR_RESOLUTION = 13  # in pixels

GRID_RESOLUTION = 10  # pixels per grid cell

# Car Class
class Car:
    def __init__(self, origin, angle, speed, screen):
        # Display parameters
        self.screen = screen
        self.width, self.height = screen.get_size()

        self.origin = np.array(origin, dtype=float)  # Store the original position, angle, speed
        self.starting_angle = angle
        self.starting_speed = speed

        self.position = np.array(origin, dtype=float)
        self.speed = speed

        self.velocity = np.array([speed * math.cos(math.radians(angle)), speed * math.sin(math.radians(angle))], dtype=float)
        self.angle = angle
        self.angular_velocity = 0.0


        # Create Subjects (What car uses to perceive the world)
        self.occupancy_grid = OccupancyGrid(self.screen, GRID_RESOLUTION, 'minimap', (5, 5), (200, 200))  # Initialize occupancy grid
        self.lidar = Lidar(NUM_LIDAR_POINTS, self.screen, LIDAR_RANGE, resolution=LIDAR_RESOLUTION) # Initialize LiDAR sensor
        self.dist = Distance(self.screen, 240, resolution=10)  # Initialize distance sensor
        self.subjects = [self.lidar, self.occupancy_grid, self.dist]  # List of subjects for the car to perceive the world

    def update(self, keys, walls, collision_map):
        # Uses speed and angle to calculate velocity
        self.velocity = np.array([self.speed * math.cos(math.radians(self.angle)), self.speed * math.sin(math.radians(self.angle))])
        # Applies an angular velocity damping effect
        self.angular_velocity *= 0.5
        # Updates the position and angle of the car using simple sums
        self.position += self.velocity
        self.angle += self.angular_velocity
        # Wrap the angle to be within 0-360 degrees
        if self.angle >= 360:
            self.angle -= 360
        elif self.angle < 0:
            self.angle += 360

        # Check for key presses to control the car
        if keys[pygame.K_UP]:
            self.speed += 0.1
        if keys[pygame.K_DOWN]:
            self.speed -= 0.1
        if keys[pygame.K_LEFT]:
            self.angular_velocity -= 1.0
        if keys[pygame.K_RIGHT]:
            self.angular_velocity += 1.0
        if keys[pygame.K_SPACE]:
            self.speed = 0.0
            self.angular_velocity = 0.0

        # Check for collisions with walls and objects (all in walls list)
        if self.check_collision(walls):
            self.reset_position()
            return True  # Collision detected
        

        data = self.lidar.get_readings(collision_map, self.position, self.angle)
        front_distance = self.dist.get_readings(collision_map, self.position, self.angle)
        distance_data = [d[0] for d in data]
        self.apply_control(distance_data)

        # Update the occupancy grid with the collision map
        self.occupancy_grid.mark_occupied_routine(data, LIDAR_RESOLUTION, LIDAR_RANGE)
        
        

    def apply_control(self, lidar_data):
        """
        Basic collision avoidance using LiDAR data.
        If an obstacle is too close in front, rotate to avoid it.
        """
        THRESHOLD_DISTANCE = 60  # pixels
        ROTATION_SPEED = 15       # degrees/frame

        # Get front 30° window (±15° around forward)
        N = len(lidar_data)
        cone_width = int(30 / 360 * N)  # number of LiDAR rays in 30°
        front_indices = list(range(-cone_width // 2, cone_width // 2))

        # Sample distances in front
        front_distances = [lidar_data[i % N] for i in front_indices]

        # Decision: too close?
        if min(front_distances) < THRESHOLD_DISTANCE:
            # Obstacle ahead → rotate
            self.angular_velocity = ROTATION_SPEED
        else:
            # Clear → go straight
            self.angular_velocity = 0

    def check_collision(self, walls):
        car_rect = pygame.Rect(self.position[0] - CAR_SIZE[0] / 2, self.position[1] - CAR_SIZE[1] / 2, CAR_SIZE[0], CAR_SIZE[1])
        for wall in walls:
            if car_rect.colliderect(wall):
                return True
        return False

    def reset_position(self):
        self.position = np.array(self.origin, dtype=float)
        self.velocity = np.array([0.0, 0.0], dtype=float)
        self.speed = self.starting_speed
        self.angle = self.starting_angle

    def draw(self, screen, collision_map):
        """
        Draws the car on the screen and simulates LiDAR measurements.
        """
        points = [
            (self.position[0] + CAR_SIZE[0] / 2 * math.cos(math.radians(self.angle)) - CAR_SIZE[1] / 2 * math.sin(math.radians(self.angle)),
             self.position[1] + CAR_SIZE[0] / 2 * math.sin(math.radians(self.angle)) + CAR_SIZE[1] / 2 * math.cos(math.radians(self.angle))),
            (self.position[0] - CAR_SIZE[0] / 2 * math.cos(math.radians(self.angle)) - CAR_SIZE[1] / 2 * math.sin(math.radians(self.angle)),
             self.position[1] - CAR_SIZE[0] / 2 * math.sin(math.radians(self.angle)) + CAR_SIZE[1] / 2 * math.cos(math.radians(self.angle))),
            (self.position[0] - CAR_SIZE[0] / 2 * math.cos(math.radians(self.angle)) + CAR_SIZE[1] / 2 * math.sin(math.radians(self.angle)),
             self.position[1] - CAR_SIZE[0] / 2 * math.sin(math.radians(self.angle)) - CAR_SIZE[1] / 2 * math.cos(math.radians(self.angle))),
            (self.position[0] + CAR_SIZE[0] / 2 * math.cos(math.radians(self.angle)) + CAR_SIZE[1] / 2 * math.sin(math.radians(self.angle)),
             self.position[1] + CAR_SIZE[0] / 2 * math.sin(math.radians(self.angle)) - CAR_SIZE[1] / 2 * math.cos(math.radians(self.angle)))
        ]
        pygame.draw.polygon(screen, (255, 0, 0), points)
        arrow_length = 30
        arrow_x = self.position[0] + arrow_length * math.cos(math.radians(self.angle))
        arrow_y = self.position[1] + arrow_length * math.sin(math.radians(self.angle))
        pygame.draw.line(screen, (0, 0, 0), (self.position[0], self.position[1]), (arrow_x, arrow_y), 3)
        pygame.draw.circle(screen, (0, 0, 0), (arrow_x, arrow_y), 5)


        for subject in self.subjects:
            subject.draw()
        
# Run this only if you are running the file directly
if __name__ == '__main__':
    print("\nYou are running the car class directly.\n")
