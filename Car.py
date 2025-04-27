import pygame
import numpy as np
import math
from Occupancygrid import OccupancyGrid 



# Constants
CAR_SIZE = (40, 20)
LIDAR_RANGE = 300
NUM_LIDAR_POINTS = 120


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

        self.GRID_RESOLUTION = 10  # pixels per grid cell
        self.occupancy_grid = OccupancyGrid(self.screen, self.GRID_RESOLUTION, mode = 'minimap')  # Initialize occupancy grid


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
        
        data = self.get_lidar_measurements(collision_map)
        lidar_data = [d[0] for d in data]
        self.apply_control(lidar_data)

        # Update the occupancy grid with the collision map
        self.lidar_res = 10  # resolution of the lidar in pixels
        for distance, (x, y) in data:
            if (distance < (LIDAR_RANGE - self.lidar_res)):  # Only mark hits
                self.occupancy_grid.mark_occupied(x, y)
        

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

        lidar = self.get_lidar_measurements(collision_map)
        for distance, point in lidar:
            normalized = distance / LIDAR_RANGE
            r = int(255 * normalized)
            g = 0
            b = int(255 * (1 - normalized))
            pygame.draw.circle(screen, (r, g, b), point, 5)

        self.occupancy_grid.draw(screen, position=(0, 0), size=(100, 100))
            

    def get_lidar_measurements(self, collision_map):
        """
        Uses a precomputed collision map (2D boolean array) to simulate
        fast LiDAR raycasting. Returns both distance and hit point.
        """
        measurements = []
        height, width = collision_map.shape[1], collision_map.shape[0]  # x = width, y = height

        # Precompute angle values
        angles = [
            math.radians(self.angle + i * (360 / NUM_LIDAR_POINTS))
            for i in range(NUM_LIDAR_POINTS)
        ]
        cos_vals = [math.cos(a) for a in angles]
        sin_vals = [math.sin(a) for a in angles]

        for i in range(NUM_LIDAR_POINTS):
            cos_a = cos_vals[i]
            sin_a = sin_vals[i]

            for dist in range(0, LIDAR_RANGE, 2):  # Step by 2 for speed
                x = int(self.position[0] + dist * cos_a)
                y = int(self.position[1] + dist * sin_a)

                if x < 0 or x >= width or y < 0 or y >= height:
                    measurements.append((dist, (x, y)))
                    break

                if collision_map[x, y]:
                    measurements.append((dist, (x, y)))
                    break
            else:
                x = int(self.position[0] + LIDAR_RANGE * cos_a)
                y = int(self.position[1] + LIDAR_RANGE * sin_a)
                measurements.append((LIDAR_RANGE, (x, y)))
        # add noise
        noise = np.random.normal(0, 3, len(measurements))
        for i in range(len(measurements)):
            dist, point = measurements[i]
            dist += noise[i]
            dist = max(0, min(LIDAR_RANGE, dist))
            x = int(self.position[0] + dist * cos_vals[i])
            y = int(self.position[1] + dist * sin_vals[i])
            measurements[i] = (dist, (x, y))
        return measurements




# Run this only if you are running the file directly
if __name__ == '__main__':
    print("\nYou are running the car class directly.\n")
