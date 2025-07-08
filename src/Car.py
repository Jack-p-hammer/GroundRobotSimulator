import pygame
import numpy as np
import math
import redis

from Perception import OccupancyGrid 
from Sensors import Lidar
from Sensors import Distance
from Sensors import IMU


# Car Class
class Car:
    def __init__(self, origin, angle, speed, screen, car_size=(40, 20), add_ons=None, channels = None, server = None):
        # Display parameters
        self.screen = screen
        self.width, self.height = screen.get_size()
        self.CAR_SIZE = car_size

        self.origin = np.array(origin, dtype=float)  # Store the original position, angle, speed
        self.starting_angle = angle
        self.starting_speed = speed

        self.position = np.array(origin, dtype=float)
        self.speed = speed

        self.velocity = np.array([speed * math.cos(math.radians(angle)), speed * math.sin(math.radians(angle))], dtype=float)
        self.angle = angle
        self.angular_velocity = 0.0


        self.BODY_DATA = np.array([self.position,
                                  self.velocity,
                                  [self.angle, 0],
                                  [self.angular_velocity, 0]])
        
        


        self.setup_channels(channels)
        self.setup_server(server[0])

        self.sensors = list()
        self.perceptors = list()
        self.data = dict()
        self.setup_subjects(add_ons)  # Setup additional subjects based on extras parameter
        self.subjects = self.sensors + self.perceptors

    def setup_channels(self, channels):
        for diction in channels:
            if diction['type'] == "Publisher Channels":
                print(f"Publishing to: {diction['channels']}")
            if diction['type'] == "Subscriber Channels":
                print(f"Subscribing to: {diction['channels']}")

    def setup_server(self, server_info):
        host = server_info["host"]
        port = server_info["port"]
        channels = server_info["channels"]
        print(server_info)

        r = redis.Redis()

        pass


    def setup_subjects(self, extras):
        """
        Setup additional subjects for the car based on the extras parameter.
        This can be used to add more sensors or perception capabilities.
        """
        for sensor_dict in extras:
            if sensor_dict['type'] == '2D Lidar':
                self.lidar = Lidar(sensor_dict['num_points'], self.screen, sensor_dict['range'], sensor_dict['resolution'])
                self.sensors.append(self.lidar)
                self.data[sensor_dict['type']] = 0

            elif sensor_dict['type'] == 'DistanceSensor':
                self.dist = Distance(self.screen, sensor_dict['range'], sensor_dict['resolution'])
                self.sensors.append(self.dist)
                self.data[sensor_dict['type']] = 0

            elif sensor_dict['type'] == 'IMU':
                self.IMU = IMU(self.screen, sensor_dict['resolution'])
                self.sensors.append(self.IMU)
                self.data[sensor_dict['type']] = 0


            elif sensor_dict['type'] == 'OccupancyGrid':
                self.occupancy_grid = OccupancyGrid(self.screen, sensor_dict['resolution'], sensor_dict['mode'], sensor_dict['position'], sensor_dict['size'])
                self.perceptors.append(self.occupancy_grid)

            else:
                print(f"Extra type '{sensor_dict['type']}' does not exist in the code base (yet), add it! ")

    

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

        self.BODY_DATA = np.array([self.position,
                            self.velocity,
                            [self.angle, 0],
                            [self.angular_velocity, 0]])

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
        
        self.get_data(col_map=collision_map)
        self.update_perceptors()
        self.apply_control()        
        self.draw(self.screen, collision_map)




    def get_data(self, col_map):
        for sensor in self.sensors:
            sensor_type = sensor.__name__() 
            if sensor_type in self.data.keys():
                self.data[sensor_type] = sensor.get_readings(col_map, self.BODY_DATA)

    def update_perceptors(self):
        # Find lidar object
        for perceptor in self.perceptors:
            perceptor.update(self.data, self.sensors)
            

    def apply_control(self):
        """
        Basic collision avoidance using LiDAR data.
        If an obstacle is too close in front, rotate to avoid it.
        """
        lidar_data = self.data['2D Lidar'][:,0]
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
        car_rect = pygame.Rect(self.position[0] - self.CAR_SIZE[0] / 2, self.position[1] - self.CAR_SIZE[1] / 2, self.CAR_SIZE[0], self.CAR_SIZE[1])
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
        CAR_SIZE = self.CAR_SIZE
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
