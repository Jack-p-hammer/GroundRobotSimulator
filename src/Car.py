import pygame
import numpy as np
import math
import redis

from src.Perception import OccupancyGrid 
from src.Sensors import Lidar
from src.Sensors import Distance
from src.Sensors import IMU
from src.ControlMethods import NavigationController, ObstacleAvoidanceController, ExplorationController


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
        
        # Navigation and control
        self.navigation_controller = NavigationController(self)
        self.obstacle_avoidance = ObstacleAvoidanceController(self)
        self.exploration_controller = ExplorationController(self)
        
        # Control mode: 'manual', 'navigation', 'exploration', 'autonomous'
        self.control_mode = 'manual'
        self.active_controller = None

        self.setup_channels(channels)
        if server and len(server) > 0:
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
                self.occupancy_grid = OccupancyGrid(
                    self.screen,
                    sensor_dict['resolution'],
                    sensor_dict['mode'],
                    sensor_dict['position'],
                    sensor_dict['size'],
                    robot_width=self.CAR_SIZE[0],
                    safety_margin=2
                )
                self.perceptors.append(self.occupancy_grid)

            else:
                print(f"Extra type '{sensor_dict['type']}' does not exist in the code base (yet), add it! ")

    def set_control_mode(self, mode):
        """Set the control mode for the car."""
        self.control_mode = mode
        
        # Deactivate all controllers
        self.navigation_controller.deactivate()
        self.obstacle_avoidance.deactivate()
        self.exploration_controller.deactivate()
        
        # Activate appropriate controller
        if mode == 'navigation':
            self.active_controller = self.navigation_controller
            self.navigation_controller.activate()
        elif mode == 'exploration':
            self.active_controller = self.exploration_controller
            self.exploration_controller.activate()
        elif mode == 'autonomous':
            # Autonomous mode uses both navigation and obstacle avoidance
            self.navigation_controller.activate()
            self.obstacle_avoidance.activate()
            self.active_controller = self.navigation_controller
        else:  # manual
            self.active_controller = None

    def set_goal(self, goal_x, goal_y):
        """Set a navigation goal."""
        if hasattr(self, 'occupancy_grid'):
            # Set goal in occupancy grid
            self.occupancy_grid.set_goal(goal_x, goal_y)
            
            # Find path to goal
            path = self.occupancy_grid.find_path_to_goal(self.position)
            
            if path:
                # Convert grid path to world coordinates
                world_path = []
                for grid_point in path:
                    world_x, world_y = self.occupancy_grid.grid_to_world(grid_point[0], grid_point[1])
                    world_path.append([world_x, world_y])
                
                # Set path in navigation controller
                self.navigation_controller.set_path(world_path)
                self.navigation_controller.set_goal(goal_x, goal_y)
                
                # Switch to navigation mode
                self.set_control_mode('navigation')
                return True
            else:
                print("No path found to goal!")
                return False
        else:
            # No occupancy grid, use direct navigation
            self.navigation_controller.set_goal(goal_x, goal_y)
            self.set_control_mode('navigation')
            return True

    def handle_mouse_click(self, mouse_pos):
        """Handle mouse clicks for goal setting on minimap."""
        if hasattr(self, 'occupancy_grid') and self.occupancy_grid.mode == "minimap":
            # Check if click is within minimap bounds
            map_x, map_y = self.occupancy_grid.position
            map_width, map_height = self.occupancy_grid.size
            
            if (map_x <= mouse_pos[0] <= map_x + map_width and 
                map_y <= mouse_pos[1] <= map_y + map_height):
                
                # Convert minimap coordinates to world coordinates
                relative_x = (mouse_pos[0] - map_x) / map_width
                relative_y = (mouse_pos[1] - map_y) / map_height
                
                world_x = relative_x * self.width
                world_y = relative_y * self.height
                
                # Set goal
                success = self.set_goal(world_x, world_y)
                if success:
                    print(f"Goal set at ({world_x:.1f}, {world_y:.1f})")
                return True
        return False

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

        # Handle control based on mode
        if self.control_mode == 'manual':
            # Manual control with keyboard
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
        else:
            # Autonomous control
            if self.active_controller:
                self.active_controller.update(keys, walls, collision_map)
            
            # Apply obstacle avoidance in autonomous modes
            if self.control_mode == 'autonomous':
                self.obstacle_avoidance.update(keys, walls, collision_map)

        # Check for collisions with walls and objects (all in walls list)
        if self.check_collision(walls):
            self.reset_position()
            return True  # Collision detected
        
        self.get_data(col_map=collision_map)
        self.update_perceptors()
        self.draw(self.screen, collision_map)

    def get_data(self, col_map):
        for sensor in self.sensors:
            sensor_type = sensor.__name__() 
            if sensor_type in self.data.keys():
                self.data[sensor_type] = sensor.get_readings(col_map, self.BODY_DATA)

    def update_perceptors(self):
        # Find lidar object
        for perceptor in self.perceptors:
            if hasattr(perceptor, 'update'):
                if isinstance(perceptor, OccupancyGrid):
                    perceptor.update(self.data, self.sensors, self.position)
                else:
                    perceptor.update(self.data, self.sensors)

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
        
        # Color based on control mode
        if self.control_mode == 'manual':
            color = (255, 0, 0)  # Red
        elif self.control_mode == 'navigation':
            color = (0, 255, 0)  # Green
        elif self.control_mode == 'exploration':
            color = (0, 0, 255)  # Blue
        else:  # autonomous
            color = (255, 165, 0)  # Orange
            
        pygame.draw.polygon(screen, color, points)
        
        # Draw direction arrow
        arrow_length = 30
        arrow_x = self.position[0] + arrow_length * math.cos(math.radians(self.angle))
        arrow_y = self.position[1] + arrow_length * math.sin(math.radians(self.angle))
        pygame.draw.line(screen, (0, 0, 0), (self.position[0], self.position[1]), (arrow_x, arrow_y), 3)
        pygame.draw.circle(screen, (0, 0, 0), (arrow_x, arrow_y), 5)

        # Draw goal and path if in navigation mode
        if self.control_mode == 'navigation' and self.navigation_controller.goal_position is not None:
            goal_pos = self.navigation_controller.goal_position
            pygame.draw.circle(screen, (0, 255, 0), (int(goal_pos[0]), int(goal_pos[1])), 10)
            pygame.draw.circle(screen, (255, 255, 255), (int(goal_pos[0]), int(goal_pos[1])), 8)
            
            # Draw path on main screen
            if hasattr(self, 'occupancy_grid') and self.occupancy_grid.path and len(self.occupancy_grid.path) > 1:
                for i, waypoint in enumerate(self.occupancy_grid.path):
                    if i < len(self.occupancy_grid.path) - 1:  # Don't draw line from last waypoint
                        wx1, wy1 = self.occupancy_grid.grid_to_world(waypoint[0], waypoint[1])
                        wx2, wy2 = self.occupancy_grid.grid_to_world(self.occupancy_grid.path[i+1][0], self.occupancy_grid.path[i+1][1])
                        
                        pygame.draw.line(screen, (0, 0, 255), 
                                       (int(wx1), int(wy1)), 
                                       (int(wx2), int(wy2)), 3)
                        
                        # Draw waypoint markers
                        pygame.draw.circle(screen, (255, 0, 255), (int(wx1), int(wy1)), 5)

        for subject in self.subjects:
            subject.draw()
        
# Run this only if you are running the file directly
if __name__ == '__main__':
    print("\nYou are running the car class directly.\n")
