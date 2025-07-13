import numpy as np
import math

# Create general control method class that acts as the parents for specific methods:
 
class Controller:
    def __init__(self, robot):
        self.robot = robot
        self.active = False
        self.goal_reached = False
        self.path_following = False

    def activate(self):
        """Activate the controller."""
        self.active = True
        self.goal_reached = False

    def deactivate(self):
        """Deactivate the controller."""
        self.active = False
        self.path_following = False

    def update(self, keys, walls, collision_map):
        """Update the controller - to be overridden by subclasses."""
        pass

# Implement pure pursuit
class PurePursuit(Controller):
    def __init__(self, robot, lookahead_distance=50):
        super().__init__(robot)
        self.lookahead_distance = lookahead_distance
        self.current_waypoint = None
        self.waypoint_index = 0

    def set_path(self, path):
        """Set a path to follow."""
        self.path = path
        self.waypoint_index = 0
        self.path_following = True
        if path:
            self.current_waypoint = path[0]

    def update(self, keys, walls, collision_map):
        """Update pure pursuit controller."""
        if not self.active or not self.path_following or not self.path:
            return

        # Get current position and orientation
        pos = self.robot.position
        angle = self.robot.angle

        # Find closest point on path
        closest_dist = float('inf')
        closest_index = 0
        
        for i, waypoint in enumerate(self.path):
            dist = np.linalg.norm(np.array(waypoint) - pos)
            if dist < closest_dist:
                closest_dist = dist
                closest_index = i

        # Find lookahead point
        lookahead_point = None
        for i in range(closest_index, len(self.path)):
            waypoint = np.array(self.path[i])
            dist = np.linalg.norm(waypoint - pos)
            if dist >= self.lookahead_distance:
                lookahead_point = waypoint
                break

        if lookahead_point is None:
            # Use last waypoint if no lookahead point found
            lookahead_point = np.array(self.path[-1])

        # Calculate desired angle to lookahead point
        desired_angle = math.degrees(math.atan2(
            lookahead_point[1] - pos[1],
            lookahead_point[0] - pos[0]
        ))

        # Normalize angle difference
        angle_diff = desired_angle - angle
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360

        # Apply control
        self.robot.angular_velocity = angle_diff * 0.1  # Proportional control
        self.robot.speed = 2.0  # Constant forward speed

        # Check if goal reached
        if closest_index >= len(self.path) - 1 and closest_dist < 20:
            self.goal_reached = True
            self.path_following = False

class NavigationController(Controller):
    def __init__(self, robot):
        super().__init__(robot)
        self.goal_position = None
        self.path = []
        self.current_waypoint_index = 0
        self.arrival_threshold = 15  # pixels
        self.waypoint_threshold = 25  # pixels
        self.max_speed = 3.0
        self.min_speed = 0.5
        self.angular_gain = 0.15
        self.speed_gain = 0.1

    def set_goal(self, goal_x, goal_y):
        """Set a goal position."""
        self.goal_position = np.array([goal_x, goal_y])
        self.goal_reached = False
        self.current_waypoint_index = 0

    def set_path(self, path):
        """Set a path to follow."""
        self.path = path
        self.current_waypoint_index = 0
        self.path_following = True

    def update(self, keys, walls, collision_map):
        """Update navigation controller."""
        if not self.active:
            return

        if self.goal_position is None:
            return

        current_pos = self.robot.position
        current_angle = self.robot.angle

        # Check for obstacles first
        lidar_data = self.robot.data.get('2D Lidar', np.zeros((1, 3)))
        obstacle_detected = self.check_navigation_obstacles(lidar_data, current_angle)
        
        if obstacle_detected:
            # Obstacle detected, slow down and turn more aggressively
            self.robot.speed = max(0.5, self.robot.speed * 0.7)  # Slow down but don't stop
            self.robot.angular_velocity = 1.0  # Turn more aggressively to find clear path
            return

        # If we have a path, follow it
        if self.path and self.current_waypoint_index < len(self.path):
            target = np.array(self.path[self.current_waypoint_index])
            
            # Check if we've reached current waypoint
            dist_to_waypoint = np.linalg.norm(target - current_pos)
            if dist_to_waypoint < self.waypoint_threshold:
                self.current_waypoint_index += 1
                if self.current_waypoint_index >= len(self.path):
                    # Reached end of path
                    self.goal_reached = True
                    self.robot.speed = 0
                    self.robot.angular_velocity = 0
                    return
                else:
                    target = np.array(self.path[self.current_waypoint_index])
        else:
            # No path, go directly to goal
            target = self.goal_position

        # Calculate desired angle to target
        desired_angle = math.degrees(math.atan2(
            target[1] - current_pos[1],
            target[0] - current_pos[0]
        ))

        # Normalize angle difference
        angle_diff = desired_angle - current_angle
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360

        # Calculate distance to target
        dist_to_target = np.linalg.norm(target - current_pos)

        # Check if goal reached
        if dist_to_target < self.arrival_threshold:
            self.goal_reached = True
            self.robot.speed = 0
            self.robot.angular_velocity = 0
            return

        # Apply proportional control
        self.robot.angular_velocity = angle_diff * self.angular_gain
        
        # Speed control based on distance and angle
        speed_factor = min(1.0, float(dist_to_target) / 100.0)  # Slower when closer
        angle_factor = max(0.3, 1.0 - abs(angle_diff) / 90.0)  # Slower when turning
        target_speed = self.max_speed * speed_factor * angle_factor
        target_speed = max(self.min_speed, target_speed)
        
        # Smooth speed changes
        speed_diff = target_speed - self.robot.speed
        self.robot.speed += speed_diff * self.speed_gain
        
        # Debug output for path following (less frequent)
        if self.path and self.current_waypoint_index < len(self.path) and self.current_waypoint_index % 5 == 0:
            print(f"Navigation: Waypoint {self.current_waypoint_index + 1}/{len(self.path)}, speed: {self.robot.speed:.1f}")

    def check_navigation_obstacles(self, lidar_data, current_angle):
        """Check for obstacles in navigation path."""
        if len(lidar_data) == 0:
            return False
            
        # Check front 45 degrees for obstacles (narrower cone)
        front_angle_range = 22.5  # degrees on each side
        safety_distance = 60  # pixels (closer detection)
        
        # Count obstacles in the front cone
        obstacle_count = 0
        total_rays = 0
        
        for i, (distance, x, y) in enumerate(lidar_data):
            # Calculate the angle of this LiDAR ray relative to robot's current angle
            ray_angle = (i / len(lidar_data)) * 360
            relative_angle = ray_angle - current_angle
            
            # Normalize angle to -180 to 180
            while relative_angle > 180:
                relative_angle -= 360
            while relative_angle < -180:
                relative_angle += 360
            
            # Check if this ray is in the front direction
            if abs(relative_angle) < front_angle_range:
                total_rays += 1
                if distance < safety_distance:
                    obstacle_count += 1
        
        # Only consider it an obstacle if more than 50% of front rays hit obstacles (less sensitive)
        if total_rays > 0 and (obstacle_count / total_rays) > 0.5:
            return True
        
        return False  # No significant obstacles in front

class ExplorationController(Controller):
    def __init__(self, robot):
        super().__init__(robot)
        self.safety_distance = 150  # Much larger safety distance
        self.exploration_speed = 0.8  # Slower speed for better control
        self.turn_speed = 0.5  # Even slower turning
        self.last_direction_change = 0
        self.direction_change_interval = 180  # 3 seconds at 60 FPS

    def update(self, keys, walls, collision_map):
        """Very conservative exploration with early obstacle detection."""
        if not self.active:
            return

        current_pos = self.robot.position
        current_angle = self.robot.angle

        # Get LiDAR data for obstacle detection
        lidar_data = self.robot.data.get('2D Lidar', np.zeros((1, 3)))
        
        if len(lidar_data) == 0:
            return

        # Check if there are obstacles in front (wider cone)
        front_obstacles = self.check_front_obstacles(lidar_data, current_angle)
        
        # Temporarily disable side wall detection to stop spinning
        # side_walls = self.check_side_walls(lidar_data, current_angle)
        side_walls = False
        
        if front_obstacles or side_walls:
            # Obstacle in front or dangerous side wall, turn slowly
            self.robot.speed = 0.0  # Stop moving forward
            self.robot.angular_velocity = self.turn_speed  # Turn slowly
            if front_obstacles:
                print("Front obstacle detected, turning slowly")
            else:
                print("Side wall detected, turning to avoid parallel collision")
        else:
            # Clear path ahead, move forward
            self.robot.speed = self.exploration_speed
            self.robot.angular_velocity = 0.0  # Go straight
            print("Clear path, moving forward")

    def check_front_obstacles(self, lidar_data, current_angle):
        """Check if there are obstacles in the forward direction with wider detection."""
        # Check the front 90 degrees (45 degrees on each side) - wider detection
        front_angle_range = 45  # degrees
        
        for i, (distance, x, y) in enumerate(lidar_data):
            # Calculate the angle of this LiDAR ray relative to robot's current angle
            ray_angle = (i / len(lidar_data)) * 360
            relative_angle = ray_angle - current_angle
            
            # Normalize angle to -180 to 180
            while relative_angle > 180:
                relative_angle -= 360
            while relative_angle < -180:
                relative_angle += 360
            
            # Check if this ray is in the front direction
            if abs(relative_angle) < front_angle_range:
                if distance < self.safety_distance:
                    print(f"Front obstacle detected at {distance} pixels, angle {relative_angle:.1f}°")
                    return True  # Obstacle detected in front
        
        return False  # No obstacles in front

    def check_side_walls(self, lidar_data, current_angle):
        """Check for side walls that could cause parallel collision."""
        # Check the sides (perpendicular to robot's direction) - much more conservative
        side_angle_range = 15  # degrees on each side (much narrower)
        side_safety_distance = 50  # Much closer detection for side walls
        
        for i, (distance, x, y) in enumerate(lidar_data):
            # Calculate the angle of this LiDAR ray relative to robot's current angle
            ray_angle = (i / len(lidar_data)) * 360
            relative_angle = ray_angle - current_angle
            
            # Normalize angle to -180 to 180
            while relative_angle > 180:
                relative_angle -= 360
            while relative_angle < -180:
                relative_angle += 360
            
            # Check if this ray is on the sides (perpendicular to forward direction)
            if abs(abs(relative_angle) - 90) < side_angle_range:
                if distance < side_safety_distance:
                    print(f"Side wall detected at {distance} pixels, angle {relative_angle:.1f}°")
                    return True  # Side wall detected
        
        return False  # No dangerous side walls

    def activate(self):
        """Activate the exploration controller."""
        super().activate()
        self.robot.speed = self.exploration_speed
        self.last_direction_change = 0
        print("Exploration mode activated - very conservative behavior")

class ObstacleAvoidanceController(Controller):
    def __init__(self, robot):
        super().__init__(robot)
        self.safety_distance = 60
        self.avoidance_strength = 0.2

    def update(self, keys, walls, collision_map):
        """Update obstacle avoidance controller."""
        if not self.active:
            return

        # Get LiDAR data for obstacle detection
        lidar_data = self.robot.data.get('2D Lidar', np.zeros((1, 3)))
        
        if len(lidar_data) == 0:
            return

        # Find closest obstacle
        min_distance = float('inf')
        min_angle = 0
        
        for distance, x, y in lidar_data:
            if distance < min_distance:
                min_distance = distance
                # Calculate angle to this point
                dx = x - self.robot.position[0]
                dy = y - self.robot.position[1]
                min_angle = math.degrees(math.atan2(dy, dx))

        # If obstacle is too close, apply avoidance
        if min_distance < self.safety_distance:
            # Calculate avoidance direction (opposite to obstacle)
            avoidance_angle = min_angle + 180  # Opposite direction
            
            # Calculate desired angle
            current_angle = self.robot.angle
            angle_diff = avoidance_angle - current_angle
            
            # Normalize angle difference
            while angle_diff > 180:
                angle_diff -= 360
            while angle_diff < -180:
                angle_diff += 360

            # Apply avoidance control
            self.robot.angular_velocity = angle_diff * self.avoidance_strength
            self.robot.speed = max(0.5, self.robot.speed * 0.8)  # Slow down
        else:
            # No obstacles, maintain current behavior
            pass