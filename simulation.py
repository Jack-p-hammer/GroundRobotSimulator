import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "1"
import pygame
import yaml
from src.Car import Car
import numpy as np
import argparse

parser = argparse.ArgumentParser(description='A script that demonstrates argument parsing.')

parser.add_argument('input_file', help='The path to the input file.')
args = parser.parse_args()

print(f"Input file: {args.input_file}")

config_file = args.input_file
if not os.path.exists(config_file):
    print(f"Config file '{config_file}' does not exist. Please provide a valid path.")
    exit(1)

print("\n\n")
print("Loading config file and setting up simulation...\n")
try:
    # Load config
    with open(config_file, "r") as f:
        config = yaml.safe_load(f)

    WIDTH = config["window"]["width"]
    HEIGHT = config["window"]["height"]
    FPS = config["window"]["FPS"]
    BG_COLOR = (220, 220, 220)
    WALL_COLOR = (0, 0, 0)
    
    # Initialize pygame
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption(config["window"]["title"])
    clock = pygame.time.Clock()

    # Obstacles from config
    obstacles = [pygame.Rect(*coords) for coords in config["obstacles"]]

    # Add room walls to obstacles
    wall_thickness = 5
    walls = obstacles + [
        pygame.Rect(0, 0, WIDTH, wall_thickness),               # Top
        pygame.Rect(0, 0, wall_thickness, HEIGHT),              # Left
        pygame.Rect(0, HEIGHT - wall_thickness, WIDTH, wall_thickness),  # Bottom
        pygame.Rect(WIDTH - wall_thickness, 0, wall_thickness, HEIGHT)   # Right
    ]

    # Instantiate all cars from config
    cars = []
    for car_cfg in config["cars"]:
        car = Car(
            origin=car_cfg["position"],
            angle=car_cfg["angle"],
            speed=car_cfg["speed"],
            screen=screen,
            car_size= car_cfg["size"],
            add_ons= car_cfg.get("subjects", []),  # Pass subjects as add-ons
            channels = car_cfg.get("channels", []),
            server = config["server"]
        )
        cars.append(car)
    
    # Need to implement the subjects via config file above

    print("simulation loaded.\n")
except:
    print("Error loading config file or initializing simulation.\n")
    raise

def build_collision_map(walls, width, height):
    collision_map = np.zeros((width, height), dtype=bool)
    for wall in walls:
        left, top, w, h = wall
        collision_map[left:left + w, top:top + h] = True
    return collision_map

def draw_ui(screen, cars):
    """Draw UI elements including control mode and instructions."""
    font = pygame.font.Font(None, 24)
    
    # Draw control instructions
    instructions = [
        "Controls:",
        "M - Manual mode",
        "N - Navigation mode", 
        "E - Exploration mode",
        "A - Autonomous mode",
        "R - Reset cars",
        "G - Regenerate exploration points",
        "Click minimap to set goal",
        "ESC - Quit"
    ]
    
    y_offset = 10
    for instruction in instructions:
        text = font.render(instruction, True, (0, 0, 0))
        screen.blit(text, (WIDTH - 200, y_offset))
        y_offset += 20
    
    # Draw car status
    y_offset = 200
    for i, car in enumerate(cars):
        status_text = f"Car {i+1}: {car.control_mode}"
        text = font.render(status_text, True, (0, 0, 0))
        screen.blit(text, (WIDTH - 200, y_offset))
        y_offset += 20
        
        # Show goal if in navigation mode
        if car.control_mode == 'navigation' and hasattr(car, 'navigation_controller'):
            if car.navigation_controller.goal_position is not None:
                goal_text = f"  Goal: ({car.navigation_controller.goal_position[0]:.0f}, {car.navigation_controller.goal_position[1]:.0f})"
                text = font.render(goal_text, True, (0, 100, 0))
                screen.blit(text, (WIDTH - 200, y_offset))
                y_offset += 20
        
        # Show exploration info if in exploration mode
        if car.control_mode == 'exploration' and hasattr(car, 'exploration_controller'):
            # Check if robot is turning or moving forward
            lidar_data = car.data.get('2D Lidar', np.zeros((1, 3)))
            turning = False
            turning_reason = ""
            
            if len(lidar_data) > 0:
                # Check front obstacles with wider cone
                front_angle_range = 45  # Wider detection
                front_obstacle = False
                for i, (distance, x, y) in enumerate(lidar_data):
                    ray_angle = (i / len(lidar_data)) * 360
                    relative_angle = ray_angle - car.angle
                    while relative_angle > 180:
                        relative_angle -= 360
                    while relative_angle < -180:
                        relative_angle += 360
                    
                    if abs(relative_angle) < front_angle_range:
                        if distance < car.exploration_controller.safety_distance:
                            front_obstacle = True
                            break
                
                if front_obstacle:
                    turning = True
                    turning_reason = "Front"
            
            if turning:
                status = f"Turning ({turning_reason})"
                color = (255, 165, 0)  # Orange for turning
            else:
                status = "Forward"
                color = (0, 0, 100)  # Blue for forward
            
            exp_text = f"  Status: {status}"
            text = font.render(exp_text, True, color)
            screen.blit(text, (WIDTH - 200, y_offset))
            y_offset += 20

def draw_room():
    screen.fill(BG_COLOR)
    for obs in walls:
        pygame.draw.rect(screen, WALL_COLOR, obs)
    
    # Draw all cars
    for car in cars:
        car.draw(screen, collision_map)
    
    # Draw UI
    draw_ui(screen, cars)
    
    pygame.display.update()

collision_map = build_collision_map(walls, WIDTH, HEIGHT)

def main():
    running = True
    exploration_update_counter = 0  # Counter for periodic exploration updates
    
    while running:
        clock.tick(FPS)
        exploration_update_counter += 1
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # Handle mouse clicks for goal setting
                mouse_pos = pygame.mouse.get_pos()
                for car in cars:
                    if car.handle_mouse_click(mouse_pos):
                        break  # Only handle click for first car that accepts it

        keys = pygame.key.get_pressed()

        # Check key presses for various actions
        if keys[pygame.K_ESCAPE]: # Pressing Escape Closes Window
            running = False
        if keys[pygame.K_r]: # Pressing r resets all cars to their original positions
            for car in cars:
                car.reset_position()
                car.set_control_mode('manual')
        
        # Control mode switching
        if keys[pygame.K_m]:  # Manual mode
            for car in cars:
                car.set_control_mode('manual')
        elif keys[pygame.K_n]:  # Navigation mode
            for car in cars:
                car.set_control_mode('navigation')
        elif keys[pygame.K_e]:  # Exploration mode
            for car in cars:
                car.set_control_mode('exploration')
        elif keys[pygame.K_a]:  # Autonomous mode
            for car in cars:
                car.set_control_mode('autonomous')
        elif keys[pygame.K_g]:  # Regenerate exploration points
            for car in cars:
                if car.control_mode == 'exploration' and hasattr(car, 'occupancy_grid'):
                    print("Exploration mode now uses systematic wall following - no points needed")

        # Periodic exploration point regeneration (every 2 seconds at 60 FPS)
        if exploration_update_counter >= 120:
            # No longer needed with new exploration controller
            exploration_update_counter = 0

        for car in cars:
            car.update(keys, walls, collision_map)
        draw_room()

    pygame.quit()
    print("Simulation closed.\n")

if __name__ == "__main__":
    main()

