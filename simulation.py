import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "1"
import pygame
import yaml
from Car import Car
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
    FPS = 60
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
        print(f"Loading car: {car_cfg['id']}")
        car = Car(
            origin=car_cfg["position"],
            angle=car_cfg["angle"],
            speed=car_cfg["speed"],
            screen=screen
        )
        cars.append(car)
    
    # Need to impliment the subjects via config file above

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

def draw_room():
    screen.fill(BG_COLOR)
    for obs in walls:
        pygame.draw.rect(screen, WALL_COLOR, obs)
    
    # Draw all cars
    for car in cars:
        car.draw(screen, collision_map)
    pygame.display.update()

collision_map = build_collision_map(walls, WIDTH, HEIGHT)


def main():
    running = True
    while running:
        clock.tick(FPS)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        keys = pygame.key.get_pressed()

        # Check key presses for various actions
        if keys[pygame.K_ESCAPE]: # Pressing Escape Closes Window
            running = False
        if keys[pygame.K_r]: # {Pressing resets all cars to their original positions}
            for car in cars:
                car.reset_position()
        

        for car in cars:
            car.update(keys, walls, collision_map)
        draw_room()

    pygame.quit()
    print("Simulation closed.\n")

if __name__ == "__main__":
    main()

