# Ground Robot Navigation Simulation

An advanced 2D ground robot simulation with mapping, path planning, and autonomous navigation capabilities.

<img width="1596" height="1264" alt="image" src="https://github.com/user-attachments/assets/783f08b5-d001-4ede-a2f4-4a9beecfff23" />


## Features

### 🗺️ **Mapping & Navigation**
- **Real-time Occupancy Grid Mapping**: Robots build maps using LiDAR data
- **A* Path Planning**: Intelligent pathfinding to avoid obstacles
- **Goal-based Navigation**: Click anywhere on the minimap to set navigation goals
- **Multi-robot Support**: Run multiple robots simultaneously

### 🎮 **Control Modes**
- **Manual Mode (M)**: Direct keyboard control (arrow keys)
- **Navigation Mode (N)**: Autonomous navigation to clicked goals
- **Exploration Mode (E)**: Autonomous exploration of unknown areas
- **Autonomous Mode (A)**: Full autonomy with obstacle avoidance

### 📊 **Sensors & Perception**
- **360° LiDAR**: High-resolution laser scanning
- **Distance Sensors**: Forward-facing obstacle detection
- **IMU**: Inertial measurement for orientation
- **Occupancy Grid**: Real-time mapping with exploration tracking

### 🖱️ **Interactive Features**
- **Click-to-Navigate**: Click on minimap to set goals
- **Visual Feedback**: Different colors for different control modes
- **Exploration Tracking**: See percentage of area explored
- **Path Visualization**: Watch planned paths in real-time

## Quick Start

### Prerequisites
```bash
pip install -r requirements.txt
```

### Running the Simulation
```bash
python simulation.py config.yaml
```

### Controls
- **Arrow Keys**: Manual control (in manual mode)
- **M**: Switch to manual mode
- **N**: Switch to navigation mode
- **E**: Switch to exploration mode
- **A**: Switch to autonomous mode
- **R**: Reset all robots to starting positions
- **Click Minimap**: Set navigation goal
- **ESC**: Quit simulation

## How It Works

### 1. **Mapping Process**
The robots use LiDAR sensors to scan their environment and build an occupancy grid map:
- **Red areas**: Obstacles detected by LiDAR
- **White areas**: Free space confirmed by sensors
- **Gray areas**: Unexplored regions

### 2. **Path Planning**
When you click a goal on the minimap:
1. The system converts minimap coordinates to world coordinates
2. A* pathfinding algorithm finds the optimal path
3. The robot follows waypoints along the planned path
4. Blue lines show the planned route on the minimap

### 3. **Navigation Modes**

#### Manual Mode (Red Robot)
- Direct keyboard control
- Use arrow keys to drive
- Space to stop

#### Navigation Mode (Green Robot)
- Autonomous navigation to goals
- Follows planned paths
- Stops when goal is reached

#### Exploration Mode (Blue Robot)
- Autonomous exploration of unknown areas
- Builds map while moving
- Can be given specific exploration points

#### Autonomous Mode (Orange Robot)
- Combines navigation with obstacle avoidance
- Safest mode for complex environments
- Automatically avoids obstacles while following paths

## Project Structure

```
GroundRobots/
├── simulation.py          # Main simulation file
├── src/
│   ├── Car.py            # Robot class with navigation
│   ├── ControlMethods.py # Navigation controllers
│   ├── Perception.py     # Mapping and path planning
│   ├── Sensors.py        # LiDAR, IMU, distance sensors
│   └── config.yaml       # Configuration file
├── requirements.txt       # Python dependencies
└── README.md            # This file
```

## Configuration

Edit `src/config.yaml` to customize:

### Robot Settings
```yaml
cars:
  - id: car1
    position: [400, 200]  # Starting position
    angle: 0              # Starting orientation
    speed: 0.75           # Base speed
    size: [40, 20]        # Robot dimensions
```

### Sensor Configuration
```yaml
subjects:
  - type: "2D Lidar"
    num_points: 120       # Number of LiDAR rays
    range: 300           # Maximum range
    resolution: 1        # Distance resolution
```

### Environment
```yaml
obstacles:
  - [200, 150, 100, 100]  # [x, y, width, height]
```

## Advanced Features

### Multi-Robot Coordination
- Run multiple robots simultaneously
- Each robot has its own map and navigation
- Robots can operate independently or cooperatively

### Real-time Communication
- Redis integration for inter-robot communication
- Publish/subscribe channels for data sharing
- Extensible for distributed systems

### Sensor Fusion
- Combine data from multiple sensors
- IMU for orientation tracking
- Distance sensors for obstacle avoidance
- LiDAR for mapping and navigation

## Technical Details

### Architecture
- **Modular Design**: Separate classes for sensors, perception, and control
- **Extensible**: Easy to add new sensors or control algorithms
- **Configurable**: All parameters in YAML configuration
- **Real-time**: 60 FPS simulation with smooth visualization

### Algorithms
- **A* Pathfinding**: Optimal path planning with obstacle avoidance
- **Pure Pursuit**: Smooth path following for mobile robots
- **Occupancy Grid**: Probabilistic mapping with exploration tracking
- **LiDAR Simulation**: Ray-casting for realistic sensor data

### Performance
- **Efficient Collision Detection**: Pre-computed collision maps
- **Optimized Rendering**: Hardware-accelerated graphics
- **Memory Efficient**: Sparse data structures for large maps

## Future Enhancements

### Planned Features
- **SLAM**: Simultaneous Localization and Mapping
- **Multi-robot SLAM**: Collaborative mapping
- **Dynamic Obstacles**: Moving objects in environment
- **3D Visualization**: Elevation and terrain mapping
- **Machine Learning**: Neural network-based navigation
- **ROS Integration**: Robot Operating System compatibility

### Research Applications
- **Autonomous Vehicle Simulation**: Test navigation algorithms
- **Multi-robot Coordination**: Study swarm behaviors
- **Sensor Fusion Research**: Combine multiple sensor types
- **Path Planning Algorithms**: Compare different approaches

