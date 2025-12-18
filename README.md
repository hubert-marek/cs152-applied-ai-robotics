# Box Pushing Robot - CS152 Applied AI Robotics

A PyBullet-based simulation of an autonomous robot that explores an environment, locates a box, and pushes it to a goal location using A* path planning and sensor-based localization.

## Quick Start

### Prerequisites

- Python 3.11+
- [uv](https://github.com/astral-sh/uv) package manager (recommended)

### Installation

```bash
# Clone the repository
git clone <repository-url>
cd cs152-applied-ai-robotics

# Install dependencies with uv
uv sync
```

### Running the Simulation

```bash
# Run default test case
uv run python main.py

# Run specific test cases
uv run python main.py --test A    # Straight push (easiest)
uv run python main.py --test B    # One turn required
uv run python main.py --test C    # Multiple turns (hardest)
```

## Test Cases

| Test | Name | Description |
|------|------|-------------|
| A | Straight Push | Box directly east of robot; no turns needed |
| B | One Turn | Box north of robot; requires one turn to push |
| C | Two Turns | Box diagonal; multiple reorientations needed |
| default | Default | Standard test configuration |

## Configuration

### Test Case Parameters (`main.py`)

Edit `TEST_CASES` dictionary to customize box and goal positions:

```python
TEST_CASES = {
    "A": {
        "name": "Straight Push",
        "box_pos": (2.25, 0.75),   # World position in meters (x, y)
        "goal_grid": (7, 1),       # World grid cell (gx, gy)
        "description": "...",
    },
    # Add your own test cases here
}
```

**Coordinate system:**
- Room is 5m × 5m by default
- Grid cells are 0.5m × 0.5m (10×10 grid)
- Robot starts at world grid (1, 1), facing EAST
- `box_pos`: World position in meters `(x, y)`
- `goal_grid`: World grid coordinates `(gx, gy)` where `gx, gy ∈ [0, 9]`

### Simulation Parameters (`simulation.py`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `TIME_STEP` | 1/240 s | Physics timestep (240 Hz) |
| `CELL_SIZE` | 0.5 m | Grid cell size |
| `room_size` | 5.0 m | Room dimensions |

### Robot Parameters (`robot.py`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MAX_MOVE_SPEED` | 10 rad/s | Max wheel speed for movement |
| `MIN_MOVE_SPEED` | 2 rad/s | Min wheel speed for movement |
| `MAX_TURN_SPEED` | 5 rad/s | Max wheel speed for turning |
| `MIN_TURN_SPEED` | 1.5 rad/s | Min wheel speed for turning |
| `DISTANCE_TOLERANCE` | 0.005 m | Waypoint arrival threshold |
| `ANGLE_TOLERANCE` | 1.5° | Heading alignment threshold |
| `WHEEL_RADIUS` | 0.05 m | Wheel radius (from URDF) |

### Box Properties (`simulation.py`, `_create_box()`)

```python
box_size = 0.25   # Box side length in meters (25cm cube)
box_mass = 1.0    # Box mass in kg
```

### LiDAR Parameters (`robot.py`, `LiDAR` class)

```python
num_rays = 60         # Number of rays per scan
max_range = 2.5       # Maximum detection range (meters)
ray_start_offset = 0.1  # Ray start offset from sensor
```

## Output

### Metrics Files

After each run, metrics are saved to `metrics_{test_case}.json`:

```json
{
  "test_case": { "name": "...", "robot_start": [...], "box_start": [...], "goal": [...] },
  "mission": { "success": true, "wall_clock_sec": 4.5, "simulated_sec": 15.2, "sim_steps": 3648 },
  "phases": {
    "exploration": { "duration_sec": 0.5, "astar_searches": 2, ... },
    "delivery": { "duration_sec": 4.0, "push_attempts": 3, ... }
  },
  "astar": { "total_searches": 5, "total_nodes": 150, ... },
  "push": { "total_attempts": 3, "success_rate": 100.0 }
}
```

### Console Output

The simulation prints detailed logs including:
- Robot pose (internal and actual)
- LiDAR scan statistics
- A* search results
- Push attempt outcomes
- Recalibration events

## Project Structure

```
cs152-applied-ai-robotics/
├── main.py           # Entry point, test case definitions
├── simulation.py     # PyBullet environment setup
├── kb.py             # Knowledge base (robot beliefs)
├── mapping.py        # LiDAR → occupancy grid
├── planning.py       # A* path planning (navigation + Sokoban)
├── robot.py          # Robot controller (wheels, sensors, pose)
├── mission.py        # High-level mission execution
├── metrics.py        # Performance metrics collection
├── algorithms/       # Algorithm documentation
│   ├── README.md
│   ├── controller.md
│   └── coordinate_transforms.md
└── PybulletRobotics/ # Reference URDF files
    └── urdf/
        └── simple_two_wheel_car.urdf
```
