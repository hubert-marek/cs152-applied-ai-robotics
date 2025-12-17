"""
Simulation Environment - PyBullet setup and environment creation.

Centralizes all simulation parameters and environment building.
"""

import os

import pybullet
import pybullet_data

# =============================================================================
# SIMULATION PARAMETERS
# =============================================================================

TIME_STEP = 1.0 / 240.0  # Physics timestep (240 Hz)
CELL_SIZE = 0.5  # Grid cell size in meters


# =============================================================================
# ENVIRONMENT SETUP
# =============================================================================


def connect(gui: bool = True) -> int:
    """
    Connect to PyBullet.

    Args:
        gui: If True, open GUI window. If False, run headless (DIRECT mode).

    Returns:
        Physics client ID.
    """
    mode = pybullet.GUI if gui else pybullet.DIRECT
    client_id = pybullet.connect(mode)
    return client_id


def create_environment(room_size: float = 5.0) -> dict:
    """
    Create simulation environment with walls, box, goal, and grid.

    Args:
        room_size: Size of the square room in meters.

    Returns:
        Dict with environment info:
        - plane_id, wall_ids, box_id, goal_id
        - room_size, grid_size
        - box_grid, goal_grid (initial grid positions)
    """
    pybullet.resetSimulation()
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    pybullet.setGravity(0, 0, -9.8)
    pybullet.setTimeStep(TIME_STEP)
    pybullet.setPhysicsEngineParameter(enableConeFriction=1)

    # Ground plane
    plane_id = pybullet.loadURDF("plane.urdf")
    pybullet.changeDynamics(plane_id, -1, lateralFriction=1.0)

    # Walls
    wall_ids = _create_walls(room_size)

    # Box (pushable object)
    box_id, box_pos = _create_box(room_size)

    # Goal marker
    goal_id, goal_pos = _create_goal()

    # Grid visualization
    _draw_grid(room_size)

    # Camera
    pybullet.resetDebugVisualizerCamera(7.0, 45, -45, [room_size / 2, room_size / 2, 0])

    grid_cells = int(room_size / CELL_SIZE)
    return {
        "plane_id": plane_id,
        "wall_ids": wall_ids,
        "box_id": box_id,
        "goal_id": goal_id,
        "room_size": room_size,
        "grid_size": grid_cells,
        # Grid conversion: cell centers are at (i*CELL_SIZE + CELL_SIZE/2)
        "box_grid": (
            int(round((box_pos[0] - CELL_SIZE / 2) / CELL_SIZE)),
            int(round((box_pos[1] - CELL_SIZE / 2) / CELL_SIZE)),
        ),
        "goal_grid": (
            int(round((goal_pos[0] - CELL_SIZE / 2) / CELL_SIZE)),
            int(round((goal_pos[1] - CELL_SIZE / 2) / CELL_SIZE)),
        ),
    }


def _create_walls(room_size: float) -> list[int]:
    """Create room walls. Returns list of wall body IDs."""
    wall_thickness = 0.1
    wall_height = 0.5
    wall_ids = []

    # Horizontal wall shapes
    h_col = pybullet.createCollisionShape(
        pybullet.GEOM_BOX,
        halfExtents=[room_size / 2, wall_thickness / 2, wall_height / 2],
    )
    h_vis = pybullet.createVisualShape(
        pybullet.GEOM_BOX,
        halfExtents=[room_size / 2, wall_thickness / 2, wall_height / 2],
        rgbaColor=[0.5, 0.5, 0.5, 1],
    )

    # Vertical wall shapes
    v_col = pybullet.createCollisionShape(
        pybullet.GEOM_BOX,
        halfExtents=[wall_thickness / 2, room_size / 2, wall_height / 2],
    )
    v_vis = pybullet.createVisualShape(
        pybullet.GEOM_BOX,
        halfExtents=[wall_thickness / 2, room_size / 2, wall_height / 2],
        rgbaColor=[0.5, 0.5, 0.5, 1],
    )

    # South wall
    wall_ids.append(
        pybullet.createMultiBody(
            0, h_col, h_vis, [room_size / 2, -wall_thickness / 2, wall_height / 2]
        )
    )
    # North wall
    wall_ids.append(
        pybullet.createMultiBody(
            0,
            h_col,
            h_vis,
            [room_size / 2, room_size + wall_thickness / 2, wall_height / 2],
        )
    )
    # West wall
    wall_ids.append(
        pybullet.createMultiBody(
            0, v_col, v_vis, [-wall_thickness / 2, room_size / 2, wall_height / 2]
        )
    )
    # East wall
    wall_ids.append(
        pybullet.createMultiBody(
            0,
            v_col,
            v_vis,
            [room_size + wall_thickness / 2, room_size / 2, wall_height / 2],
        )
    )

    return wall_ids


def _create_box(room_size: float) -> tuple[int, list[float]]:
    """Create pushable box. Returns (box_id, position)."""
    box_size = 0.3  # 20cm box for compact robot
    box_mass = 0.5  # Slightly heavier for more realistic physics
    box_pos = [3.25, 3.25, box_size / 2]

    box_col = pybullet.createCollisionShape(
        pybullet.GEOM_BOX, halfExtents=[box_size / 2] * 3
    )
    box_vis = pybullet.createVisualShape(
        pybullet.GEOM_BOX, halfExtents=[box_size / 2] * 3, rgbaColor=[1, 0.5, 0, 1]
    )
    box_id = pybullet.createMultiBody(box_mass, box_col, box_vis, box_pos)
    pybullet.changeDynamics(box_id, -1, lateralFriction=1)

    return box_id, box_pos


def _create_goal() -> tuple[int, list[float]]:
    """Create goal marker. Returns (goal_id, position)."""
    goal_pos = [1.25, 1.25, 0.01]
    goal_vis = pybullet.createVisualShape(
        pybullet.GEOM_CYLINDER, radius=0.2, length=0.02, rgbaColor=[0, 1, 0, 0.5]
    )
    goal_id = pybullet.createMultiBody(
        0, baseVisualShapeIndex=goal_vis, basePosition=goal_pos
    )
    return goal_id, goal_pos


def _draw_grid(room_size: float):
    """Draw grid lines on floor."""
    grid_cells = int(room_size / CELL_SIZE)
    for i in range(grid_cells + 1):
        pos = i * CELL_SIZE
        pybullet.addUserDebugLine(
            [0, pos, 0.01], [room_size, pos, 0.01], [0.3, 0.3, 0.3]
        )
        pybullet.addUserDebugLine(
            [pos, 0, 0.01], [pos, room_size, 0.01], [0.3, 0.3, 0.3]
        )


def load_robot(start_grid_pos: tuple[int, int], start_orientation: int) -> int:
    """
    Load robot at grid position.

    Args:
        start_grid_pos: (x, y) grid cell
        start_orientation: Cardinal direction (NORTH=0, EAST=1, SOUTH=2, WEST=3)

    Returns:
        Robot body ID.
    """
    import math

    # Orientation angles (same as in robot.py, imported here to avoid circular dep)
    ORIENTATION_ANGLES = {0: math.pi / 2, 1: 0, 2: -math.pi / 2, 3: math.pi}

    world_x = start_grid_pos[0] * CELL_SIZE + CELL_SIZE / 2
    world_y = start_grid_pos[1] * CELL_SIZE + CELL_SIZE / 2
    yaw = ORIENTATION_ANGLES[start_orientation]

    urdf_path = os.path.join(
        os.path.dirname(__file__),
        "PybulletRobotics",
        "urdf",
        "simple_two_wheel_car.urdf",
    )

    robot_id = pybullet.loadURDF(
        urdf_path,
        [world_x, world_y, 0.10],  # Wheels have 5cm radius; keep them on the ground
        pybullet.getQuaternionFromEuler([0, 0, yaw]),
    )

    # Friction tuning for simple_two_wheel_car.urdf
    # Driven wheels are joints 0 and 1 in this URDF.
    WHEEL_JOINTS = [0, 1]

    pybullet.changeDynamics(robot_id, -1, lateralFriction=0.2)
    for link in range(pybullet.getNumJoints(robot_id)):
        friction = 5.0 if link in WHEEL_JOINTS else 0.2
        pybullet.changeDynamics(robot_id, link, lateralFriction=friction)

    return robot_id


def step(n: int = 1):
    """Step simulation forward n timesteps."""
    for _ in range(n):
        pybullet.stepSimulation()


def settle(steps: int = 100):
    """Let simulation settle (for physics stabilization after loading objects)."""
    step(steps)
