"""
Box Pushing Robot - CS152 Final Project

Main entry point for the box-pushing mission.

Architecture:
- simulation.py : PyBullet setup, environment creation
- kb.py         : Knowledge base (beliefs about the world)
- mapping.py    : Sensor interpretation (LiDAR → occupancy)
- planning.py   : Path planning (A*, exploration, box delivery)
- robot.py      : Low-level robot control (wheels, sensors)
- mission.py    : Mission execution (plan → actions → movements)
- metrics.py    : Performance metrics collection
- main.py       : Entry point (this file)

Usage:
    uv run python main.py              # Default test case
    uv run python main.py --test A     # Run test case A (straight push)
    uv run python main.py --test B     # Run test case B (one turn)
    uv run python main.py --test C     # Run test case C (two turns)
"""

import argparse
import json

import simulation
import kb
from robot import RobotController, EAST
from mission import MissionController
from metrics import reset_metrics, get_metrics

# Test case configurations: (box_world_grid, goal_world_grid, description)
# Robot always starts at PyBullet world grid (1, 1) → internal (0, 0), facing EAST
# Room is 5m x 5m, grid cells are 0.5m, so valid grid range is 0-9
TEST_CASES = {
    "A": {
        "name": "Straight Push",
        # Robot at (1,1), box at (4,1), goal at (7,1) - all on same row, robot facing east
        "box_pos": (2.25, 0.75),  # World meters - grid (4,1) → internal (3, 0)
        "goal_grid": (7, 1),  # World grid → internal (6, 0) - straight east
        "description": "Box directly east of robot; straight push, no turns needed",
    },
    "B": {
        "name": "One Turn",
        # Box is north, goal is further north - robot must turn once to push
        "box_pos": (0.75, 2.25),  # World meters - grid (1,4) → internal (0, 3)
        "goal_grid": (1, 7),  # World grid → internal (0, 6)
        "description": "Box north of robot; requires one turn to face and push",
    },
    "C": {
        "name": "Two Turns",
        # Box diagonal, goal in different direction - needs repositioning
        "box_pos": (2.25, 2.25),  # World meters - grid (4,4) → internal (3, 3)
        "goal_grid": (1, 7),  # World grid → internal (0, 6) - push west then north
        "description": "Box diagonal; multiple reorientations needed",
    },
    "default": {
        "name": "Default",
        "box_pos": (1.25, 4.25),  # Original default position
        "goal_grid": (7, 7),
        "description": "Standard test configuration",
    },
}


def main(test_case: str = "default"):
    """Run the box-pushing mission."""
    # Reset metrics for fresh run
    reset_metrics()

    print("=" * 60)
    print("Box Pushing Robot - CS152 Final Project")
    print("=" * 60)

    # Get test case config
    config = TEST_CASES.get(test_case, TEST_CASES["default"])
    print(f"\nTest Case: {config['name']}")
    print(f"Description: {config['description']}")

    # Connect to PyBullet
    print("\n[1] Connecting to PyBullet...")
    try:
        client_id = simulation.connect(gui=True)
        if client_id < 0:
            raise RuntimeError("PyBullet GUI connection failed")
        print("    Connected to GUI!")
        realtime = False
    except Exception:
        print("    GUI unavailable, using DIRECT mode")
        client_id = simulation.connect(gui=False)
        if client_id < 0:
            raise RuntimeError("PyBullet DIRECT connection failed")
        realtime = False

    # Create environment with test case configuration
    print("\n[2] Creating environment...")
    env = simulation.create_environment(
        room_size=5.0,
        box_pos=config["box_pos"],
        goal_grid=config["goal_grid"],
    )
    print(
        f"    Grid: {env['grid_size']}x{env['grid_size']} cells ({simulation.CELL_SIZE}m each)"
    )
    print(f"    Box at (world grid): {env['box_grid']} (unknown to robot)")
    print(f"    Goal at (world grid): {env['goal_grid']} (known to robot)")

    # Robot's internal frame: starts at (0,0)
    # Convert world coordinates to internal frame
    pybullet_start = (1, 1)  # Robot spawn position in PyBullet world grid

    # Goal position in internal frame (relative to robot start)
    goal_internal = (
        env["goal_grid"][0] - pybullet_start[0],
        env["goal_grid"][1] - pybullet_start[1],
    )
    # Box position in internal frame (debug-only; robot does not know it)
    box_internal = (
        env["box_grid"][0] - pybullet_start[0],
        env["box_grid"][1] - pybullet_start[1],
    )

    # Initialize Knowledge Base (cells start as UNKNOWN for exploration)
    print("\n[3] Initializing Knowledge Base...")
    knowledge = kb.KnowledgeBase()
    # Constrain the KB to the real room bounds in the INTERNAL frame.
    # PyBullet grid is [0..grid_size-1]; internal grid = pybullet - pybullet_start.
    min_x = -pybullet_start[0]
    max_x = env["grid_size"] - 1 - pybullet_start[0]
    min_y = -pybullet_start[1]
    max_y = env["grid_size"] - 1 - pybullet_start[1]
    knowledge.set_bounds(min_x, max_x, min_y, max_y)
    knowledge.set_goal(*goal_internal)
    print("    All cells start as UNKNOWN (exploration mode)")
    print(f"    Goal marked at: {goal_internal} (internal frame)")
    print(f"    Box (debug only): {box_internal} (internal frame)")

    # Load robot at PyBullet position (internal frame is (0,0))
    start_heading = EAST
    print(f"\n[4] Loading robot at PyBullet grid {pybullet_start}...")
    robot_id = simulation.load_robot(pybullet_start, start_heading)

    # KB uses internal frame: robot starts at (0,0)
    knowledge.set_robot(0, 0, start_heading)

    # Create controller (internal frame starts at 0,0)
    controller = RobotController(
        robot_id,
        knowledge,
        wall_ids=env["wall_ids"],
    )

    # Let simulation settle
    simulation.settle()

    # Initial scan to populate KB around start position
    print("\n[5] Initial LiDAR scan...")
    scan_data = controller.scan_lidar(visualize=realtime)
    knowledge.print_grid()

    # Record wall landmarks from known position (0,0) for later recalibration
    print("\n[6] Recording wall landmarks from initial position...")
    num_landmarks = controller.record_wall_landmarks(scan_data)
    print(f"    Recorded {num_landmarks} wall landmarks for drift correction")

    # Create mission controller
    mission = MissionController(
        controller,
        knowledge,
        box_id=env["box_id"],
    )

    # Record test case configuration in metrics (before mission starts)
    # We do this here because we know the actual box position from env,
    # whereas the KB doesn't know box position until exploration finds it
    get_metrics().set_test_case(
        name=config["name"],
        robot_pos=(0, 0),  # Internal frame start
        box_pos=box_internal,  # Actual box position in internal frame
        goal_pos=goal_internal,  # Goal position in internal frame
    )

    # Run the full mission
    print("\n[7] Starting mission...")
    success = mission.run_full_mission(realtime=realtime)

    # Final state
    print("\n" + "=" * 60)
    if success:
        print("MISSION SUCCESSFUL!")
    else:
        print("MISSION FAILED")
    print("=" * 60)

    knowledge.print_grid()

    # Export metrics to JSON file
    metrics_dict = get_metrics().to_dict()
    metrics_file = f"metrics_{test_case}.json"
    with open(metrics_file, "w") as f:
        json.dump(metrics_dict, f, indent=2)
    print(f"\nMetrics saved to {metrics_file}")

    # Print LaTeX table for copy-paste
    print("\n--- LaTeX Table (copy-paste ready) ---")
    print(get_metrics().to_latex_table())

    # Keep window open for inspection (GUI mode only)
    if realtime:
        import pybullet

        print("\nPress Ctrl+C to exit...")
        try:
            while pybullet.isConnected():
                simulation.step()
        except KeyboardInterrupt:
            pass
        pybullet.disconnect()

    return success


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Box Pushing Robot - CS152 Final Project"
    )
    parser.add_argument(
        "--test",
        type=str,
        default="default",
        choices=["A", "B", "C", "default"],
        help="Test case to run: A (straight), B (one turn), C (two turns), default",
    )
    args = parser.parse_args()

    main(test_case=args.test)
