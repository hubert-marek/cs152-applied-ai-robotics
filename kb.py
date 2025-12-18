from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from math import pi
from typing import Iterator, Optional


UNKNOWN, FREE, OCC = -1, 0, 1

# Angles are assigned for debugging and readibiliy purpouse.
NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3
ORIENTATION_NAMES = {NORTH: "N", EAST: "E", SOUTH: "S", WEST: "W"}
ORIENTATION_ANGLES = {NORTH: pi / 2, EAST: 0.0, SOUTH: -pi / 2, WEST: pi}
DIRECTION_VECTORS = {NORTH: (0, 1), EAST: (1, 0), SOUTH: (0, -1), WEST: (-1, 0)}


@dataclass(slots=True)
class Pose:
    """Discrete pose on the grid. Dataclass used in KB"""

    x: int
    y: int
    heading: int  # NORTH/EAST/SOUTH/WEST


class KnowledgeBase:
    """
    Sparse grid Knowledge Base (KB).

    Core idea:
    - `occ[(x,y)]` stores occupancy only (UNKNOWN/FREE/OCC).
    - Robot/goal/box are stored as separate fields (not encoded into occ).
    """

    def __init__(self):
        self.occ = defaultdict(lambda: UNKNOWN)
        self.visited = set()  # {(rx, ry)}
        self.robot: Optional[Pose] = None
        self.goal: Optional[tuple[int, int]] = None
        self.box: Optional[tuple[int, int]] = None

    # TODO I think we need to remove the set bounds function to be exactly alligned with room KB? Or it does not have influence on acutal robot only helps us in debugging purpuses?
    def set_bounds(self, min_x: int, max_x: int, min_y: int, max_y: int) -> None:
        """Set finite grid bounds for planning/mapping (inclusive)."""
        if min_x > max_x or min_y > max_y:
            raise ValueError("Invalid bounds")
        self.bounds = (min_x, max_x, min_y, max_y)

    def in_bounds(self, x: int, y: int) -> bool:
        """Helper to return True if (x,y) is within bounds (or if bounds are not set)."""
        if self.bounds is None:
            return True
        min_x, max_x, min_y, max_y = self.bounds
        return min_x <= x <= max_x and min_y <= y <= max_y

    def set_robot(self, x: int, y: int, heading: int) -> None:
        """Set the robot pose and mark its cell as visited/free."""
        if not self.in_bounds(x, y):
            raise ValueError(f"Robot pose out of bounds: ({x}, {y})")
        self.robot = Pose(x, y, heading)
        self.visited.add((x, y))
        self.mark_free(x, y)

    def update_pose_from_continuous(
        self, x_meters: float, y_meters: float, yaw_radians: float, cell_size: float
    ) -> None:
        """
        Update robot pose from continuous measurements (real odometry). Even tough the robot has capability of turning directly to specifc angles
        We will only allow 4 directions of movment for pathfinding algorithms.

        Converts continuous (x, y, yaw) to discrete grid pose.
        - Grid cell = round((meters - cell_size/2) / cell_size)
        - Heading = nearest cardinal direction from yaw

        Args:
            x_meters: x position in meters
            y_meters: y position in meters
            yaw_radians: heading in radians (0 = East, pi/2 = North, etc.)
            cell_size: meters per grid cell
        """
        # Convert to grid cells (cell centers at i*cell_size + cell_size/2)
        grid_x = int(round((x_meters - cell_size / 2) / cell_size))
        grid_y = int(round((y_meters - cell_size / 2) / cell_size))

        # Convert yaw to nearest cardinal heading
        # EAST=0°, NORTH=90°, WEST=180°, SOUTH=-90°
        # Normalize yaw to [0, 2π)
        yaw_normalized = yaw_radians % (2 * pi)

        # Map to heading: each quadrant is 90° = π/2
        # EAST: [-45°, 45°) → [0, π/4) ∪ [7π/4, 2π)
        # NORTH: [45°, 135°) → [π/4, 3π/4)
        # WEST: [135°, 225°) → [3π/4, 5π/4)
        # SOUTH: [225°, 315°) → [5π/4, 7π/4)
        if yaw_normalized < pi / 4 or yaw_normalized >= 7 * pi / 4:
            heading = EAST
        elif yaw_normalized < 3 * pi / 4:
            heading = NORTH
        elif yaw_normalized < 5 * pi / 4:
            heading = WEST
        else:
            heading = SOUTH

        if self.robot is None:
            self.robot = Pose(grid_x, grid_y, heading)
        else:
            self.robot.x = grid_x
            self.robot.y = grid_y
            self.robot.heading = heading

        self.visited.add((grid_x, grid_y))
        self.mark_free(grid_x, grid_y)

    def move_robot(self, new_x: int, new_y: int) -> None:
        """Move robot to a new grid cell (keeps current heading)."""
        if self.robot is None:
            raise ValueError("Robot pose not set")
        if not self.in_bounds(new_x, new_y):
            raise ValueError(f"Robot move out of bounds: ({new_x}, {new_y})")
        self.robot.x = new_x
        self.robot.y = new_y
        self.visited.add((new_x, new_y))
        self.mark_free(new_x, new_y)

    def step_turn(self, direction: str) -> None:
        """Update robot heading after a discrete turn ('left'/'right')."""
        if self.robot is None:
            raise ValueError("Robot pose not set")
        if direction == "left":
            self.robot.heading = (self.robot.heading - 1) % 4
        elif direction == "right":
            self.robot.heading = (self.robot.heading + 1) % 4
        else:
            raise ValueError("direction must be 'left' or 'right'")

    def set_goal(self, x: int, y: int) -> None:
        """Set the goal cell (treated as traversable)."""
        if not self.in_bounds(x, y):
            raise ValueError(f"Goal out of bounds: ({x}, {y})")
        self.goal = (x, y)
        self.mark_free(x, y)

    def set_box(self, x: int, y: int) -> None:
        """Set/overwrite the box cell."""
        if not self.in_bounds(x, y):
            # Ignore obviously-invalid localization results instead of poisoning the KB.
            # TODO - I Think this should not be here because we are doing SLAM? And we dont know the bounds fo the room?
            return
        self.box = (x, y)
        # The box is a dynamic object, not a static obstacle. So we do nOT encode it into the occupancy grid, otherwise the box leaves behind
        # "ghost walls" when it moves. Path planning already treats `self.box` as non-traversable via `is_traversable()`.
        self.mark_free(x, y)

    def move_box(self, new_x: int, new_y: int) -> None:
        """Update box position after a push. Clears old cell, marks new as occupied."""
        if not self.in_bounds(new_x, new_y):
            return
        if self.box is not None:
            # Clear old box cell (mark as free since robot can now pass through)
            old_x, old_y = self.box
            self.mark_free(old_x, old_y)
        self.box = (new_x, new_y)
        # Same rationale as in set_box(): keep dynamic box out of occ grid.
        self.mark_free(new_x, new_y)

    def clear_box(self) -> None:
        """Clear box knowledge (does not modify occupancy)."""
        self.box = None

    # Convenience properties

    @property
    def robot_x(self) -> int:
        return 0 if self.robot is None else self.robot.x

    @property
    def robot_y(self) -> int:
        return 0 if self.robot is None else self.robot.y

    @property
    def robot_heading(self) -> int:
        """Robot heading (NORTH/EAST/SOUTH/WEST). Defaults to EAST if not set."""
        return EAST if self.robot is None else self.robot.heading

    @property
    def box_x(self) -> int | None:
        return None if self.box is None else self.box[0]

    @property
    def box_y(self) -> int | None:
        return None if self.box is None else self.box[1]

    @property
    def box_found(self) -> bool:
        return self.box is not None

    @property
    def goal_x(self) -> int | None:
        return None if self.goal is None else self.goal[0]

    @property
    def goal_y(self) -> int | None:
        return None if self.goal is None else self.goal[1]

    # Convenience methods

    def get_cell_ahead(self) -> tuple[int, int]:
        """Get grid cell directly ahead of robot."""
        dx, dy = DIRECTION_VECTORS[self.robot_heading]
        return (self.robot_x + dx, self.robot_y + dy)

    def is_box_at_goal(self) -> bool:
        """Check if box is at goal position."""
        return self.box is not None and self.goal is not None and self.box == self.goal

    def update_from_move(self, direction: str) -> None:
        """Update KB after robot moves ('forward' or 'backward')."""
        dx, dy = DIRECTION_VECTORS[self.robot_heading]
        if direction == "forward":
            new_x, new_y = self.robot_x + dx, self.robot_y + dy
        elif direction == "backward":
            new_x, new_y = self.robot_x - dx, self.robot_y - dy
        else:
            raise ValueError("direction must be 'forward' or 'backward'")
        self.move_robot(new_x, new_y)

    def update_from_turn(self, direction: str) -> None:
        """Update KB after robot turns ('left' or 'right'). Alias for step_turn."""
        self.step_turn(direction)

    def get_robot_state(self) -> str:
        """Get human-readable robot state string."""
        return f"({self.robot_x}, {self.robot_y}) facing {ORIENTATION_NAMES[self.robot_heading]}"

    def print_grid(self, pad: int = 1) -> None:
        """Print ASCII grid to stdout."""
        print(self.to_ascii(pad=pad))
        box_str = f"Box: ({self.box_x}, {self.box_y})" if self.box_found else "Box: ?"
        goal_str = f"Goal: ({self.goal_x}, {self.goal_y})" if self.goal else "Goal: ?"
        print(f"Robot: {self.get_robot_state()} | {goal_str} | {box_str}")

    def get_cell(self, x: int, y: int) -> int:
        if not self.in_bounds(x, y):
            return OCC
        return self.occ[(x, y)]

    def set_cell(self, x: int, y: int, state: int) -> None:
        if state not in (UNKNOWN, FREE, OCC):
            raise ValueError("state must be UNKNOWN, FREE, or OCC")
        if not self.in_bounds(x, y):
            return
        self.occ[(x, y)] = state

    def is_unknown(self, x: int, y: int) -> bool:
        return self.get_cell(x, y) == UNKNOWN

    def is_free(self, x: int, y: int) -> bool:
        return self.get_cell(x, y) == FREE

    def is_occupied(self, x: int, y: int) -> bool:
        return self.get_cell(x, y) == OCC

    def mark_free(self, x: int, y: int) -> None:
        if not self.in_bounds(x, y):
            return
        # Allow clearing an OCC cell if later evidence indicates it is free.
        # (This also prevents "ghost obstacles" if a dynamic object was ever
        # incorrectly written into occ.)
        self.occ[(x, y)] = FREE

    def mark_occupied(self, x: int, y: int) -> None:
        if not self.in_bounds(x, y):
            return
        self.occ[(x, y)] = OCC

    def is_traversable(self, x: int, y: int, *, allow_unknown: bool = False) -> bool:
        """
        Traversable for robot path planning.

        - FREE is traversable
        - UNKNOWN can optionally be treated as traversable (useful for optimistic exploration)
        - OCC is not traversable
        - If box is known, treat its cell as non-traversable.
        """
        if not self.in_bounds(x, y):
            return False
        if self.box is not None and (x, y) == self.box:
            return False
        state = self.get_cell(x, y)
        return state == FREE or (allow_unknown and state == UNKNOWN)

    def neighbors4(self, x: int, y: int) -> Iterator[tuple[int, int]]:
        """4-connected neighbors grid"""
        yield (x + 1, y)
        yield (x - 1, y)
        yield (x, y + 1)
        yield (x, y - 1)

    def frontiers(self) -> set[tuple[int, int]]:
        """
        Return frontier cells: UNKNOWN cells adjacent (4-neigh) to FREE cells.
        Works naturally for sparse maps by expanding around known FREE cells.
        """
        fr: set[tuple[int, int]] = set()
        # Iterate over all the cells in the grid
        for (x, y), state in self.occ.items():
            # If the cell is not FREE, skip it
            if state != FREE:
                continue
            # Iterate over all the neighbors of the cell
            for nx, ny in self.neighbors4(x, y):
                # If the neighbor is UNKNOWN, add it to the frontier
                if self.is_unknown(nx, ny):
                    fr.add((nx, ny))
        return fr

    # Helper Debug output
    def to_ascii(self, *, pad: int = 1) -> str:
        """
        Render a small ASCII map of known cells, including robot/goal/box overlay.
        For sparse maps, bounds are derived from known cells + objects.
        """
        points: set[tuple[int, int]] = set(self.occ.keys())
        if self.robot is not None:
            points.add((self.robot.x, self.robot.y))
        if self.goal is not None:
            points.add(self.goal)
        if self.box is not None:
            points.add(self.box)

        if not points:
            points = {(0, 0)}

        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        min_x, max_x = min(xs) - pad, max(xs) + pad
        min_y, max_y = min(ys) - pad, max(ys) + pad

        robot_symbol = {NORTH: "^", EAST: ">", SOUTH: "v", WEST: "<"}

        lines: list[str] = []
        for y in range(max_y, min_y - 1, -1):
            row = []
            for x in range(min_x, max_x + 1):
                if self.robot is not None and (x, y) == (self.robot.x, self.robot.y):
                    row.append(robot_symbol[self.robot.heading])
                    continue
                if self.goal is not None and (x, y) == self.goal:
                    row.append("G")
                    continue
                if self.box is not None and (x, y) == self.box:
                    row.append("B")
                    continue
                v = self.get_cell(x, y)
                row.append({UNKNOWN: "?", FREE: ".", OCC: "#"}[v])
            lines.append("".join(row))
        return "\n".join(lines)
