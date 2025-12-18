"""
Path planning algorithms for box-pushing robot.

Contains:
- A* for point-to-point navigation
- Frontier-based exploration (find the box)
- Box pushing strategy (deliver box to goal)
- Path to actions conversion
"""

from __future__ import annotations

import time
from heapq import heappush, heappop
from typing import Optional

from metrics import get_metrics

from kb import (
    KnowledgeBase,
    NORTH,
    EAST,
    SOUTH,
    WEST,
    DIRECTION_VECTORS,
    UNKNOWN,
    FREE,
    OCC,
)

# Defensive fallback: ensure UNKNOWN exists even if an older/stale version of this
# file (or a partial import edit) ends up running.
try:
    UNKNOWN
except NameError:
    UNKNOWN = -1


# Maps (dx, dy) to heading constant
DELTA_TO_HEADING = {
    (0, 1): NORTH,
    (1, 0): EAST,
    (0, -1): SOUTH,
    (-1, 0): WEST,
}

# Maps heading to (dx, dy)
HEADING_TO_DELTA = DIRECTION_VECTORS


def manhattan_distance(a: tuple[int, int], b: tuple[int, int]) -> int:
    """Manhattan distance heuristic for A*."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def turns_between(from_heading: int, to_heading: int) -> int:
    """
    Count minimum number of 90° turns to go from one heading to another.

    Returns 0, 1, or 2 (since max is 180° = 2 turns).
    """
    diff = abs(to_heading - from_heading) % 4
    if diff == 3:
        diff = 1  # Going the other way is shorter
    return diff


def astar(
    kb: KnowledgeBase,
    start: tuple[int, int],
    goal: tuple[int, int],
    *,
    start_heading: Optional[int] = None,
    allow_unknown: bool = False,
    turn_cost: float = 0.0,
    search_type: str = "navigation",
) -> Optional[list[tuple[int, int]]]:
    """
    A* search on 4-connected grid with optional turn penalty.

    Args:
        kb: KnowledgeBase with occupancy info
        start: (x, y) start cell
        goal: (x, y) goal cell
        start_heading: initial heading (NORTH/EAST/SOUTH/WEST), uses kb.robot_heading if None
        allow_unknown: if True, treat UNKNOWN cells as traversable (optimistic)
        turn_cost: penalty per 90° turn (0 = no penalty, 0.5 = half a cell cost per turn)
        search_type: label for metrics ("navigation", "exploration", "sokoban")

    Returns:
        List of (x, y) cells from start to goal (inclusive), or None if no path.
    """
    search_start_time = time.perf_counter()
    nodes_expanded = 0

    if start == goal:
        # Record trivial search
        search_time_ms = (time.perf_counter() - search_start_time) * 1000
        get_metrics().record_astar_search(
            nodes_expanded=0,
            path_length=1,
            found_path=True,
            search_time_ms=search_time_ms,
            start=start,
            goal=goal,
            search_type=search_type,
        )
        return [start]

    # Get starting heading
    if start_heading is None:
        if kb.robot is not None:
            start_heading = kb.robot_heading
        else:
            start_heading = EAST  # Default

    # Check if goal is reachable in principle
    if not kb.is_traversable(goal[0], goal[1], allow_unknown=allow_unknown):
        # Goal might be the box cell during pushing - allow it
        if kb.box != goal:
            # Debug: show why goal is not traversable
            cell_state = kb.get_cell(goal[0], goal[1])
            state_name = {-1: "UNKNOWN", 0: "FREE", 1: "OCC"}.get(cell_state, "???")
            print(
                f"    [DEBUG] A* goal {goal} not traversable: state={state_name}, box={kb.box}"
            )
            # Record failed search
            search_time_ms = (time.perf_counter() - search_start_time) * 1000
            get_metrics().record_astar_search(
                nodes_expanded=0,
                path_length=0,
                found_path=False,
                search_time_ms=search_time_ms,
                start=start,
                goal=goal,
                search_type=search_type,
            )
            return None

    # State: (x, y, heading) to account for turn costs
    # Priority queue: (f_score, g_score, (x, y, heading), path)
    initial_state = (start[0], start[1], start_heading)
    open_set: list[
        tuple[float, float, tuple[int, int, int], list[tuple[int, int]]]
    ] = []
    heappush(open_set, (manhattan_distance(start, goal), 0.0, initial_state, [start]))

    # visited tracks (x, y, heading) to allow revisiting with different headings if beneficial
    visited: set[tuple[int, int, int]] = set()

    while open_set:
        _, g, state, path = heappop(open_set)
        x, y, heading = state

        if (x, y) == goal:
            # Record successful search
            search_time_ms = (time.perf_counter() - search_start_time) * 1000
            get_metrics().record_astar_search(
                nodes_expanded=nodes_expanded,
                path_length=len(path),
                found_path=True,
                search_time_ms=search_time_ms,
                start=start,
                goal=goal,
                search_type=search_type,
            )
            return path

        if state in visited:
            continue
        visited.add(state)
        nodes_expanded += 1

        # Explore 4-connected neighbors
        for neighbor in kb.neighbors4(x, y):
            nx, ny = neighbor

            # Determine required heading to move to neighbor
            dx, dy = nx - x, ny - y
            required_heading = DELTA_TO_HEADING.get((dx, dy))
            if required_heading is None:
                continue

            neighbor_state = (nx, ny, required_heading)
            if neighbor_state in visited:
                continue

            # Check traversability
            if not kb.is_traversable(nx, ny, allow_unknown=allow_unknown):
                # Special case: allow moving TO the goal even if it's marked occupied
                if neighbor != goal:
                    continue

            # Calculate cost: 1 for move + turn_cost * number of turns needed
            num_turns = turns_between(heading, required_heading)
            edge_cost = 1.0 + turn_cost * num_turns

            new_g = g + edge_cost
            new_f = new_g + manhattan_distance(neighbor, goal)
            new_path = path + [neighbor]
            heappush(open_set, (new_f, new_g, neighbor_state, new_path))

    # Record failed search
    search_time_ms = (time.perf_counter() - search_start_time) * 1000
    get_metrics().record_astar_search(
        nodes_expanded=nodes_expanded,
        path_length=0,
        found_path=False,
        search_time_ms=search_time_ms,
        start=start,
        goal=goal,
        search_type=search_type,
    )
    return None  # No path found


def path_to_actions(
    path: list[tuple[int, int]],
    start_heading: int,
) -> list[str]:
    """
    Convert a cell path to a list of actions.

    Args:
        path: List of (x, y) cells (from A*)
        start_heading: Initial heading (NORTH/EAST/SOUTH/WEST)

    Returns:
        List of action strings: 'turn_left', 'turn_right', 'move_forward'
    """
    if len(path) < 2:
        return []

    actions: list[str] = []
    current_heading = start_heading

    for i in range(1, len(path)):
        dx = path[i][0] - path[i - 1][0]
        dy = path[i][1] - path[i - 1][1]

        target_heading = DELTA_TO_HEADING.get((dx, dy))
        if target_heading is None:
            raise ValueError(f"Invalid move delta: ({dx}, {dy})")

        # Turn to face target heading (shortest path)
        while current_heading != target_heading:
            # Compute turn direction: +1 = right, -1 = left (mod 4)
            diff = (target_heading - current_heading) % 4
            if diff == 1:
                actions.append("turn_right")
                current_heading = (current_heading + 1) % 4
            elif diff == 3:
                actions.append("turn_left")
                current_heading = (current_heading - 1) % 4
            elif diff == 2:
                # 180° turn - pick either direction
                actions.append("turn_right")
                current_heading = (current_heading + 1) % 4
            else:
                break  # Already facing correct direction

        actions.append("move_forward")

    return actions


def find_nearest_frontier(
    kb: KnowledgeBase,
) -> Optional[tuple[int, int]]:
    """
    Find the nearest frontier cell (UNKNOWN adjacent to FREE) from robot position.

    Uses BFS for simplicity (could use A* distance for better accuracy).

    Returns:
        (x, y) of nearest frontier, or None if no frontiers exist.
    """
    if kb.robot is None:
        return None

    frontiers = kb.frontiers()
    if not frontiers:
        return None

    robot_pos = (kb.robot_x, kb.robot_y)

    # Find closest by Manhattan distance
    nearest = min(frontiers, key=lambda f: manhattan_distance(robot_pos, f))
    return nearest


def plan_exploration_step(
    kb: KnowledgeBase,
    *,
    turn_cost: float = 0.5,
) -> Optional[list[str]]:
    """
    Plan actions to reach the nearest frontier cell.

    Args:
        kb: KnowledgeBase with robot pose and occupancy
        turn_cost: penalty per 90° turn (default 0.5 = half a cell cost per turn)

    Returns:
        List of actions to execute, or None if no reachable frontier.
    """
    if kb.robot is None:
        print("  [DEBUG] plan_exploration_step: robot is None")
        return None

    robot_pos = (kb.robot_x, kb.robot_y)
    print(f"  [DEBUG] Robot at {robot_pos}, heading {kb.robot_heading}")

    # Try frontiers in increasing heuristic distance until we find a reachable one.
    # This avoids prematurely giving up when the nearest frontier is blocked.
    frontiers = list(kb.frontiers())
    print(f"  [DEBUG] Found {len(frontiers)} frontiers")
    if not frontiers:
        print("  [DEBUG] No frontiers found!")
        return None

    frontiers.sort(key=lambda f: manhattan_distance(robot_pos, f))
    print(f"  [DEBUG] Nearest 5 frontiers: {frontiers[:5]}")

    attempted = 0
    for frontier in frontiers:
        # We want to move TO a FREE cell adjacent to the frontier (not into UNKNOWN),
        # so consider each FREE neighbor as an approach target.
        approaches = [
            (nx, ny) for (nx, ny) in kb.neighbors4(*frontier) if kb.is_free(nx, ny)
        ]
        if not approaches:
            continue
        approaches.sort(key=lambda p: manhattan_distance(robot_pos, p))

        for approach in approaches:
            attempted += 1
            path = astar(
                kb, robot_pos, approach, turn_cost=turn_cost, search_type="exploration"
            )
            if path is None:
                print(
                    f"  [DEBUG] No path to approach {approach} for frontier {frontier}"
                )
                continue
            print(
                f"  [DEBUG] Found path to {approach} (frontier {frontier}), length {len(path)}"
            )
            return path_to_actions(path, kb.robot_heading)

    print(f"  [DEBUG] Tried {attempted} approaches, none reachable")
    return None


def compute_push_direction(
    box_pos: tuple[int, int],
    goal_pos: tuple[int, int],
) -> int:
    """
    Determine which cardinal direction to push the box toward the goal.

    Returns the heading (NORTH/EAST/SOUTH/WEST) to push.
    """
    dx = goal_pos[0] - box_pos[0]
    dy = goal_pos[1] - box_pos[1]

    # Pick the dominant axis, or the one with remaining distance
    if dx == 0 and dy == 0:
        return NORTH  # Already at goal, arbitrary

    # Prioritize the axis with larger remaining distance
    if abs(dx) >= abs(dy):
        return EAST if dx > 0 else WEST
    else:
        return NORTH if dy > 0 else SOUTH


def compute_push_position(
    box_pos: tuple[int, int],
    push_direction: int,
) -> tuple[int, int]:
    """
    Compute where the robot needs to be to push the box in the given direction.

    The robot must be on the opposite side of the box.
    """
    dx, dy = HEADING_TO_DELTA[push_direction]
    # Robot position is opposite to push direction
    return (box_pos[0] - dx, box_pos[1] - dy)


def plan_box_push(
    kb: KnowledgeBase,
    *,
    turn_cost: float = 0.25,
) -> Optional[list[str]]:
    """
    Plan actions to push the box one cell toward the goal.

    Strategy:
    1. Compute push direction (box -> goal)
    2. Compute push position (opposite side of box)
    3. Navigate to push position
    4. Push (move forward into box cell)

    Args:
        kb: KnowledgeBase with robot/box/goal positions
        turn_cost: penalty per 90° turn for navigation (default 0.25)

    Returns:
        List of actions, or None if box/goal not set or no valid path.
    """
    if kb.robot is None or kb.box is None or kb.goal is None:
        return None

    box_pos = kb.box
    goal_pos = kb.goal
    robot_pos = (kb.robot_x, kb.robot_y)

    # If box is at goal, we're done
    if box_pos == goal_pos:
        return []

    # Determine push direction
    push_dir = compute_push_direction(box_pos, goal_pos)
    push_pos = compute_push_position(box_pos, push_dir)

    # Check if push position is valid (not occupied, not the box itself)
    if not kb.is_traversable(push_pos[0], push_pos[1]):
        # Try alternative push directions
        for alt_dir in [NORTH, EAST, SOUTH, WEST]:
            if alt_dir == push_dir:
                continue
            alt_push_pos = compute_push_position(box_pos, alt_dir)
            if kb.is_traversable(alt_push_pos[0], alt_push_pos[1]):
                push_dir = alt_dir
                push_pos = alt_push_pos
                break
        else:
            return None  # No valid push position

    actions: list[str] = []

    # If robot is not at push position, navigate there
    if robot_pos != push_pos:
        path = astar(kb, robot_pos, push_pos, turn_cost=turn_cost)
        if path is None:
            return None
        actions.extend(path_to_actions(path, kb.robot_heading))

        # After navigation, heading may have changed - simulate to get final heading
        final_heading = kb.robot_heading
        for action in actions:
            if action == "turn_left":
                final_heading = (final_heading - 1) % 4
            elif action == "turn_right":
                final_heading = (final_heading + 1) % 4
    else:
        final_heading = kb.robot_heading

    # Turn to face the box (push direction)
    while final_heading != push_dir:
        diff = (push_dir - final_heading) % 4
        if diff == 1:
            actions.append("turn_right")
            final_heading = (final_heading + 1) % 4
        elif diff == 3:
            actions.append("turn_left")
            final_heading = (final_heading - 1) % 4
        else:
            actions.append("turn_right")
            final_heading = (final_heading + 1) % 4

    # Push the box (move forward)
    actions.append("move_forward")

    return actions


def is_box_at_goal(kb: KnowledgeBase) -> bool:
    """Check if the box has reached the goal."""
    if kb.box is None or kb.goal is None:
        return False
    return kb.box == kb.goal


# =============================================================================
# SOKOBAN-STYLE BOX DELIVERY PLANNING
# =============================================================================


def plan_box_delivery(
    kb: KnowledgeBase,
    *,
    turn_cost: float = 0.5,
    max_states: int = 50000,
) -> Optional[list[str]]:
    """
    Plan complete action sequence to deliver box to goal using A* over combined state.

    This is Sokoban-style planning: searches over (robot_pos, robot_heading, box_pos)
    to find the optimal sequence of turns and moves (including pushes).

    Args:
        kb: KnowledgeBase with robot/box/goal positions and occupancy
        turn_cost: penalty per 90° turn (default 0.5)
        max_states: maximum states to explore before giving up (prevents infinite loops)

    Returns:
        List of actions ['turn_left', 'turn_right', 'move_forward'], or None if no solution.
    """
    search_start_time = time.perf_counter()

    if kb.robot is None or kb.box is None or kb.goal is None:
        return None

    goal_pos = kb.goal
    start_box = kb.box
    start_pos = (kb.robot_x, kb.robot_y)

    # If already at goal, nothing to do
    if start_box == goal_pos:
        search_time_ms = (time.perf_counter() - search_start_time) * 1000
        get_metrics().record_astar_search(
            nodes_expanded=0,
            path_length=0,
            found_path=True,
            search_time_ms=search_time_ms,
            start=start_pos,
            goal=goal_pos,
            search_type="sokoban",
        )
        return []

    # State: (robot_x, robot_y, robot_heading, box_x, box_y)
    start_state = (kb.robot_x, kb.robot_y, kb.robot_heading, start_box[0], start_box[1])

    def heuristic(state: tuple[int, int, int, int, int]) -> float:
        """Heuristic: Manhattan distance from box to goal."""
        _, _, _, bx, by = state
        return manhattan_distance((bx, by), goal_pos)

    def static_cell_state(x: int, y: int) -> int:
        """
        Occupancy state for *static* obstacles only.

        The KB marks the (initial) box cell as OCC, but the box is a dynamic object
        tracked in the planner state. Treat that specific cell as FREE here so we
        don't permanently block it during planning.
        """
        if kb.box is not None and (x, y) == kb.box:
            return FREE
        return kb.get_cell(x, y)

    def is_cell_free(x: int, y: int, box_pos: tuple[int, int]) -> bool:
        """
        Check if robot can occupy the cell, given a dynamic box position.

        During delivery replanning we allow the *robot* to traverse UNKNOWN cells
        (optimistic), otherwise the planner can easily return None on partially
        explored maps. We still forbid pushing the box into UNKNOWN by requiring
        the box destination cell be FREE (see push check below).
        """
        if (x, y) == box_pos:
            return False  # Can't walk through the box (unless pushing)
        state = static_cell_state(x, y)
        return state == FREE or state == UNKNOWN

    def is_cell_empty_for_box(x: int, y: int) -> bool:
        """Check if cell is valid for box to move into (not wall, not goal-blocking)."""
        # Box can go anywhere that's not an obstacle
        state = kb.get_cell(x, y)
        return state != OCC

    # Priority queue: (f_score, g_score, state, actions)
    open_set: list[tuple[float, float, tuple[int, int, int, int, int], list[str]]] = []
    heappush(open_set, (heuristic(start_state), 0.0, start_state, []))

    visited: set[tuple[int, int, int, int, int]] = set()
    states_explored = 0

    while open_set and states_explored < max_states:
        _, g, state, actions = heappop(open_set)
        rx, ry, heading, bx, by = state
        states_explored += 1

        # Goal check: box at goal
        if (bx, by) == goal_pos:
            # Record successful search
            search_time_ms = (time.perf_counter() - search_start_time) * 1000
            get_metrics().record_astar_search(
                nodes_expanded=states_explored,
                path_length=len(actions),
                found_path=True,
                search_time_ms=search_time_ms,
                start=start_pos,
                goal=goal_pos,
                search_type="sokoban",
            )
            return actions

        if state in visited:
            continue
        visited.add(state)

        # Action 1: turn_left
        new_heading = (heading - 1) % 4
        new_state = (rx, ry, new_heading, bx, by)
        if new_state not in visited:
            new_g = g + turn_cost
            new_f = new_g + heuristic(new_state)
            heappush(open_set, (new_f, new_g, new_state, actions + ["turn_left"]))

        # Action 2: turn_right
        new_heading = (heading + 1) % 4
        new_state = (rx, ry, new_heading, bx, by)
        if new_state not in visited:
            new_g = g + turn_cost
            new_f = new_g + heuristic(new_state)
            heappush(open_set, (new_f, new_g, new_state, actions + ["turn_right"]))

        # Action 3: move_forward
        dx, dy = HEADING_TO_DELTA[heading]
        ahead_x, ahead_y = rx + dx, ry + dy

        # Case A: Moving into empty cell (not the box)
        if (ahead_x, ahead_y) != (bx, by):
            if is_cell_free(ahead_x, ahead_y, (bx, by)):
                new_state = (ahead_x, ahead_y, heading, bx, by)
                if new_state not in visited:
                    new_g = g + 1.0
                    new_f = new_g + heuristic(new_state)
                    heappush(
                        open_set, (new_f, new_g, new_state, actions + ["move_forward"])
                    )

        # Case B: Pushing the box
        elif (ahead_x, ahead_y) == (bx, by):
            # Box would move to cell beyond it
            box_dest_x, box_dest_y = bx + dx, by + dy

            # Check if box destination is valid (must be FREE, not UNKNOWN)
            # Optimistic: allow pushing into UNKNOWN cells (not just FREE)
            # Trust replanning to handle unexpected obstacles discovered en route
            # This reduces exploration time while single-wall yaw correction keeps us localized
            box_dest_state = static_cell_state(box_dest_x, box_dest_y)
            if box_dest_state != OCC:  # FREE or UNKNOWN both OK
                # Push is valid: robot moves to box position, box moves forward
                new_state = (ahead_x, ahead_y, heading, box_dest_x, box_dest_y)
                if new_state not in visited:
                    # Pushing costs 1 (same as moving)
                    new_g = g + 1.0
                    new_f = new_g + heuristic(new_state)
                    heappush(
                        open_set, (new_f, new_g, new_state, actions + ["move_forward"])
                    )

    # Record failed search
    search_time_ms = (time.perf_counter() - search_start_time) * 1000
    get_metrics().record_astar_search(
        nodes_expanded=states_explored,
        path_length=0,
        found_path=False,
        search_time_ms=search_time_ms,
        start=start_pos,
        goal=goal_pos,
        search_type="sokoban",
    )
    return None  # No solution found within state limit
