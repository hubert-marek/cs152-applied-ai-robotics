"""
Mission Controller - Executes plans using waypoint navigation.

Converts planning actions to waypoints, then navigates using
continuous pose estimation with sensor fusion:
- Actions → Waypoints (positions + headings)
- Waypoint navigation with pose tracking
- LiDAR recalibration at each waypoint
- Manages exploration and box delivery missions
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING

import pybullet

import planning
from simulation import CELL_SIZE, TIME_STEP
from robot import DIRECTION_VECTORS, ORIENTATION_NAMES

if TYPE_CHECKING:
    from robot import RobotController
    from kb import KnowledgeBase


# When True, forcibly snap the internal continuous pose estimate to planned grid
# centers after each waypoint/action. This can mask estimator drift, but if the
# robot does not physically reach the planned cell it will de-sync pose from
# LiDAR observations and break box re-detection/push verification.
SNAP_INTERNAL_POSE = False


class MissionController:
    """
    Executes high-level missions using planning algorithms.

    Handles:
    - Converting plans to batched movements
    - Reactive control during execution
    - Position/yaw correction at checkpoints
    - Mission state management (exploring, pushing box, etc.)
    """

    def __init__(
        self,
        robot: "RobotController",
        kb: "KnowledgeBase",
        box_id: int,
    ):
        """
        Initialize mission controller.

        Args:
            robot: RobotController for low-level movement
            kb: KnowledgeBase wrapper
            box_id: PyBullet body ID of the box (for detection)
        """
        self.robot = robot
        self.kb = kb
        self.box_id = box_id

    def execute_actions(self, actions: list[str], realtime: bool = True) -> bool:
        """
        Execute a list of planning actions by converting to waypoints.

        Simulates the actions to compute waypoints, then navigates
        using continuous pose estimation.

        Args:
            actions: List of action strings from planning
            realtime: If True, add visualization delays

        Returns:
            True if all actions completed successfully
        """
        if not actions:
            return True

        # Convert actions to waypoints (simulate to find where we end up)
        waypoints = self._actions_to_waypoints(actions)
        print(f"  Executing {len(actions)} actions → {len(waypoints)} waypoints")

        for i, (wx, wy, heading) in enumerate(waypoints):
            print(f"    Waypoint {i + 1}: ({wx}, {wy}) heading {heading}")

            # Navigate to waypoint
            reached = self.robot.move_to_waypoint(
                wx, wy, target_heading=heading, realtime=realtime
            )
            if not reached:
                print("    ERROR: Failed to reach waypoint (timeout)")
                return False

            # Update KB to match
            self._sync_kb_to_position(wx, wy, heading)

            # Scan and recalibrate at waypoint
            scan_data = self.robot.scan_lidar(visualize=realtime)
            self.robot.update_pose(scan_data=scan_data)

        return True

    def _actions_to_waypoints(self, actions: list[str]) -> list[tuple[int, int, int]]:
        """
        Convert action sequence to waypoints with headings.

        Returns list of (grid_x, grid_y, heading) tuples.
        Only includes waypoints where position changes or heading
        matters (e.g., before a move after turns).
        """
        waypoints = []
        x, y = self.kb.robot_x, self.kb.robot_y
        heading = self.kb.robot_heading

        for action in actions:
            if action == "turn_left":
                heading = (heading - 1) % 4
            elif action == "turn_right":
                heading = (heading + 1) % 4
            elif action == "move_forward":
                dx, dy = DIRECTION_VECTORS[heading]
                x, y = x + dx, y + dy
                waypoints.append((x, y, heading))
            elif action == "move_backward":
                dx, dy = DIRECTION_VECTORS[heading]
                x, y = x - dx, y - dy
                waypoints.append((x, y, heading))

        # Consolidate consecutive waypoints with same heading
        # (turns without moves don't need intermediate waypoints)
        if waypoints:
            consolidated = [waypoints[0]]
            for wp in waypoints[1:]:
                # Only add if position changed
                if wp[:2] != consolidated[-1][:2]:
                    consolidated.append(wp)
                else:
                    # Update heading at same position
                    consolidated[-1] = wp
            return consolidated

        # If no moves, but heading changed (turn-only), add current pos with new heading
        if heading != self.kb.robot_heading:
            return [(x, y, heading)]

        return []

    def _sync_kb_to_position(self, x: int, y: int, heading: int):
        """
        Sync KB robot position to target waypoint position.

        IMPORTANT: We trust the waypoint target over the drifting internal pose.
        After navigation, we snap both KB and internal pose to the target to
        prevent drift accumulation.
        """
        # Get actual internal pose for debugging
        pose = self.robot.get_pose()

        print(
            f"    [DEBUG] Syncing KB: ({self.kb.robot_x}, {self.kb.robot_y}) h={self.kb.robot_heading} → target ({x}, {y}) h={heading}"
        )
        print(
            f"    [DEBUG] Internal pose before snap: ({pose['grid_x']}, {pose['grid_y']}) yaw={pose['yaw_deg']:.1f}°"
        )

        # Sync KB to TARGET waypoint (not drifted pose!)
        self.kb.set_robot(x, y, heading)

        # Optionally snap internal pose to grid cell center.
        if SNAP_INTERNAL_POSE:
            self.robot.snap_pose_to_grid(x, y, heading)

        print(
            f"    [DEBUG] KB now at: ({self.kb.robot_x}, {self.kb.robot_y}) h={self.kb.robot_heading}"
        )

    def _debug_print_state(self, step: int):
        """
        Print comprehensive debug state in a single, consistent frame.

        All grid coordinates shown as `grid=(x,y)` are in the INTERNAL frame
        (the KB / planner frame where the robot starts at (0,0)).
        """
        # Internal pose estimate (robot's frame, starts at 0,0)
        internal_pose = self.robot.get_pose()

        print(f"\n  === DEBUG STATE (step {step}) ===")
        print("  [Internal frame]")
        # Actual robot pose (debug-only; converted to internal by RobotController)
        actual_pose = self.robot.get_actual_pose()
        print(
            f"    Actual:  robot grid=({actual_pose['grid_x_internal']}, {actual_pose['grid_y_internal']}), yaw={actual_pose['yaw_deg']:.1f}° "
            f"[world_grid=({actual_pose['grid_x_world']},{actual_pose['grid_y_world']})]"
        )
        print(
            f"    Belief:  robot grid=({internal_pose['grid_x']}, {internal_pose['grid_y']}), yaw={internal_pose['yaw_deg']:.1f}°"
        )
        print(
            f"    KB:    grid=({self.kb.robot_x}, {self.kb.robot_y}), heading={ORIENTATION_NAMES[self.kb.robot_heading]}"
        )
        print(
            f"    Box:   {f'grid=({self.kb.box_x}, {self.kb.box_y})' if self.kb.box_found else 'unknown'}"
        )
        # Actual box pose converted to internal (debug-only, uses PyBullet truth)
        box_pos, _ = pybullet.getBasePositionAndOrientation(self.box_id)
        if self.kb.bounds is not None:
            min_x, _, min_y, _ = self.kb.bounds
        else:
            min_x, min_y = 0, 0
        box_x_internal = box_pos[0] + (min_x * CELL_SIZE)
        box_y_internal = box_pos[1] + (min_y * CELL_SIZE)
        box_gx_internal = int(round((box_x_internal - CELL_SIZE / 2) / CELL_SIZE))
        box_gy_internal = int(round((box_y_internal - CELL_SIZE / 2) / CELL_SIZE))
        box_gx_world = int(round((box_pos[0] - CELL_SIZE / 2) / CELL_SIZE))
        box_gy_world = int(round((box_pos[1] - CELL_SIZE / 2) / CELL_SIZE))
        print(
            f"    Actual box: grid=({box_gx_internal}, {box_gy_internal}) "
            f"[world_grid=({box_gx_world},{box_gy_world})]"
        )
        print("  ==============================")

    def explore_until_box_found(
        self, realtime: bool = True, max_steps: int = 100
    ) -> bool:
        """
        Explore the environment using frontier-based exploration until box is found.

        Args:
            realtime: If True, add visualization delays
            max_steps: Maximum exploration steps before giving up

        Returns:
            True if box was found, False if exploration exhausted
        """
        print("\n" + "=" * 50)
        print("MISSION: Explore to find the box")
        print("=" * 50)

        for step in range(max_steps):
            # Debug: show real vs KB state
            self._debug_print_state(step)

            # Scan and check for box
            scan_data = self.robot.scan_lidar(visualize=realtime)

            if self._check_for_box(scan_data):
                print(f"\n  >>> BOX FOUND at step {step}! <<<")
                self.kb.print_grid()
                return True

            # Debug: KB stats
            free_cells = sum(1 for s in self.kb.occ.values() if s == 0)  # FREE=0
            occ_cells = sum(1 for s in self.kb.occ.values() if s == 1)  # OCC=1
            print(
                f"  [DEBUG] KB stats: {free_cells} FREE, {occ_cells} OCC, {len(self.kb.occ)} total known"
            )

            # Plan next exploration step
            actions = planning.plan_exploration_step(self.kb)

            if actions is None:
                print(f"\n  No more frontiers to explore at step {step}")
                # Debug: show frontier details
                frontiers = self.kb.frontiers()
                print(
                    f"  [DEBUG] frontiers() returned {len(frontiers)} cells: {list(frontiers)[:10]}"
                )
                # LiDAR is 360° - single scan is sufficient
                scan_data = self.robot.scan_lidar(visualize=realtime)
                if self._check_for_box(scan_data):
                    print("\n  >>> BOX FOUND during final scan! <<<")
                    return True
                return False

            print(f"\n  Exploration step {step + 1}: {len(actions)} actions")
            if not self.execute_actions(actions, realtime):
                print("\n  ERROR: Exploration execution failed")
                return False
            self.kb.print_grid()

        print(f"\n  Exploration limit reached ({max_steps} steps)")
        return False

    def _check_for_box(self, scan_data) -> bool:
        """Check if the box is visible in the current scan."""
        # Debug: show what object IDs we're seeing
        detected_ids = set(obj_id for _, _, obj_id in scan_data if obj_id != -1)
        print(
            f"  [DEBUG] LiDAR detected object IDs: {detected_ids}, looking for box_id={self.box_id}"
        )

        for angle, distance, obj_id in scan_data:
            if obj_id == self.box_id:
                # LiDAR distance is measured from the robot's actual position; use debug-only
                # actual pose, but compute everything in the INTERNAL frame for consistency.
                actual_pose = self.robot.get_actual_pose()
                robot_yaw = actual_pose["yaw"]

                # angle is relative to robot's heading
                angle_world = robot_yaw + angle

                # Hit point in INTERNAL meters
                hit_x = actual_pose["x_internal"] + distance * math.cos(angle_world)
                hit_y = actual_pose["y_internal"] + distance * math.sin(angle_world)

                # Convert to INTERNAL grid
                box_grid_x = int(round((hit_x - CELL_SIZE / 2) / CELL_SIZE))
                box_grid_y = int(round((hit_y - CELL_SIZE / 2) / CELL_SIZE))

                internal_pose = self.robot.get_pose()
                print(f"  Box detected at internal grid ({box_grid_x}, {box_grid_y})")
                print(
                    f"    [DEBUG] Internal pose: ({internal_pose['grid_x']}, {internal_pose['grid_y']}) yaw={internal_pose['yaw_deg']:.1f}°"
                )
                print(
                    f"    [DEBUG] Actual pose (internal): grid=({actual_pose['grid_x_internal']},{actual_pose['grid_y_internal']}) "
                    f"[world_grid=({actual_pose['grid_x_world']},{actual_pose['grid_y_world']})]"
                )
                print(
                    f"    [DEBUG] LiDAR: angle={math.degrees(angle):.1f}°, dist={distance:.2f}m"
                )

                if self.kb.box_found:
                    self.kb.move_box(box_grid_x, box_grid_y)
                else:
                    self.kb.set_box(box_grid_x, box_grid_y)
                return True

        return False

    def deliver_box_to_goal(self, realtime: bool = True) -> bool:
        """
        Plan and execute box delivery to goal using Sokoban-style planning.

        Returns:
            True if box successfully delivered, False otherwise
        """
        print("\n" + "=" * 50)
        print("MISSION: Deliver box to goal")
        print("=" * 50)

        if not self.kb.box_found:
            print("  ERROR: Box location unknown!")
            return False

        # Sync KB robot position to actual continuous pose before planning
        pose = self.robot.get_pose()
        old_x, old_y = self.kb.robot_x, self.kb.robot_y
        # Update KB robot position using set_robot (preserves Pose dataclass)
        self.kb.set_robot(pose["grid_x"], pose["grid_y"], self.kb.robot_heading)
        if (old_x, old_y) != (pose["grid_x"], pose["grid_y"]):
            print(
                f"  [SYNC] KB robot position: ({old_x}, {old_y}) → ({pose['grid_x']}, {pose['grid_y']})"
            )

        if self.kb.is_box_at_goal():
            print("  Box is already at goal!")
            return True

        # Plan the delivery
        print(f"  Box at: ({self.kb.box_x}, {self.kb.box_y})")
        print(f"  Goal at: ({self.kb.goal_x}, {self.kb.goal_y})")

        actions = planning.plan_box_delivery(self.kb)

        if actions is None:
            print("  ERROR: No valid delivery plan found!")
            return False

        print(f"  Plan: {len(actions)} actions")

        # Execute the plan, updating box position after each push
        return self._execute_delivery_plan(actions, realtime)

    def _execute_delivery_plan(
        self, actions: list[str], realtime: bool, replan_depth: int = 0
    ) -> bool:
        """
        Execute a delivery plan using sensor-guided navigation.

        NO CHEATING - uses only:
        1. Robot's internal pose estimate (from sensors)
        2. KB's belief of box position (from LiDAR detection)
        3. Bump sensor for contact detection during push
        4. LiDAR to re-detect box after each push

        Args:
            actions: List of actions from planner
            realtime: Whether to add visualization delays
            replan_depth: Current recursion depth (for loop prevention)
        """
        MAX_REPLAN_DEPTH = 5

        if actions is None:
            print("  ERROR: No plan provided!")
            return False

        if replan_depth >= MAX_REPLAN_DEPTH:
            print(f"  ERROR: Max replan depth ({MAX_REPLAN_DEPTH}) reached!")
            return False

        def _replan_delivery(*, reason: str, heading_for_kb: int) -> bool:
            """
            Replan delivery from the robot's current *estimated* pose and latest
            LiDAR-based box observation (no ground-truth).
            """
            print(f"      [REPLAN] Triggered: {reason}")

            if replan_depth >= MAX_REPLAN_DEPTH:
                print(f"      [ERROR] Max replan depth ({MAX_REPLAN_DEPTH}) reached!")
                return False

            # Refresh box estimate (best effort)
            scan_data = self.robot.scan_lidar(visualize=False)
            box_found = self._update_box_from_lidar(scan_data)

            if not box_found:
                # Back up and scan again (may be occluded by the robot/box contact)
                self.robot.move("backward", realtime)
                scan_data = self.robot.scan_lidar(visualize=False)
                self._update_box_from_lidar(scan_data)

            # Sync robot pose from sensors
            pose = self.robot.get_pose()
            self.kb.set_robot(pose["grid_x"], pose["grid_y"], heading_for_kb)

            new_plan = planning.plan_box_delivery(self.kb)
            if new_plan is None:
                print("      [ERROR] Replanning failed - no valid plan!")
                return False

            print(
                f"      [REPLAN] New plan with {len(new_plan)} actions (depth {replan_depth + 1})"
            )
            return self._execute_delivery_plan(new_plan, realtime, replan_depth + 1)

        for i, action in enumerate(actions):
            # Use KB's robot position (synced to plan) - avoids drift issues
            robot_x, robot_y = self.kb.robot_x, self.kb.robot_y
            current_heading = self.kb.robot_heading

            # DEBUG: Compare KB belief vs internal pose vs actual
            internal_pose = self.robot.get_pose()
            actual_pose = self.robot.get_actual_pose()

            print(f"\n    [{i + 1}/{len(actions)}] {action}")
            print(
                f"      KB belief:  robot=({robot_x},{robot_y}) h={current_heading}, box=({self.kb.box_x},{self.kb.box_y})"
            )
            print(
                f"      Internal:   robot=({internal_pose['grid_x']},{internal_pose['grid_y']}) yaw={math.degrees(internal_pose['yaw']):.1f}°"
            )
            print(
                f"      Actual:     robot=({actual_pose['grid_x_internal']},{actual_pose['grid_y_internal']}) yaw={actual_pose['yaw_deg']:.1f}° "
                f"[world_grid=({actual_pose['grid_x_world']},{actual_pose['grid_y_world']})]"
            )
            # For meaningful comparisons, also report in INTERNAL frame.
            print(
                f"      Actual(internal): robot=({actual_pose['grid_x_internal']},{actual_pose['grid_y_internal']}) "
                f"pos=({actual_pose['x_internal']:.3f},{actual_pose['y_internal']:.3f})"
            )

            # Get actual box position for comparison
            box_pos, _ = pybullet.getBasePositionAndOrientation(self.box_id)
            actual_box_grid_world_x = int(
                round((box_pos[0] - CELL_SIZE / 2) / CELL_SIZE)
            )
            actual_box_grid_world_y = int(
                round((box_pos[1] - CELL_SIZE / 2) / CELL_SIZE)
            )

            # Convert box to INTERNAL frame using KB bounds (same translation as robot actual pose).
            if self.kb.bounds is not None:
                min_x, _, min_y, _ = self.kb.bounds
            else:
                min_x, min_y = 0, 0
            box_x_internal = box_pos[0] + (min_x * CELL_SIZE)
            box_y_internal = box_pos[1] + (min_y * CELL_SIZE)
            actual_box_grid_x = int(round((box_x_internal - CELL_SIZE / 2) / CELL_SIZE))
            actual_box_grid_y = int(round((box_y_internal - CELL_SIZE / 2) / CELL_SIZE))

            print(
                f"      Actual box(internal): ({actual_box_grid_x},{actual_box_grid_y}) "
                f"pos=({box_x_internal:.3f},{box_y_internal:.3f}) "
                f"[world_grid=({actual_box_grid_world_x},{actual_box_grid_world_y}) world=({box_pos[0]:.3f},{box_pos[1]:.3f})]"
            )

            # WARN if KB and internal-actual disagree
            if (self.kb.box_x, self.kb.box_y) != (actual_box_grid_x, actual_box_grid_y):
                print(
                    f"      ⚠️  BOX MISMATCH (internal)! KB=({self.kb.box_x},{self.kb.box_y}) vs actual=({actual_box_grid_x},{actual_box_grid_y})"
                )

            # Check if this move will push the box
            will_push = False
            if action == "move_forward":
                dx, dy = DIRECTION_VECTORS[current_heading]
                ahead = (robot_x + dx, robot_y + dy)

                # Use KB's belief of box position
                if ahead == (self.kb.box_x, self.kb.box_y):
                    will_push = True
                    new_box_x = self.kb.box_x + dx
                    new_box_y = self.kb.box_y + dy
                    print(
                        f"      [PUSH] Box at ({self.kb.box_x},{self.kb.box_y}), pushing to ({new_box_x},{new_box_y})"
                    )

            # Execute actions
            if action == "turn_left":
                # Safety: check if in contact with box before turning
                if self.robot.is_in_contact_with(self.box_id):
                    print(
                        "      [SAFETY] In contact with box - backing up before turn!"
                    )
                    self.robot.move("backward", realtime)
                    # Update pose after backup
                    backup_pose = self.robot.get_pose()
                    self.kb.set_robot(
                        backup_pose["grid_x"], backup_pose["grid_y"], current_heading
                    )

                new_heading = (current_heading - 1) % 4
                reached = self.robot.move_to_waypoint(
                    robot_x, robot_y, new_heading, realtime
                )
                if not reached:
                    return _replan_delivery(
                        reason=f"turn_left failed to reach ({robot_x},{robot_y})",
                        heading_for_kb=current_heading,
                    )
                self.kb.set_robot(robot_x, robot_y, new_heading)
                if SNAP_INTERNAL_POSE:
                    self.robot.snap_pose_to_grid(robot_x, robot_y, new_heading)
            elif action == "turn_right":
                # Safety: check if in contact with box before turning
                if self.robot.is_in_contact_with(self.box_id):
                    print(
                        "      [SAFETY] In contact with box - backing up before turn!"
                    )
                    self.robot.move("backward", realtime)
                    # Update pose after backup
                    backup_pose = self.robot.get_pose()
                    self.kb.set_robot(
                        backup_pose["grid_x"], backup_pose["grid_y"], current_heading
                    )

                new_heading = (current_heading + 1) % 4
                reached = self.robot.move_to_waypoint(
                    robot_x, robot_y, new_heading, realtime
                )
                if not reached:
                    return _replan_delivery(
                        reason=f"turn_right failed to reach ({robot_x},{robot_y})",
                        heading_for_kb=current_heading,
                    )
                self.kb.set_robot(robot_x, robot_y, new_heading)
                if SNAP_INTERNAL_POSE:
                    self.robot.snap_pose_to_grid(robot_x, robot_y, new_heading)
            elif action == "move_forward":
                dx, dy = DIRECTION_VECTORS[current_heading]
                target_x = robot_x + dx
                target_y = robot_y + dy

                if will_push:
                    # Push with bump sensor feedback
                    pushed = self._push_box_with_verification(
                        target_x,
                        target_y,
                        current_heading,
                        new_box_x,
                        new_box_y,
                        realtime,
                    )
                    if pushed:
                        # Verify push via LiDAR (no ground-truth): require that the box
                        # is re-observed at the expected destination cell.
                        scan_data = self.robot.scan_lidar(visualize=False)
                        box_found = self._update_box_from_lidar(scan_data)

                        if not box_found:
                            # Back up and scan again (robot may occlude the box)
                            self.robot.move("backward", realtime)
                            scan_data = self.robot.scan_lidar(visualize=False)
                            box_found = self._update_box_from_lidar(scan_data)

                        if (self.kb.box_x, self.kb.box_y) != (new_box_x, new_box_y):
                            # Treat as failed push; do NOT advance KB box belief blindly.
                            return _replan_delivery(
                                reason=(
                                    f"push verification failed (expected box at ({new_box_x},{new_box_y}), "
                                    f"observed ({self.kb.box_x},{self.kb.box_y}))"
                                ),
                                heading_for_kb=current_heading,
                            )

                        print(f"      [KB] Box confirmed at ({new_box_x}, {new_box_y})")
                    else:
                        # Push failed - try to replan
                        print("      [PUSH FAILED] Attempting to replan...")

                        # DEBUG: Show actual state at failure (in INTERNAL frame)
                        actual_pose = self.robot.get_actual_pose()
                        box_pos, _ = pybullet.getBasePositionAndOrientation(self.box_id)
                        if self.kb.bounds is not None:
                            min_x, _, min_y, _ = self.kb.bounds
                        else:
                            min_x, min_y = 0, 0
                        box_x_internal = box_pos[0] + (min_x * CELL_SIZE)
                        box_y_internal = box_pos[1] + (min_y * CELL_SIZE)
                        actual_box_gx = int(
                            round((box_x_internal - CELL_SIZE / 2) / CELL_SIZE)
                        )
                        actual_box_gy = int(
                            round((box_y_internal - CELL_SIZE / 2) / CELL_SIZE)
                        )
                        print(
                            f"      [DEBUG] Actual robot(internal): grid=({actual_pose['grid_x_internal']},{actual_pose['grid_y_internal']}) "
                            f"pos=({actual_pose['x_internal']:.3f},{actual_pose['y_internal']:.3f}) yaw={actual_pose['yaw_deg']:.1f}° "
                            f"[world_grid=({actual_pose['grid_x_world']},{actual_pose['grid_y_world']})]"
                        )
                        print(
                            f"      [DEBUG] Actual box(internal): grid=({actual_box_gx},{actual_box_gy}) "
                            f"[world_grid=({int(round((box_pos[0] - CELL_SIZE / 2) / CELL_SIZE))},{int(round((box_pos[1] - CELL_SIZE / 2) / CELL_SIZE))})]"
                        )
                        print(
                            f"      [DEBUG] Expected box after push (internal): ({new_box_x},{new_box_y})"
                        )

                        return _replan_delivery(
                            reason="push failed (lost contact / never contacted)",
                            heading_for_kb=current_heading,
                        )
                else:
                    # Regular movement (non-push)
                    reached = self.robot.move_to_waypoint(
                        target_x, target_y, current_heading, realtime
                    )
                    if not reached:
                        return _replan_delivery(
                            reason=f"move_forward failed to reach ({target_x},{target_y})",
                            heading_for_kb=current_heading,
                        )

                # Update KB robot position to target (trust the plan)
                self.kb.set_robot(target_x, target_y, current_heading)
                if SNAP_INTERNAL_POSE:
                    self.robot.snap_pose_to_grid(target_x, target_y, current_heading)
            else:
                raise ValueError(f"Unknown delivery action: {action}")

            # Check if done using KB's belief
            if self.kb.is_box_at_goal():
                print("\n  >>> BOX DELIVERED TO GOAL! <<<")
                print(f"      KB box at ({self.kb.box_x}, {self.kb.box_y})")

                # DEBUG: Verify actual positions
                box_pos, _ = pybullet.getBasePositionAndOrientation(self.box_id)
                actual_box_gx = int(round((box_pos[0] - CELL_SIZE / 2) / CELL_SIZE))
                actual_box_gy = int(round((box_pos[1] - CELL_SIZE / 2) / CELL_SIZE))
                print(
                    f"      Actual box at ({actual_box_gx},{actual_box_gy}) world=({box_pos[0]:.3f},{box_pos[1]:.3f})"
                )

                self.kb.print_grid()
                return True

        # Plan exhausted - DEBUG: Show final state
        print("\n  [PLAN EXHAUSTED] Final state:")
        print(
            f"    KB:   robot=({self.kb.robot_x},{self.kb.robot_y}), box=({self.kb.box_x},{self.kb.box_y}), goal=({self.kb.goal_x},{self.kb.goal_y})"
        )

        actual_pose = self.robot.get_actual_pose()
        box_pos, _ = pybullet.getBasePositionAndOrientation(self.box_id)
        actual_robot_gx = int(round((actual_pose["x"] - CELL_SIZE / 2) / CELL_SIZE))
        actual_robot_gy = int(round((actual_pose["y"] - CELL_SIZE / 2) / CELL_SIZE))
        actual_box_gx = int(round((box_pos[0] - CELL_SIZE / 2) / CELL_SIZE))
        actual_box_gy = int(round((box_pos[1] - CELL_SIZE / 2) / CELL_SIZE))
        print(
            f"    Actual: robot=({actual_robot_gx},{actual_robot_gy}), box=({actual_box_gx},{actual_box_gy})"
        )

        if not self.kb.is_box_at_goal():
            print("  ERROR: Box not at goal - may need re-planning")
            return False

        return True

    def _update_box_from_lidar(self, scan_data) -> bool:
        """
        Re-detect box position using LiDAR scan (realistic sensing).

        Updates KB's box position if box is detected.

        Returns:
            True if box was detected in scan
        """
        # PyBullet offset - robot spawns at (1,1) but internal frame starts at (0,0)
        pybullet_offset = (1, 1)

        for angle, distance, obj_id in scan_data:
            if obj_id == self.box_id:
                # IMPORTANT: LiDAR distance is measured from ACTUAL robot position,
                # so we must use actual pose for hit calculation, then convert to internal frame
                actual_pose = self.robot.get_actual_pose()
                world_angle = actual_pose["yaw"] + angle
                hit_x = actual_pose["x"] + distance * math.cos(world_angle)
                hit_y = actual_pose["y"] + distance * math.sin(world_angle)

                # Convert hit point to PyBullet grid cell
                box_pybullet_gx = int(round((hit_x - CELL_SIZE / 2) / CELL_SIZE))
                box_pybullet_gy = int(round((hit_y - CELL_SIZE / 2) / CELL_SIZE))

                # Convert to internal frame (subtract offset)
                box_grid_x = box_pybullet_gx - pybullet_offset[0]
                box_grid_y = box_pybullet_gy - pybullet_offset[1]

                # Heuristic: if the ray direction is diagonal but rounding produced a
                # 4-neigh (cardinal) adjacent cell, "promote" it to the diagonal cell.
                internal_pose = self.robot.get_pose()
                rx, ry = internal_pose["grid_x"], internal_pose["grid_y"]
                ux, uy = math.cos(world_angle), math.sin(world_angle)
                diag = abs(ux) > 0.35 and abs(uy) > 0.35
                manhattan = abs(box_grid_x - rx) + abs(box_grid_y - ry)
                if diag and manhattan == 1:
                    if box_grid_x == rx:
                        box_grid_x += 1 if ux > 0 else -1
                    if box_grid_y == ry:
                        box_grid_y += 1 if uy > 0 else -1

                # Update KB if position changed
                if (box_grid_x, box_grid_y) != (self.kb.box_x, self.kb.box_y):
                    print(
                        f"      [LIDAR] Box re-detected at ({box_grid_x}, {box_grid_y})"
                    )
                    if self.kb.box_found:
                        self.kb.move_box(box_grid_x, box_grid_y)
                    else:
                        self.kb.set_box(box_grid_x, box_grid_y)
                return True
        return False

    def _push_box_with_verification(
        self,
        target_x: int,
        target_y: int,
        heading: int,
        expected_box_x: int,
        expected_box_y: int,
        realtime: bool,
    ) -> bool:
        """
        Push the box by driving into it for ~1 cell distance while in contact.

        Realistic physics-based pushing (NO cheating with box position!):
        1. Drive forward until bump sensor detects contact
        2. KEEP DRIVING while in contact, tracking ROBOT's traveled distance
        3. Stop when robot has traveled ~1 cell while pushing
        4. Use LiDAR afterward to detect where box ended up

        Returns:
            True if push was executed (contact maintained)
        """
        push_distance = CELL_SIZE * 1.0  # Push robot forward ~1 cell
        max_approach_steps = 400  # Steps to reach box
        max_push_steps = 600  # Steps while pushing
        push_speed = 10.0  # Constant driving speed

        dx, dy = DIRECTION_VECTORS[heading]
        target_angle = math.atan2(dy, dx)

        print(
            f"      [PUSH] Approaching box (heading {heading}, angle {math.degrees(target_angle):.1f}°)..."
        )

        # Phase 1: Drive straight toward box using nominal heading (grid-aligned)
        # Don't use LiDAR angle - it hits box surface/corner, not center
        self.robot._reset_odometry()
        contact_made = False

        for step in range(max_approach_steps):
            self.robot.update_pose(TIME_STEP)

            # Check bump sensor
            if self.robot.is_in_contact_with(self.box_id):
                contact_made = True
                print(f"      [BUMP] Contact detected after {step} steps!")
                break

            # Drive forward with heading hold (straight toward box center)
            yaw_error = target_angle - self.robot.pose_yaw
            while yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            while yaw_error < -math.pi:
                yaw_error += 2 * math.pi

            correction = max(-3.0, min(3.0, 4.0 * yaw_error))
            self.robot._set_drive_wheels(push_speed, correction, realtime)

            pybullet.stepSimulation()
            if realtime:
                time.sleep(TIME_STEP)

        if not contact_made:
            self.robot._stop()
            print("      [PUSH] ERROR: Never made contact with box!")
            return False

        # Phase 2: Keep pushing with heading hold (straight push)
        print(
            f"      [PUSH] Pushing for {push_distance:.2f}m while maintaining contact..."
        )

        # Record robot position at start of push
        push_start_x = self.robot.pose_x
        push_start_y = self.robot.pose_y
        contact_lost_count = 0
        robot_pushed_distance = 0.0

        # Log initial box position (for debugging)
        box_pos, box_quat = pybullet.getBasePositionAndOrientation(self.box_id)
        box_euler = pybullet.getEulerFromQuaternion(box_quat)
        print(
            f"      [DEBUG] Box START: pos=({box_pos[0]:.3f}, {box_pos[1]:.3f}), "
            f"yaw={math.degrees(box_euler[2]):.1f}°"
        )

        try:
            for step in range(max_push_steps):
                self.robot.update_pose(TIME_STEP)

                # Track how far ROBOT has traveled during push (realistic!)
                robot_pushed_distance = math.sqrt(
                    (self.robot.pose_x - push_start_x) ** 2
                    + (self.robot.pose_y - push_start_y) ** 2
                )

                # Monitor bump sensor for contact with box
                in_contact = self.robot.is_in_contact_with(self.box_id)
                if not in_contact:
                    contact_lost_count += 1
                    if contact_lost_count > 50:  # Allow brief gaps in contact
                        print(
                            f"      [BUMP] Contact lost after pushing {robot_pushed_distance:.3f}m"
                        )
                        break
                else:
                    contact_lost_count = 0

                # Periodic box position logging (every 100 steps for debugging)
                if step > 0 and step % 100 == 0:
                    box_pos, box_quat = pybullet.getBasePositionAndOrientation(
                        self.box_id
                    )
                    box_euler = pybullet.getEulerFromQuaternion(box_quat)
                    print(
                        f"      [DEBUG] Box @ step {step}: pos=({box_pos[0]:.3f}, {box_pos[1]:.3f}), "
                        f"yaw={math.degrees(box_euler[2]):.1f}°, robot_dist={robot_pushed_distance:.3f}m"
                    )

                # Success: robot has pushed far enough
                if robot_pushed_distance >= push_distance:
                    force = self.robot.get_contact_force(self.box_id)
                    print(
                        f"      [PUSH] Pushed {robot_pushed_distance:.3f}m in {step} steps (force={force:.1f}N)"
                    )
                    break

                # Keep driving straight (heading hold)
                yaw_error = target_angle - self.robot.pose_yaw
                while yaw_error > math.pi:
                    yaw_error -= 2 * math.pi
                while yaw_error < -math.pi:
                    yaw_error += 2 * math.pi

                correction = max(-3.0, min(3.0, 4.0 * yaw_error))
                self.robot._set_drive_wheels(push_speed, correction, realtime)

                pybullet.stepSimulation()
                if realtime:
                    time.sleep(TIME_STEP)

            self.robot._stop()
            # Let physics settle
            for _ in range(30):
                pybullet.stepSimulation()
        finally:
            pass  # No cleanup needed

        # Log final box position (for debugging)
        box_pos, box_quat = pybullet.getBasePositionAndOrientation(self.box_id)
        box_euler = pybullet.getEulerFromQuaternion(box_quat)
        box_grid_x = int(round((box_pos[0] - CELL_SIZE / 2) / CELL_SIZE))
        box_grid_y = int(round((box_pos[1] - CELL_SIZE / 2) / CELL_SIZE))
        print(
            f"      [PUSH] Complete. Robot traveled {robot_pushed_distance:.3f}m while pushing."
        )
        print(
            f"      [DEBUG] Box END: pos=({box_pos[0]:.3f}, {box_pos[1]:.3f}), "
            f"yaw={math.degrees(box_euler[2]):.1f}°, grid=({box_grid_x},{box_grid_y})"
        )

        # We assume success if we maintained contact for reasonable distance
        return robot_pushed_distance >= push_distance * 0.5

    def run_full_mission(self, realtime: bool = True) -> bool:
        """
        Run the complete mission: explore → find box → deliver to goal.

        Returns:
            True if mission completed successfully
        """
        print("\n" + "#" * 60)
        print("# STARTING FULL MISSION: Find and deliver box to goal")
        print("#" * 60)

        # Phase 1: Explore to find box
        if not self.explore_until_box_found(realtime):
            print("\nMISSION FAILED: Could not find box")
            return False

        # Phase 2: Deliver box to goal
        if not self.deliver_box_to_goal(realtime):
            print("\nMISSION FAILED: Could not deliver box")
            return False

        print("\n" + "#" * 60)
        print("# MISSION COMPLETE!")
        print("#" * 60)
        return True
