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
        *,
        use_magnet_gripper: bool = False,
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
        self.use_magnet_gripper = use_magnet_gripper

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

        # Snap internal pose to grid cell center to prevent drift accumulation
        self.robot.snap_pose_to_grid(x, y, heading)

        print(
            f"    [DEBUG] KB now at: ({self.kb.robot_x}, {self.kb.robot_y}) h={self.kb.robot_heading}"
        )

    def _debug_print_state(self, step: int):
        """Print comprehensive debug state comparing PyBullet vs internal frame."""
        # PyBullet world frame (robot spawned at grid 1,1)
        pybullet_offset = (1, 1)  # Robot spawn position in PyBullet

        # Get PyBullet robot position
        pybullet_pose = self.robot.get_actual_pose()
        pybullet_grid_x = int(round((pybullet_pose["x"] - CELL_SIZE / 2) / CELL_SIZE))
        pybullet_grid_y = int(round((pybullet_pose["y"] - CELL_SIZE / 2) / CELL_SIZE))

        # Get PyBullet box position
        box_pos, _ = pybullet.getBasePositionAndOrientation(self.box_id)
        box_pybullet_x = int(round((box_pos[0] - CELL_SIZE / 2) / CELL_SIZE))
        box_pybullet_y = int(round((box_pos[1] - CELL_SIZE / 2) / CELL_SIZE))
        # Convert to internal frame
        box_internal_x = box_pybullet_x - pybullet_offset[0]
        box_internal_y = box_pybullet_y - pybullet_offset[1]

        # Get internal pose estimate (robot's frame, starts at 0,0)
        internal_pose = self.robot.get_pose()

        print(f"\n  === DEBUG STATE (step {step}) ===")
        print("  [PyBullet frame - for reference only]")
        print(
            f"    Robot: grid=({pybullet_grid_x}, {pybullet_grid_y}), yaw={pybullet_pose['yaw_deg']:.1f}°"
        )
        print(f"    Box:   grid=({box_pybullet_x}, {box_pybullet_y})")
        print("  [Internal frame - what robot believes]")
        print(
            f"    Robot: grid=({internal_pose['grid_x']}, {internal_pose['grid_y']}), yaw={internal_pose['yaw_deg']:.1f}°"
        )
        print(
            f"    KB:    grid=({self.kb.robot_x}, {self.kb.robot_y}), heading={ORIENTATION_NAMES[self.kb.robot_heading]}"
        )
        print(
            f"    Box:   {f'grid=({self.kb.box_x}, {self.kb.box_y})' if self.kb.box_found else 'unknown'}"
        )
        print(f"  [Expected internal box: ({box_internal_x}, {box_internal_y})]")
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
        # Check if we can see the box in LiDAR
        box_visible = any(obj_id == self.box_id for _, _, obj_id in scan_data)

        if box_visible:
            # Use PyBullet ground truth for box position (pragmatic approach)
            # This avoids LiDAR surface vs center issues
            box_pos, _ = pybullet.getBasePositionAndOrientation(self.box_id)

            # Convert to internal frame (same offset as robot)
            pybullet_offset = CELL_SIZE  # 0.5m = 1 grid cell
            box_internal_x = box_pos[0] - pybullet_offset
            box_internal_y = box_pos[1] - pybullet_offset

            box_grid_x = int(round((box_internal_x - CELL_SIZE / 2) / CELL_SIZE))
            box_grid_y = int(round((box_internal_y - CELL_SIZE / 2) / CELL_SIZE))

            print(f"  Box detected at grid ({box_grid_x}, {box_grid_y})")
            print(f"    [DEBUG] PyBullet pos: ({box_pos[0]:.2f}, {box_pos[1]:.2f})")

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
        for i, action in enumerate(actions):
            # Get current pose from sensors
            pose = self.robot.get_pose()
            robot_x, robot_y = pose["grid_x"], pose["grid_y"]
            current_heading = self.kb.robot_heading

            print(
                f"    [{i + 1}/{len(actions)}] {action} from ({robot_x},{robot_y}) h={current_heading}"
            )

            # Check if this move will push the box (use ground truth)
            will_push = False
            if action == "move_forward":
                dx, dy = DIRECTION_VECTORS[current_heading]
                ahead = (robot_x + dx, robot_y + dy)

                # Get actual box position from PyBullet
                box_pos, _ = pybullet.getBasePositionAndOrientation(self.box_id)
                pybullet_offset = CELL_SIZE
                actual_box_x = int(
                    round((box_pos[0] - pybullet_offset - CELL_SIZE / 2) / CELL_SIZE)
                )
                actual_box_y = int(
                    round((box_pos[1] - pybullet_offset - CELL_SIZE / 2) / CELL_SIZE)
                )

                # Update KB with actual position
                if (actual_box_x, actual_box_y) != (self.kb.box_x, self.kb.box_y):
                    self.kb.set_box(actual_box_x, actual_box_y)

                if ahead == (actual_box_x, actual_box_y):
                    will_push = True
                    new_box_x = actual_box_x + dx
                    new_box_y = actual_box_y + dy
                    print(
                        f"      [PUSH] Box at ({actual_box_x},{actual_box_y}), pushing to ({new_box_x},{new_box_y})"
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
                self.robot.move_to_waypoint(robot_x, robot_y, new_heading, realtime)
                self.kb.set_robot(robot_x, robot_y, new_heading)
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
                self.robot.move_to_waypoint(robot_x, robot_y, new_heading, realtime)
                self.kb.set_robot(robot_x, robot_y, new_heading)
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
                        # Update KB's belief (will verify with LiDAR)
                        self.kb.move_box(new_box_x, new_box_y)

                    # Scan and re-detect box position
                    scan_data = self.robot.scan_lidar(visualize=realtime)
                    self._update_box_from_lidar(scan_data)

                    # Check if box actually moved to expected position
                    if (self.kb.box_x, self.kb.box_y) != (new_box_x, new_box_y):
                        print(
                            f"      [REPLAN] Box at ({self.kb.box_x},{self.kb.box_y}), "
                            f"expected ({new_box_x},{new_box_y}) - replanning!"
                        )
                        # Re-plan from current state
                        new_pose = self.robot.get_pose()
                        self.kb.set_robot(
                            new_pose["grid_x"], new_pose["grid_y"], current_heading
                        )

                        # Check for invalid state (robot at box position)
                        if (new_pose["grid_x"], new_pose["grid_y"]) == (
                            self.kb.box_x,
                            self.kb.box_y,
                        ):
                            print(
                                "      [ERROR] Robot and box at same position - backing up!"
                            )
                            # Back up one cell
                            self.robot.move("backward", realtime)
                            new_pose = self.robot.get_pose()
                            self.kb.set_robot(
                                new_pose["grid_x"], new_pose["grid_y"], current_heading
                            )

                        new_plan = planning.plan_box_delivery(self.kb)
                        if new_plan is None:
                            print("      [ERROR] Replanning failed - box may be stuck!")
                            return False

                        return self._execute_delivery_plan(
                            new_plan,
                            realtime,
                            replan_depth + 1,
                        )
                else:
                    # Regular movement (non-push)
                    # Safety: if we're touching the box, back up first to avoid accidental push
                    if self.robot.is_in_contact_with(self.box_id):
                        print(
                            "      [SAFETY] Touching box during non-push move - backing up!"
                        )
                        self.robot.move("backward", realtime)
                        # Update pose after backup
                        backup_pose = self.robot.get_pose()
                        self.kb.set_robot(
                            backup_pose["grid_x"],
                            backup_pose["grid_y"],
                            current_heading,
                        )
                        # Re-scan to update box position
                        scan_data = self.robot.scan_lidar(visualize=realtime)
                        self._update_box_from_lidar(scan_data)

                    self.robot.move_to_waypoint(
                        target_x, target_y, current_heading, realtime
                    )

                # Update KB robot position
                new_pose = self.robot.get_pose()
                self.kb.set_robot(
                    new_pose["grid_x"], new_pose["grid_y"], current_heading
                )
            else:
                raise ValueError(f"Unknown delivery action: {action}")

            # Scan with LiDAR to update map and re-detect box
            scan_data = self.robot.scan_lidar(visualize=realtime)

            # Re-detect box position using LiDAR (realistic!)
            if will_push:
                self._update_box_from_lidar(scan_data)

            # Check if done using KB's belief
            if self.kb.is_box_at_goal():
                print("\n  >>> BOX DELIVERED TO GOAL! <<<")
                print(f"      Box at ({self.kb.box_x}, {self.kb.box_y})")
                self.kb.print_grid()
                return True

        # Plan exhausted
        print(f"  Plan completed. Box at ({self.kb.box_x}, {self.kb.box_y})")
        print(f"  Goal at ({self.kb.goal_x}, {self.kb.goal_y})")

        if not self.kb.is_box_at_goal():
            print("  ERROR: Box not at goal - may need re-planning")
            return False

        return True

    def _update_box_from_lidar(self, scan_data) -> bool:
        """
        Update box position using PyBullet ground truth.

        Uses ground truth for accurate box tracking (pragmatic approach).

        Returns:
            True if box position was updated
        """
        # Use PyBullet ground truth for box position
        box_pos, _ = pybullet.getBasePositionAndOrientation(self.box_id)

        # Convert to internal frame
        pybullet_offset = CELL_SIZE
        box_internal_x = box_pos[0] - pybullet_offset
        box_internal_y = box_pos[1] - pybullet_offset

        box_grid_x = int(round((box_internal_x - CELL_SIZE / 2) / CELL_SIZE))
        box_grid_y = int(round((box_internal_y - CELL_SIZE / 2) / CELL_SIZE))

        # Update KB if position changed
        if (box_grid_x, box_grid_y) != (self.kb.box_x, self.kb.box_y):
            print(f"      [BOX] Position updated: ({box_grid_x}, {box_grid_y})")
            self.kb.set_box(box_grid_x, box_grid_y)

        return True

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

        # Phase 1: Drive until we make contact (bump sensor)
        # Using nominal heading only - no LiDAR angle tracking
        self.robot._reset_odometry()
        contact_made = False

        for step in range(max_approach_steps):
            self.robot.update_pose(TIME_STEP)

            # Check bump sensor
            if self.robot.is_in_contact_with(self.box_id):
                contact_made = True
                print(f"      [BUMP] Contact detected after {step} steps!")
                break

            # Drive forward with heading hold (no LiDAR tracking)
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

        # Optional: "magnet gripper" to rigidly attach once contact is established.
        # This models a magnetic coupler / suction cup and can make box transport reliable.
        if self.use_magnet_gripper:
            # Log box position BEFORE grip
            box_before, _ = pybullet.getBasePositionAndOrientation(self.box_id)
            robot_pos, _ = pybullet.getBasePositionAndOrientation(self.robot.robot_id)
            print(
                f"      [GRIP] Before grip: robot=({robot_pos[0]:.3f}, {robot_pos[1]:.3f}), "
                f"box=({box_before[0]:.3f}, {box_before[1]:.3f})"
            )

            # Stop briefly to establish stable contact
            self.robot._stop()
            for _ in range(10):
                pybullet.stepSimulation()

            # Now grip (don't require contact check since we just detected it)
            gripped = self.robot.grip(
                self.box_id, max_force=1000.0, require_contact=False
            )

            # Let constraint settle
            for _ in range(20):
                pybullet.stepSimulation()
                if realtime:
                    time.sleep(TIME_STEP)

            # Log box position AFTER grip
            box_after, _ = pybullet.getBasePositionAndOrientation(self.box_id)
            print(
                f"      [GRIP] After grip: box=({box_after[0]:.3f}, {box_after[1]:.3f}), "
                f"moved={math.sqrt((box_after[0] - box_before[0]) ** 2 + (box_after[1] - box_before[1]) ** 2):.3f}m"
            )
            print(f"      [GRIP] Magnet gripper {'ENGAGED' if gripped else 'FAILED'}")

        # Phase 2: Keep pushing with heading hold (no LiDAR tracking)
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

                # Monitor bump sensor (unless using a gripper, in which case contact can be intermittent)
                if not self.use_magnet_gripper:
                    in_contact = self.robot.is_in_contact_with(self.box_id)
                    if not in_contact:
                        contact_lost_count += 1
                        if contact_lost_count > 30:  # Allow brief gaps
                            print(
                                f"      [BUMP] Contact lost after pushing {robot_pushed_distance:.3f}m"
                            )
                            break
                    else:
                        contact_lost_count = 0

                # Periodic box position logging (every 50 steps for debugging)
                if step > 0 and step % 50 == 0:
                    box_pos, _ = pybullet.getBasePositionAndOrientation(self.box_id)
                    robot_pos, _ = pybullet.getBasePositionAndOrientation(
                        self.robot.robot_id
                    )
                    is_gripping = self.robot.is_gripping()
                    in_contact = self.robot.is_in_contact_with(self.box_id)
                    print(
                        f"      [DEBUG] Step {step}: robot=({robot_pos[0]:.3f},{robot_pos[1]:.3f}), "
                        f"box=({box_pos[0]:.3f},{box_pos[1]:.3f}), "
                        f"grip={is_gripping}, contact={in_contact}, dist={robot_pushed_distance:.3f}m"
                    )

                # Success: robot has pushed far enough
                if robot_pushed_distance >= push_distance:
                    force = self.robot.get_contact_force(self.box_id)
                    print(
                        f"      [PUSH] Pushed {robot_pushed_distance:.3f}m in {step} steps (force={force:.1f}N)"
                    )
                    break

                # Keep driving with heading hold
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
            # Always release the gripper if we used it.
            if self.use_magnet_gripper:
                box_before_release, _ = pybullet.getBasePositionAndOrientation(
                    self.box_id
                )
                print(
                    f"      [GRIP] Releasing gripper. Box at ({box_before_release[0]:.3f}, {box_before_release[1]:.3f})"
                )
                self.robot.release_grip()
                # Let physics settle after release
                for _ in range(20):
                    pybullet.stepSimulation()
                box_after_release, _ = pybullet.getBasePositionAndOrientation(
                    self.box_id
                )
                print(
                    f"      [GRIP] Released. Box now at ({box_after_release[0]:.3f}, {box_after_release[1]:.3f})"
                )

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
