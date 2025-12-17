"""
Robot Control - Low-level robot control for PyBullet simulation.

This module handles:
- Wheel velocity control
- Movement primitives (move N cells, turn 90°)
- Sensor reading (LiDAR, odometry, gyroscope)

KB updates are handled by the mission controller, not here.
Environment setup is in simulation.py.
"""

import math
import time

import pybullet

import kb as kb_store
from mapping import integrate_lidar
from simulation import TIME_STEP, CELL_SIZE

# =============================================================================
# CONSTANTS
# =============================================================================

# Robot joint indices (match `PybulletRobotics/urdf/simple_two_wheel_car.urdf`)
# Driven wheels:
LEFT_WHEEL_JOINT = 0
RIGHT_WHEEL_JOINT = 1

# Robot physical parameters
WHEEL_RADIUS = 0.05  # 5cm (matches simple_two_wheel_car.urdf)

# Orientations
NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3
ORIENTATION_NAMES = {NORTH: "N", EAST: "E", SOUTH: "S", WEST: "W"}
ORIENTATION_ANGLES = {NORTH: math.pi / 2, EAST: 0, SOUTH: -math.pi / 2, WEST: math.pi}
DIRECTION_VECTORS = {NORTH: (0, 1), EAST: (1, 0), SOUTH: (0, -1), WEST: (-1, 0)}

# LiDAR parameters
LIDAR_LINK_IDX = 5
LIDAR_RANGE = 2.0
LIDAR_NUM_RAYS = 72


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


# =============================================================================
# LIDAR SENSOR
# =============================================================================


class Lidar:
    """LiDAR sensor using PyBullet ray casting."""

    def __init__(self, robot_id, num_rays=LIDAR_NUM_RAYS, max_range=LIDAR_RANGE):
        self.robot_id = robot_id
        self.num_rays = num_rays
        self.max_range = max_range
        self.ray_ids = None
        self.ray_start_offset = (
            0.22  # Just past robot edge (0.2m radius + 0.02m clearance)
        )

    def _get_lidar_pose(self):
        link_state = pybullet.getLinkState(self.robot_id, LIDAR_LINK_IDX)
        position = link_state[0]
        orientation = link_state[1]
        euler = pybullet.getEulerFromQuaternion(orientation)
        return position, euler[2]

    def scan(self, visualize=True):
        """Perform 360° scan. Returns list of (angle, distance, object_id)."""
        position, yaw = self._get_lidar_pose()

        ray_from, ray_to, angles = [], [], []
        for i in range(self.num_rays):
            angle = (2 * math.pi * i) / self.num_rays
            world_angle = yaw + angle
            angles.append(angle)

            start = [
                position[0] + self.ray_start_offset * math.cos(world_angle),
                position[1] + self.ray_start_offset * math.sin(world_angle),
                position[2],
            ]
            ray_from.append(start)

            end = [
                position[0]
                + (self.ray_start_offset + self.max_range) * math.cos(world_angle),
                position[1]
                + (self.ray_start_offset + self.max_range) * math.sin(world_angle),
                position[2],
            ]
            ray_to.append(end)

        results = pybullet.rayTestBatch(ray_from, ray_to)

        scan_data = []
        for i, result in enumerate(results):
            hit_id = result[0]
            hit_fraction = result[2]
            distance = hit_fraction * self.max_range if hit_id != -1 else self.max_range
            scan_data.append((angles[i], distance, hit_id))

        if visualize:
            self._draw_rays(ray_from, ray_to, results)

        return scan_data

    def _draw_rays(self, ray_from, ray_to, results):
        """Draw debug visualization lines."""
        z_offset = 0.1

        if self.ray_ids is None:
            self.ray_ids = []
            for i, result in enumerate(results):
                hit_id, hit_pos = result[0], result[3]
                start = [ray_from[i][0], ray_from[i][1], ray_from[i][2] + z_offset]

                if hit_id != -1:
                    end = [hit_pos[0], hit_pos[1], hit_pos[2] + z_offset]
                    color = [1, 0, 0]
                else:
                    end = [ray_to[i][0], ray_to[i][1], ray_to[i][2] + z_offset]
                    color = [0, 1, 0]

                self.ray_ids.append(
                    pybullet.addUserDebugLine(start, end, color, lineWidth=2)
                )
        else:
            for i, result in enumerate(results):
                hit_id, hit_pos = result[0], result[3]
                start = [ray_from[i][0], ray_from[i][1], ray_from[i][2] + z_offset]

                if hit_id != -1:
                    end = [hit_pos[0], hit_pos[1], hit_pos[2] + z_offset]
                    color = [1, 0, 0]
                else:
                    end = [ray_to[i][0], ray_to[i][1], ray_to[i][2] + z_offset]
                    color = [0, 1, 0]

                pybullet.addUserDebugLine(
                    start, end, color, lineWidth=2, replaceItemUniqueId=self.ray_ids[i]
                )

    def get_obstacle_in_front(self, scan_data, front_angle_range=math.pi / 4):
        """Check for obstacle in front. Returns (distance, object_id) or (None, None)."""
        min_dist = self.max_range
        closest_id = -1

        for angle, distance, obj_id in scan_data:
            norm_angle = angle if angle <= math.pi else angle - 2 * math.pi
            if abs(norm_angle) <= front_angle_range and distance < min_dist:
                min_dist = distance
                closest_id = obj_id

        return (min_dist, closest_id) if closest_id != -1 else (None, None)


# =============================================================================
# ROBOT CONTROLLER
# =============================================================================


class RobotController:
    """
    Low-level robot control with proportional control.

    Does NOT update KB - that's the mission controller's job.
    """

    # Control parameters
    DISTANCE_TOLERANCE = CELL_SIZE * 0.01
    ANGLE_TOLERANCE = math.radians(1.5)  # Tighter for compact robot
    MAX_MOVE_SPEED = 10  # Reduced for 4-wheel stability
    MIN_MOVE_SPEED = 2
    MAX_TURN_SPEED = 5  # Slightly increased for compact robot
    MIN_TURN_SPEED = 1.5  # Higher minimum for better control
    HEADING_KP = 6.0
    MAX_HEADING_CORRECTION = 4.0
    TURN_SNAP_KP = 6.0  # Increased gain for faster convergence
    TURN_SNAP_MAX_STEPS = 800  # More steps for precise turns

    # Pose estimation parameters
    SLIP_THRESHOLD = 0.02  # 2cm difference triggers slip detection
    WALL_RECALIB_THRESHOLD = 0.1  # Recalibrate if drift > 10cm

    def __init__(
        self,
        robot_id,
        kb: kb_store.KnowledgeBase,
        wall_ids=None,
    ):
        self.robot_id = robot_id
        self.kb = kb
        self.wall_ids = set(wall_ids) if wall_ids else None

        # Wheel odometry tracking (driven wheels)
        self.last_left_pos = self._get_wheel_position(LEFT_WHEEL_JOINT)
        self.last_right_pos = self._get_wheel_position(RIGHT_WHEEL_JOINT)

        # Internal coordinate frame:
        # - Grid cell (0,0) center is at (CELL_SIZE/2, CELL_SIZE/2)
        # - This matches `move_to_waypoint()` targets and `snap_pose_to_grid()`
        self.pose_x = CELL_SIZE / 2
        self.pose_y = CELL_SIZE / 2
        self.pose_yaw = 0.0

        # Discovered wall positions (landmarks for recalibration)
        self._wall_landmarks = {}

        self.lidar = Lidar(robot_id)
        self._ccw_command_increases_yaw = None
        # Optional "magnet gripper" (implemented as a temporary fixed constraint).
        # Disabled unless explicitly used by the mission/controller.
        self._grip_constraint_id: int | None = None

    # --- Optional Gripper / Magnet (Constraint-Based Attachment) ---

    def is_gripping(self) -> bool:
        return self._grip_constraint_id is not None

    def release_grip(self) -> None:
        """Release the magnet/gripper constraint if active."""
        if self._grip_constraint_id is None:
            return
        try:
            pybullet.removeConstraint(self._grip_constraint_id)
        finally:
            self._grip_constraint_id = None

    def grip(
        self,
        body_id: int,
        *,
        max_force: float = 500.0,
        require_contact: bool = True,
    ) -> bool:
        """
        Attach to a body using a fixed constraint (simulated magnet/gripper).

        Creates a "tow hook" style attachment at the front of the robot.

        Args:
            body_id: PyBullet body to attach to (e.g. the box)
            max_force: constraint max force
            require_contact: if True, only grip when currently in contact

        Returns:
            True if grip is active after the call.
        """
        if self._grip_constraint_id is not None:
            return True

        # Check contact if required
        if require_contact:
            contacts = pybullet.getContactPoints(bodyA=self.robot_id, bodyB=body_id)
            if not contacts:
                return False

        # Get current positions
        robot_pos, robot_orn = pybullet.getBasePositionAndOrientation(self.robot_id)
        box_pos, box_orn = pybullet.getBasePositionAndOrientation(body_id)

        # Calculate attachment point: front of robot (in robot's local frame)
        # Robot radius is ~0.2m, attach slightly in front
        robot_attach_local = (0.22, 0, 0.05)  # Front of robot, slightly up

        # For box: attach at its center (0,0,0 in local frame)
        box_attach_local = (0, 0, 0)

        print(
            f"      [GRIP DEBUG] Robot at {robot_pos[:2]}, Box at {box_pos[:2]}, "
            f"dist={math.sqrt((robot_pos[0] - box_pos[0]) ** 2 + (robot_pos[1] - box_pos[1]) ** 2):.3f}m"
        )

        cid = pybullet.createConstraint(
            parentBodyUniqueId=self.robot_id,
            parentLinkIndex=-1,
            childBodyUniqueId=body_id,
            childLinkIndex=-1,
            jointType=pybullet.JOINT_FIXED,
            jointAxis=(0, 0, 0),
            parentFramePosition=robot_attach_local,
            childFramePosition=box_attach_local,
        )
        pybullet.changeConstraint(cid, maxForce=max_force)
        self._grip_constraint_id = cid
        print(f"      [GRIP DEBUG] Constraint {cid} created with max_force={max_force}")
        return True

    def get_actual_pose(self) -> dict:
        """Get actual pose from PyBullet (debug utility; avoid for estimation/control)."""
        pos, orn = pybullet.getBasePositionAndOrientation(self.robot_id)
        euler = pybullet.getEulerFromQuaternion(orn)
        return {
            "x": pos[0],
            "y": pos[1],
            "yaw": euler[2],
            "yaw_deg": math.degrees(euler[2]),
        }

    def scan_lidar(self, visualize=True):
        """Scan and update KB occupancy."""
        scan_data = self.lidar.scan(visualize=visualize)

        # Debug: show scan stats
        hits = sum(1 for _, _, oid in scan_data if oid != -1)
        print(f"  [DEBUG] LiDAR scan: {len(scan_data)} rays, {hits} hits")

        integrate_lidar(
            self.kb,
            scan_data,
            cell_size=CELL_SIZE,
            max_range=self.lidar.max_range,
            is_obstacle=(lambda oid: oid in self.wall_ids) if self.wall_ids else None,
        )

        return scan_data

    # --- Bump Sensor (Physical Contact Detection) ---

    def is_in_contact_with(self, body_id: int) -> bool:
        """
        Check if robot is currently in physical contact with a specific body.

        This simulates a real bump sensor / contact switch that detects
        physical collision. Real robots have these!

        Args:
            body_id: PyBullet body ID to check contact with

        Returns:
            True if robot is touching the specified body
        """
        contacts = pybullet.getContactPoints(bodyA=self.robot_id, bodyB=body_id)
        return len(contacts) > 0

    def get_contact_force(self, body_id: int) -> float:
        """
        Get the total normal force being applied (force/torque sensor).

        Real robots have force sensors to measure pushing force.

        Args:
            body_id: PyBullet body ID to check contact with

        Returns:
            Total normal force in Newtons (0 if no contact)
        """
        contacts = pybullet.getContactPoints(bodyA=self.robot_id, bodyB=body_id)
        total_force = 0.0
        for contact in contacts:
            # contact[9] is the normal force
            total_force += contact[9]
        return total_force

    # --- Low-level wheel control ---

    def _get_wheel_position(self, joint):
        return pybullet.getJointState(self.robot_id, joint)[0]

    def _set_wheel_velocities(self, left, right):
        pybullet.setJointMotorControl2(
            self.robot_id,
            LEFT_WHEEL_JOINT,
            pybullet.VELOCITY_CONTROL,
            targetVelocity=left,
            force=200,
        )
        pybullet.setJointMotorControl2(
            self.robot_id,
            RIGHT_WHEEL_JOINT,
            pybullet.VELOCITY_CONTROL,
            targetVelocity=right,
            force=200,
        )

    def _stop(self):
        self._set_wheel_velocities(0, 0)

    def _wait_and_step(self, steps, realtime=False):
        for _ in range(steps):
            pybullet.stepSimulation()
            if realtime:
                time.sleep(TIME_STEP)

    def _read_gyroscope(self):
        _, angular_vel = pybullet.getBaseVelocity(self.robot_id)
        return angular_vel[2]

    def _get_distance_traveled(self):
        left = self._get_wheel_position(LEFT_WHEEL_JOINT)
        right = self._get_wheel_position(RIGHT_WHEEL_JOINT)
        left_dist = (left - self.last_left_pos) * WHEEL_RADIUS
        right_dist = (right - self.last_right_pos) * WHEEL_RADIUS
        return (left_dist + right_dist) / 2

    def _reset_odometry(self):
        self.last_left_pos = self._get_wheel_position(LEFT_WHEEL_JOINT)
        self.last_right_pos = self._get_wheel_position(RIGHT_WHEEL_JOINT)

    def _get_base_velocity(self) -> tuple[float, float]:
        """Get linear speed and angular velocity from physics."""
        linear_vel, angular_vel = pybullet.getBaseVelocity(self.robot_id)
        speed = math.sqrt(linear_vel[0] ** 2 + linear_vel[1] ** 2)
        return speed, angular_vel[2]

    # --- Pose Estimation (sensor fusion) ---

    def update_pose(self, dt: float = None, scan_data: list = None):
        """
        Update internal pose estimate from PyBullet ground truth.

        SIMPLIFIED: We use PyBullet's actual position to avoid drift issues.
        The offset converts from PyBullet world frame to internal frame
        (robot starts at internal (0,0) but PyBullet (1,1) grid).

        Args:
            dt: Unused (kept for API compatibility)
            scan_data: Unused (kept for API compatibility)
        """
        # Get actual position from PyBullet
        pos, orn = pybullet.getBasePositionAndOrientation(self.robot_id)
        euler = pybullet.getEulerFromQuaternion(orn)

        # Convert from PyBullet world frame to internal frame
        # Robot spawns at PyBullet grid (1,1) = internal grid (0,0)
        # Offset is 1 cell = 0.5 meters
        pybullet_offset = CELL_SIZE  # 0.5m = 1 grid cell

        self.pose_x = pos[0] - pybullet_offset
        self.pose_y = pos[1] - pybullet_offset
        self.pose_yaw = euler[2]

    def _get_wall_distances(self, scan_data: list) -> dict:
        """
        Extract wall distances in cardinal directions from LiDAR scan.

        Returns dict with 'front', 'back', 'left', 'right' distances.
        Only includes distances where a wall was detected.
        """
        if not scan_data:
            return {}

        # Find rays closest to cardinal directions (relative to robot)
        # angle=0 is forward, π/2 is left, π is back, -π/2 is right
        cardinal = {
            "front": 0,
            "left": math.pi / 2,
            "back": math.pi,
            "right": -math.pi / 2,
        }
        result = {}

        for direction, target_angle in cardinal.items():
            best_dist = None
            best_diff = float("inf")

            for angle, distance, obj_id in scan_data:
                # Only count walls
                if self.wall_ids and obj_id not in self.wall_ids:
                    continue

                # Normalize angle difference
                diff = abs(normalize_angle(angle - target_angle))
                if diff < math.pi / 8 and diff < best_diff:  # Within 22.5° of cardinal
                    best_diff = diff
                    best_dist = distance

            if best_dist is not None:
                result[direction] = best_dist

        return result

    def _recalibrate_from_lidar(self, scan_data: list):
        """
        Correct position estimate using discovered wall landmarks.

        First time seeing a wall: record its position as a landmark.
        Later: compare current measurement to expected, correct drift.
        """
        walls = self._get_wall_distances(scan_data)
        if not walls:
            return

        # Check each cardinal direction we can see
        for direction, distance in walls.items():
            # Calculate wall angle in internal frame
            if direction == "front":
                wall_angle = self.pose_yaw
            elif direction == "back":
                wall_angle = self.pose_yaw + math.pi
            elif direction == "left":
                wall_angle = self.pose_yaw + math.pi / 2
            elif direction == "right":
                wall_angle = self.pose_yaw - math.pi / 2
            else:
                continue

            # Wall position in internal frame
            wall_x = self.pose_x + distance * math.cos(wall_angle)
            wall_y = self.pose_y + distance * math.sin(wall_angle)

            # Determine which axis this wall constrains
            wall_angle_norm = normalize_angle(wall_angle)
            wall_angle_deg = math.degrees(wall_angle_norm)

            # Wall roughly perpendicular to X axis (East or West facing)
            if abs(wall_angle_deg) < 30 or abs(wall_angle_deg) > 150:
                axis = "x"
                wall_coord = wall_x
            # Wall roughly perpendicular to Y axis (North or South facing)
            elif 60 < abs(wall_angle_deg) < 120:
                axis = "y"
                wall_coord = wall_y
            else:
                continue  # Diagonal, skip

            landmark_key = f"{direction}_{axis}"

            if landmark_key not in self._wall_landmarks:
                # First time seeing this wall - record as landmark
                self._wall_landmarks[landmark_key] = wall_coord
                print(
                    f"    [LANDMARK] Recorded {landmark_key} at {axis}={wall_coord:.3f}"
                )
            else:
                # Seen before - check for drift and correct
                expected = self._wall_landmarks[landmark_key]
                error = wall_coord - expected

                # Only correct if error is reasonable (not a different wall)
                if abs(error) < CELL_SIZE * 2:  # Within 2 cells
                    old_val = self.pose_x if axis == "x" else self.pose_y
                    if axis == "x":
                        self.pose_x -= error * 0.5  # Partial correction
                    else:
                        self.pose_y -= error * 0.5
                    new_val = self.pose_x if axis == "x" else self.pose_y
                    if abs(error) > 0.01:  # Only log significant corrections
                        print(
                            f"    [RECALIB] {landmark_key}: error={error:.3f}, {axis}: {old_val:.3f} → {new_val:.3f}"
                        )

    def get_pose(self) -> dict:
        """Get current pose estimate (internal frame, robot starts at 0,0)."""
        return {
            "x": self.pose_x,
            "y": self.pose_y,
            "yaw": self.pose_yaw,
            "yaw_deg": math.degrees(self.pose_yaw),
            # Convert from meters-at-cell-centers back to integer grid coordinates.
            "grid_x": int(round((self.pose_x - CELL_SIZE / 2) / CELL_SIZE)),
            "grid_y": int(round((self.pose_y - CELL_SIZE / 2) / CELL_SIZE)),
        }

    def snap_pose_to_grid(self, grid_x: int, grid_y: int, heading: int) -> None:
        """
        Force internal pose to match a specific grid cell center.

        Used after plan execution to prevent drift accumulation. The physical
        robot may have drifted slightly, but we trust the plan execution
        and reset our internal estimate to the expected position.

        Args:
            grid_x: Target grid cell X
            grid_y: Target grid cell Y
            heading: Target heading (NORTH/EAST/SOUTH/WEST)
        """
        self.pose_x = grid_x * CELL_SIZE + CELL_SIZE / 2
        self.pose_y = grid_y * CELL_SIZE + CELL_SIZE / 2
        self.pose_yaw = ORIENTATION_ANGLES[heading]

    def _calibrate_turn_direction_if_needed(self, realtime=False):
        if self._ccw_command_increases_yaw is not None:
            return

        # Determine whether the wheel command pattern (-, +) results in +yaw or -yaw.
        # Use the gyroscope (angular velocity), not absolute pose.
        integrated = 0.0
        self._set_wheel_velocities(-self.MIN_TURN_SPEED, self.MIN_TURN_SPEED)
        for _ in range(10):
            self._wait_and_step(1, realtime)
            integrated += self._read_gyroscope() * TIME_STEP
        self._stop()
        self._wait_and_step(5, realtime=False)

        self._ccw_command_increases_yaw = integrated > 0

    def _set_turn_wheels(self, ccw: bool, speed: float, realtime=False):
        self._calibrate_turn_direction_if_needed(realtime)
        if self._ccw_command_increases_yaw:
            left, right = (-speed, speed) if ccw else (speed, -speed)
        else:
            left, right = (speed, -speed) if ccw else (-speed, speed)
        self._set_wheel_velocities(left, right)

    def _set_drive_wheels(
        self, base_speed: float, ccw_correction: float, realtime=False
    ):
        self._calibrate_turn_direction_if_needed(realtime)
        c = ccw_correction
        if self._ccw_command_increases_yaw:
            left, right = base_speed - c, base_speed + c
        else:
            left, right = base_speed + c, base_speed - c
        self._set_wheel_velocities(left, right)

    def _snap_to_kb_yaw(self, realtime=False):
        """Correct yaw to match KB's cardinal direction."""
        target_yaw = ORIENTATION_ANGLES[self.kb.robot_heading]
        self._turn_to_angle(target_yaw, realtime)

    # --- High-level movement primitives ---

    def move_to_waypoint(
        self,
        target_grid_x: int,
        target_grid_y: int,
        target_heading: int = None,
        realtime: bool = False,
    ) -> bool:
        """
        Navigate to target grid cell using continuous pose estimation.

        Uses internal pose tracking (sensor fusion) to reach target,
        not discrete step counting.

        Args:
            target_grid_x: Target grid cell X
            target_grid_y: Target grid cell Y
            target_heading: Optional final heading (NORTH/EAST/SOUTH/WEST)
            realtime: Add visualization delays

        Returns:
            True if reached target successfully
        """
        # Target position in internal frame (grid cells centered at cell_size/2)
        target_x = target_grid_x * CELL_SIZE + CELL_SIZE / 2
        target_y = target_grid_y * CELL_SIZE + CELL_SIZE / 2

        self._reset_odometry()
        self._calibrate_turn_direction_if_needed(realtime)

        # Phase 1: Turn to face target (in internal frame)
        dx = target_x - self.pose_x
        dy = target_y - self.pose_y
        target_angle = math.atan2(dy, dx)

        # Turn to face target if not already aligned
        yaw_error = normalize_angle(target_angle - self.pose_yaw)
        if abs(yaw_error) > self.ANGLE_TOLERANCE:
            self._turn_to_angle(target_angle, realtime)

        # Phase 2: Drive to target
        reached = False
        for step in range(3000):
            # Update pose from sensors
            self.update_pose(TIME_STEP)

            # Calculate distance to target
            dx = target_x - self.pose_x
            dy = target_y - self.pose_y
            distance = math.sqrt(dx * dx + dy * dy)

            # Check if arrived
            if distance < CELL_SIZE * 0.15:  # Within 15% of cell size
                reached = True
                break

            # Recalculate angle to target (correct for drift)
            target_angle = math.atan2(dy, dx)
            yaw_error = normalize_angle(target_angle - self.pose_yaw)

            # Proportional speed control
            proportion = min(1.0, distance / CELL_SIZE)
            speed = self.MIN_MOVE_SPEED + proportion * (
                self.MAX_MOVE_SPEED - self.MIN_MOVE_SPEED
            )

            # Heading correction while driving
            correction = max(
                -self.MAX_HEADING_CORRECTION,
                min(self.MAX_HEADING_CORRECTION, self.HEADING_KP * yaw_error),
            )

            self._set_drive_wheels(speed, correction, realtime)
            self._wait_and_step(1, realtime)

        self._stop()
        self._wait_and_step(10, realtime=False)

        # Debug: show arrival accuracy
        final_dx = target_x - self.pose_x
        final_dy = target_y - self.pose_y
        final_dist = math.sqrt(final_dx * final_dx + final_dy * final_dy)
        print(
            f"    [NAV] Waypoint ({target_grid_x},{target_grid_y}): arrived={reached}, error={final_dist:.3f}m"
        )

        # Phase 3: Final heading correction (if specified)
        if target_heading is not None:
            final_yaw = ORIENTATION_ANGLES[target_heading]
            self._turn_to_angle(final_yaw, realtime)
            self.pose_yaw = final_yaw  # Sync internal pose

        return reached

    def _turn_to_angle(self, target_yaw: float, realtime: bool = False):
        """Turn to a specific yaw angle using gyroscope feedback."""
        self._calibrate_turn_direction_if_needed(realtime)

        for _ in range(self.TURN_SNAP_MAX_STEPS):
            self.update_pose(TIME_STEP)
            yaw_error = normalize_angle(target_yaw - self.pose_yaw)

            if abs(yaw_error) <= self.ANGLE_TOLERANCE:
                self._stop()
                # Snap internal estimate to commanded yaw (keeps discrete maneuvers consistent)
                self.pose_yaw = normalize_angle(target_yaw)
                return

            speed = min(
                self.MAX_TURN_SPEED,
                max(self.MIN_TURN_SPEED, self.TURN_SNAP_KP * abs(yaw_error)),
            )
            self._set_turn_wheels(ccw=yaw_error > 0, speed=speed, realtime=realtime)
            self._wait_and_step(1, realtime)

        self._stop()

    def move(self, direction: str, realtime=False, snap_yaw=True):
        """
        Move one cell in given direction (legacy discrete method).

        Args:
            direction: 'forward' or 'backward'
            realtime: Add visualization delays
            snap_yaw: If True, correct yaw after move (disable for batched moves)
        """
        is_forward = direction == "forward"
        speed_sign = 1 if is_forward else -1

        # Track progress using the internal continuous pose estimate.
        start_x, start_y = self.pose_x, self.pose_y
        target_yaw = ORIENTATION_ANGLES[self.kb.robot_heading]

        for _ in range(2000):
            self.update_pose(TIME_STEP)  # Update internal pose
            traveled = math.sqrt(
                (self.pose_x - start_x) ** 2 + (self.pose_y - start_y) ** 2
            )
            remaining = CELL_SIZE - traveled

            if remaining <= self.DISTANCE_TOLERANCE:
                break

            proportion = min(1.0, remaining / (CELL_SIZE * 0.5))
            speed = self.MIN_MOVE_SPEED + proportion * (
                self.MAX_MOVE_SPEED - self.MIN_MOVE_SPEED
            )

            yaw_error = normalize_angle(target_yaw - self.pose_yaw)
            correction = max(
                -self.MAX_HEADING_CORRECTION,
                min(self.MAX_HEADING_CORRECTION, self.HEADING_KP * yaw_error),
            )

            self._set_drive_wheels(speed_sign * speed, correction, realtime)
            self._wait_and_step(1, realtime)

        self._stop()
        self._wait_and_step(20, realtime=False)

        if snap_yaw:
            self._snap_to_kb_yaw(realtime)

    def turn(self, direction: str, realtime=False):
        """Turn 90° in given direction ('left' or 'right')."""
        # Relative 90° turn using gyro-integrated yaw estimate (no KB dependency).
        delta = math.pi / 2 if direction == "left" else -math.pi / 2
        self._turn_to_angle(normalize_angle(self.pose_yaw + delta), realtime)
        self._wait_and_step(30, realtime=False)  # Extra settling for compact robot
