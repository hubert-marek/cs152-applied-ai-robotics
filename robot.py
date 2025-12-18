"""
Robot Control - Low-level robot control for PyBullet simulation.

This module handles:
- Wheel velocity control
- Movement primitives (move N cells, turn 90°)
- Sensor reading (LiDAR, odometry, gyroscope)
"""

import math
import time
from collections import Counter

import pybullet

import kb as kb_store
import simulation
from mapping import integrate_lidar
from simulation import TIME_STEP, CELL_SIZE
from metrics import get_metrics

# LiDAR scan time in simulation steps
# A typical scanning LiDAR (e.g., RPLiDAR A1) rotates at ~5-10Hz,
# meaning a full 360° scan takes ~100-200ms. We use 50ms (12 steps
# at 240Hz) as a conservative estimate for a fast LiDAR.
LIDAR_SCAN_STEPS = 12  # ~50ms at 240Hz

# CONSTANTS

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


# Helper
def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


# LIDAR SENSOR


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
        ray_len = max(0.0, self.max_range - self.ray_start_offset)

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
                position[0] + self.max_range * math.cos(world_angle),
                position[1] + self.max_range * math.sin(world_angle),
                position[2],
            ]
            ray_to.append(end)

        results = pybullet.rayTestBatch(ray_from, ray_to)

        scan_data = []
        for i, result in enumerate(results):
            hit_id = result[0]
            hit_fraction = result[2]

            if hit_id == -1:
                distance = self.max_range
            else:
                # hit_fraction is along [ray_from, ray_to]; convert to distance from sensor origin
                distance = self.ray_start_offset + hit_fraction * ray_len
                distance = min(self.max_range, max(0.0, distance))

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


class RobotController:
    """
    Low-level robot control with proportional control.
    """

    # Control parameters
    DISTANCE_TOLERANCE = CELL_SIZE * 0.01
    ANGLE_TOLERANCE = math.radians(1.5)
    MAX_MOVE_SPEED = 10
    MIN_MOVE_SPEED = 2
    MAX_TURN_SPEED = 5
    MIN_TURN_SPEED = 1.5
    HEADING_KP = 6.0
    MAX_HEADING_CORRECTION = 4.0
    TURN_SNAP_KP = 6.0  #
    TURN_SNAP_MAX_STEPS = 800

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

        # Discovered wall positions (landmarks for recalibration) #TODO here we need to update the code to actually use algorithm for calssyfing walls. I thin kit should be easy, bascially make a datastrcutre that stores data from the entry
        #  (remember that north, south etc is all relevative - north is where robot starts), and then if a new wall is detected (the object size is bigger than box, i.e. bigger than 60 cm widfgt based on visible rays), check the angle of this, and verify against the internal measurments assgign ID to this wall and mark it as east, north or west etc.
        self._wall_landmarks = {}
        # Map PyBullet wall body id -> semantic wall type ("east"/"west"/"north"/"south").
        # This prevents the recalibration math from ever "matching" a north-wall hit to
        # an east/west wall line (a common failure mode seen in logs).
        self._wall_id_to_type: dict[int, str] = {}

        self.lidar = Lidar(robot_id)
        self._ccw_command_increases_yaw = None
        # Optional "magnet gripper" (implemented as a temporary fixed constraint). #TODO, remove all code related to magnet gripper - it is no longer needed
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

        This is NOT a physical jaw gripper; it's a convenience tool to model a
        magnetic coupler or suction cup that can rigidly attach once contact
        is established.

        Args:
            body_id: PyBullet body to attach to (e.g. the box)
            max_force: constraint max force
            require_contact: if True, only grip when currently in contact

        Returns:
            True if grip is active after the call.
        """
        if self._grip_constraint_id is not None:
            return True

        contacts = pybullet.getContactPoints(bodyA=self.robot_id, bodyB=body_id)
        if require_contact and not contacts:
            return False

        # Prefer the first contact point to define the attachment pivots.
        # If there is no contact (require_contact=False), fall back to body origins.
        if contacts:
            # contact[5] = positionOnA (world), contact[6] = positionOnB (world)
            world_pivot_a = contacts[0][5]
            world_pivot_b = contacts[0][6]
        else:
            world_pivot_a, _ = pybullet.getBasePositionAndOrientation(self.robot_id)
            world_pivot_b, _ = pybullet.getBasePositionAndOrientation(body_id)

        # Convert world pivots into each body's local frame
        a_pos, a_orn = pybullet.getBasePositionAndOrientation(self.robot_id)
        b_pos, b_orn = pybullet.getBasePositionAndOrientation(body_id)

        a_inv_pos, a_inv_orn = pybullet.invertTransform(a_pos, a_orn)
        b_inv_pos, b_inv_orn = pybullet.invertTransform(b_pos, b_orn)

        local_pivot_a, _ = pybullet.multiplyTransforms(
            a_inv_pos, a_inv_orn, world_pivot_a, (0, 0, 0, 1)
        )
        local_pivot_b, _ = pybullet.multiplyTransforms(
            b_inv_pos, b_inv_orn, world_pivot_b, (0, 0, 0, 1)
        )

        cid = pybullet.createConstraint(
            parentBodyUniqueId=self.robot_id,
            parentLinkIndex=-1,
            childBodyUniqueId=body_id,
            childLinkIndex=-1,
            jointType=pybullet.JOINT_FIXED,
            jointAxis=(0, 0, 0),
            parentFramePosition=local_pivot_a,
            childFramePosition=local_pivot_b,
        )
        pybullet.changeConstraint(cid, maxForce=max_force)
        self._grip_constraint_id = cid
        return True

    def get_actual_pose(self) -> dict:
        """
        Get actual pose from PyBullet (debug utility; avoid for estimation/control).

        Note on frames:
        - PyBullet reports WORLD coordinates.
        - This project uses an INTERNAL grid frame where the robot starts at (0,0),
          which (in `main.py`) corresponds to a PyBullet start cell like (1,1).
        - If KB bounds are set, we can infer the world→internal translation from them:
          internal_grid = world_grid + min_x/min_y, and internal_meters = world_meters + min_x*CELL_SIZE.
        """
        pos, orn = pybullet.getBasePositionAndOrientation(self.robot_id)
        euler = pybullet.getEulerFromQuaternion(orn)

        # Infer world→internal translation from KB bounds (set in main.py).
        if getattr(self.kb, "bounds", None) is not None:
            min_x, _, min_y, _ = self.kb.bounds
        else:
            min_x, min_y = 0, 0

        # Convert world meters to internal meters via translation.
        x_internal = pos[0] + (min_x * CELL_SIZE)
        y_internal = pos[1] + (min_y * CELL_SIZE)

        # Grid conversions for convenience (both frames).
        grid_x_world = int(round((pos[0] - CELL_SIZE / 2) / CELL_SIZE))
        grid_y_world = int(round((pos[1] - CELL_SIZE / 2) / CELL_SIZE))
        grid_x_internal = int(round((x_internal - CELL_SIZE / 2) / CELL_SIZE))
        grid_y_internal = int(round((y_internal - CELL_SIZE / 2) / CELL_SIZE))

        return {
            # World frame (PyBullet)
            "x": pos[0],
            "y": pos[1],
            "grid_x_world": grid_x_world,
            "grid_y_world": grid_y_world,
            # Internal frame (KB/mission/planning)
            "x_internal": x_internal,
            "y_internal": y_internal,
            "grid_x_internal": grid_x_internal,
            "grid_y_internal": grid_y_internal,
            "yaw": euler[2],
            "yaw_deg": math.degrees(euler[2]),
        }

    def scan_lidar(self, visualize=True, recalibrate=False):
        """
        Scan environment with LiDAR and update KB occupancy.

        Args:
            visualize: Draw LiDAR rays in simulation
            recalibrate: If True, also run landmark-based pose recalibration

        Returns:
            scan_data: List of (angle, distance, obj_id) tuples
        """
        # Account for realistic LiDAR scan time (~50ms)
        simulation.step(LIDAR_SCAN_STEPS)

        scan_data = self.lidar.scan(visualize=visualize)

        # Debug: show scan stats
        hits = sum(1 for _, _, oid in scan_data if oid != -1)
        print(f"  [DEBUG] LiDAR scan: {len(scan_data)} rays, {hits} hits")

        # Optionally recalibrate pose using wall landmarks BEFORE updating the KB.
        # IMPORTANT: if recalibration changes our best pose estimate, we also
        # sync the KB robot pose so planning/mapping share the same frame.
        if recalibrate and self._wall_landmarks:
            self._recalibrate_from_lidar(scan_data)
            get_metrics().record_recalibration()

        # Use the (possibly recalibrated) internal pose estimate to locate rays.
        # This prevents occupancy updates from being "painted" relative to a stale
        # discrete KB pose when the estimator drifts.
        grid_x = int(round((self.pose_x - CELL_SIZE / 2) / CELL_SIZE))
        grid_y = int(round((self.pose_y - CELL_SIZE / 2) / CELL_SIZE))

        # Keep KB discrete pose aligned to the robot estimate (position), but
        # preserve the KB heading (planner's discrete orientation).
        try:
            if getattr(self.kb, "in_bounds", None) is None or self.kb.in_bounds(
                grid_x, grid_y
            ):
                self.kb.set_robot(grid_x, grid_y, self.kb.robot_heading)
        except Exception:
            # Never let scan/mapping crash due to a transient bad pose estimate.
            pass

        integrate_lidar(
            self.kb,
            scan_data,
            cell_size=CELL_SIZE,
            max_range=self.lidar.max_range,
            is_obstacle=(lambda oid: oid in self.wall_ids) if self.wall_ids else None,
            actual_robot_internal=(grid_x, grid_y, self.pose_yaw),
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
            simulation.step(1)
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
        Update internal pose estimate from sensors.

        Uses velocity-based distance tracking (more reliable than wheel odometry
        which can have encoder issues) and gyroscope for heading.

        NOTE: LiDAR recalibration is DISABLED - it was causing drift by recording #TODO isn't this comment outdated as we are now colling recallibrate from LIdAR and veryfing based on knwoeldge about right angle of walls?
        landmarks from already-drifted positions. We rely on pose snapping after
        each waypoint to prevent drift accumulation.

        Args:
            dt: Time step (defaults to TIME_STEP)
            scan_data: Unused (kept for API compatibility)
        """
        if dt is None:
            dt = TIME_STEP

        # Integrate base velocities directly.
        #
        # Why: The robot can slip laterally (especially while pushing). If we only
        # integrate a scalar "forward distance" along pose_yaw, we will miss
        # sideways motion and accumulate large grid errors (observed in logs as
        # persistent GRID/BOX mismatches).
        linear_vel, angular_vel = pybullet.getBaseVelocity(self.robot_id)

        # 1) Heading from gyroscope (angular velocity around Z)
        self.pose_yaw += angular_vel[2] * dt
        self.pose_yaw = normalize_angle(self.pose_yaw)

        # 2) Position in internal/world XY (axes match; internal is a translation)
        self.pose_x += linear_vel[0] * dt
        self.pose_y += linear_vel[1] * dt

        # 3) LiDAR-based recalibration using known room boundaries
        # Only runs if scan_data is provided (typically after waypoint navigation)
        if scan_data:
            self._recalibrate_from_lidar(scan_data)

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

    def _classify_wall(self, abs_angle: float) -> str | None:
        """
        Classify which wall a ray is hitting based on its absolute angle.

        In a rectangular room with axis-aligned walls:
        - East wall (positive X): rays pointing roughly East (angle ≈ 0)
        - North wall (positive Y): rays pointing roughly North (angle ≈ π/2)
        - West wall (negative X): rays pointing roughly West (angle ≈ π or -π)
        - South wall (negative Y): rays pointing roughly South (angle ≈ -π/2)

        Returns wall name or None if angle is too diagonal.
        """
        # Normalize to [-π, π]
        angle = normalize_angle(abs_angle)
        angle_deg = math.degrees(angle)

        # Allow 45° cone for each wall direction
        if -45 < angle_deg < 45:
            return "east"
        elif 45 < angle_deg < 135:
            return "north"
        elif angle_deg > 135 or angle_deg < -135:
            return "west"
        elif -135 < angle_deg < -45:
            return "south"
        return None

    def _recalibrate_from_lidar(self, scan_data: list):
        """
        Correct continuous pose estimate (position AND heading) using wall landmarks.

        Uses proper trigonometry to work at ANY heading.

        Algorithm for POSITION:
        1. For each ray hitting a known wall, calculate EXPECTED distance
        2. Compare to MEASURED distance
        3. Convert distance error to XY error and correct pose

        Algorithm for HEADING:
        We estimate yaw by minimizing the geometric residual between LiDAR hit
        points and the known wall lines. For a candidate yaw θ, a wall-hit ray
        (α, d) implies a hit point:
            hx(θ) = pose_x + d*cos(θ+α)
            hy(θ) = pose_y + d*sin(θ+α)
        For an axis-aligned wall at x=const or y=const, the residual is the
        signed distance to that wall line (in meters). We search θ in a small
        window around the current estimate and pick the θ that minimizes a
        trimmed mean of squared residuals (robust to outliers / mislabels).
        """
        # Safety: without an explicit wall-id allowlist, we cannot safely
        # distinguish walls from dynamic objects (e.g., the box). In that case,
        # NEVER attempt "wall-based" recalibration.
        if not scan_data or not self._wall_landmarks or not self.wall_ids:
            return

        # DEBUG: Record pose before recalibration
        pose_before_x, pose_before_y = self.pose_x, self.pose_y

        # Position corrections are axis-constrained:
        # - east/west walls constrain X
        # - north/south walls constrain Y
        x_errors: list[float] = []
        y_errors: list[float] = []
        wall_ray_samples: list[
            tuple[str, float, float]
        ] = []  # (wall, d_measured, d_expected)

        for ray_angle, d_measured, obj_id in scan_data:
            # Only process wall hits (never box / robot / goal)
            if obj_id not in self.wall_ids:
                continue
            if obj_id == -1:  # No hit
                continue
            if d_measured <= 0.05 or d_measured > self.lidar.max_range + 1e-6:
                continue

            a = normalize_angle(self.pose_yaw + ray_angle)
            ca = math.cos(a)
            sa = math.sin(a)

            # Prefer a stable association: wall body id -> semantic wall type.
            wall_type = self._wall_id_to_type.get(int(obj_id))

            # If we haven't associated this wall id yet, do a conservative one-shot
            # association only when the ray is close to a cardinal direction.
            if wall_type is None:
                inferred = self._classify_wall(a)
                if inferred is None:
                    continue
                expected_dirs = {
                    "east": 0.0,
                    "north": math.pi / 2,
                    "west": math.pi,
                    "south": -math.pi / 2,
                }
                if abs(normalize_angle(a - expected_dirs[inferred])) > math.radians(20):
                    continue
                # Avoid mapping multiple body IDs to the same semantic wall.
                if inferred in self._wall_id_to_type.values():
                    continue
                wall_type = inferred
                self._wall_id_to_type[int(obj_id)] = wall_type

            key = f"wall_{wall_type}"
            if key not in self._wall_landmarks:
                # If we don't yet have this landmark, seed it from the current estimate.
                # This is safe-ish because we only do it under the conservative
                # near-cardinal gating above.
                if wall_type in ("east", "west"):
                    if abs(ca) < 0.2:
                        continue
                    self._wall_landmarks[key] = self.pose_x + d_measured * ca
                else:
                    if abs(sa) < 0.2:
                        continue
                    self._wall_landmarks[key] = self.pose_y + d_measured * sa

            wall_pos = float(self._wall_landmarks[key])

            # =========================================================
            # RESIDUAL COMPUTATION for position correction
            # =========================================================
            # For an east/west wall at x = wall_pos:
            #   - Ray hits at: implied_x = pose_x + d * cos(angle)
            #   - Residual = implied_x - wall_pos
            #   - Positive residual → we think we're too far east
            #
            # For a north/south wall at y = wall_pos:
            #   - Ray hits at: implied_y = pose_y + d * sin(angle)
            #   - Residual = implied_y - wall_pos
            #   - Positive residual → we think we're too far north
            #
            # We require the ray to have significant component toward the wall
            # (|cos| > 0.2 for X walls, |sin| > 0.2 for Y walls) to avoid
            # grazing rays which are noisy and unreliable.
            # =========================================================
            if wall_type in ("east", "west"):
                if abs(ca) < 0.2:  # Ray nearly parallel to wall - skip
                    continue
                implied_x = self.pose_x + d_measured * ca
                residual = implied_x - wall_pos
                if abs(residual) > CELL_SIZE * 1.0:  # Outlier - skip
                    continue
                x_errors.append(residual)
                d_expected = (wall_pos - self.pose_x) / ca
            else:
                if abs(sa) < 0.2:  # Ray nearly parallel to wall - skip
                    continue
                implied_y = self.pose_y + d_measured * sa
                residual = implied_y - wall_pos
                if abs(residual) > CELL_SIZE * 1.0:  # Outlier - skip
                    continue
                y_errors.append(residual)
                d_expected = (wall_pos - self.pose_y) / sa

            wall_ray_samples.append((wall_type, d_measured, float(d_expected)))

        # =====================================================================
        # POSITION CORRECTION using wall distance measurements
        # =====================================================================
        # We have collected residuals: (implied_pos - known_wall_pos) for each ray.
        # A positive residual means we think we're further from origin than we are.
        # Correct by subtracting the median residual (robust to outliers).
        #
        # Why median? Some rays might hit corners, box edges, or have noise.
        # Median ignores up to 50% outliers, mean would be corrupted.
        # =====================================================================
        corrections_made = []
        yaw_corrected = False

        if x_errors:
            # Median: robust against outliers (corners, noise, misclassification)
            x_errors_sorted = sorted(x_errors)
            median_x_error = x_errors_sorted[len(x_errors_sorted) // 2]
            if abs(median_x_error) > 0.01:  # Only correct errors > 1cm
                old_x = self.pose_x
                # Apply 80% of correction (damped to prevent oscillation)
                self.pose_x -= median_x_error * 0.8
                corrections_made.append(
                    f"X: {old_x:.3f}→{self.pose_x:.3f} (err={median_x_error:+.3f}m, n={len(x_errors)})"
                )

        if y_errors:
            y_errors_sorted = sorted(y_errors)
            median_y_error = y_errors_sorted[len(y_errors_sorted) // 2]
            if abs(median_y_error) > 0.01:  # Only correct errors > 1cm
                old_y = self.pose_y
                self.pose_y -= median_y_error * 0.8
                corrections_made.append(
                    f"Y: {old_y:.3f}→{self.pose_y:.3f} (err={median_y_error:+.3f}m, n={len(y_errors)})"
                )

        # =====================================================================
        # YAW CORRECTION using axis-aligned wall constraint
        # =====================================================================
        #
        # KEY INSIGHT: In an axis-aligned environment, we don't need two
        # perpendicular walls for yaw correction. A SINGLE wall is sufficient!
        #
        # Why? Consider rays hitting an east wall at x = W:
        #   - Ray at relative angle α with distance d hits at:
        #     hit_x = pose_x + d * cos(yaw + α)
        #   - For the ray to hit the wall: hit_x should equal W
        #
        # If our yaw estimate is WRONG:
        #   - Different rays will compute different "implied" wall positions
        #   - The VARIANCE of implied positions reveals yaw error
        #   - The CORRECT yaw makes all rays agree on wall position
        #
        # We search yaw values in a small window and pick the one that
        # minimizes the variance (squared residuals) of implied wall positions.
        #
        # This works because we're correcting DRIFT, not estimating yaw
        # from scratch. We have a prior that's within ~10-15° of truth.
        # =====================================================================

        def _yaw_cost(theta: float) -> float | None:
            """
            Compute cost (mean squared residual) for candidate yaw θ.

            Lower cost = rays hitting walls imply more consistent wall positions.
            Returns None if insufficient data for reliable estimate.
            """
            residual2: list[float] = []

            for ray_angle, d_measured, obj_id in scan_data:
                # Filter: only wall hits with valid measurements
                if obj_id not in self.wall_ids:
                    continue
                if obj_id == -1:
                    continue
                if d_measured <= 0.05 or d_measured > self.lidar.max_range + 1e-6:
                    continue

                wall_type = self._wall_id_to_type.get(int(obj_id))
                if wall_type is None:
                    continue
                key = f"wall_{wall_type}"
                if key not in self._wall_landmarks:
                    continue
                wall_pos = float(self._wall_landmarks[key])

                # Compute absolute angle of this ray in world frame
                a = normalize_angle(theta + ray_angle)
                ca = math.cos(a)
                sa = math.sin(a)

                # Compute residual: difference between implied and known wall position
                if wall_type in ("east", "west"):
                    # X-axis walls: ray must have significant X component
                    if abs(ca) < 0.2:
                        continue
                    # Implied wall position from this ray
                    implied_wall_x = self.pose_x + d_measured * ca
                    r = implied_wall_x - wall_pos
                else:
                    # Y-axis walls: ray must have significant Y component
                    if abs(sa) < 0.2:
                        continue
                    implied_wall_y = self.pose_y + d_measured * sa
                    r = implied_wall_y - wall_pos

                # Reject outliers (likely misclassified or noise)
                if abs(r) > CELL_SIZE * 1.0:
                    continue
                residual2.append(r * r)

            # Need sufficient samples for reliable correction
            # NOTE: Single wall is enough! We don't require both X and Y walls
            # because variance of implied positions detects yaw error either way
            if len(residual2) < 8:
                return None

            # Robust statistic: trimmed mean (ignore worst 30% as outliers)
            residual2.sort()
            keep = max(5, int(len(residual2) * 0.7))
            return sum(residual2[:keep]) / keep

        # Evaluate cost at current yaw estimate
        current_cost = _yaw_cost(self.pose_yaw)
        best_theta = self.pose_yaw
        best_cost = current_cost if current_cost is not None else float("inf")

        # Search a ±25° window around current yaw in 0.5° steps
        # This is drift correction, not full estimation - we expect small errors
        window = math.radians(25.0)
        step = math.radians(0.5)
        theta = self.pose_yaw - window
        end = self.pose_yaw + window
        while theta <= end:
            c = _yaw_cost(theta)
            if c is not None and c < best_cost:
                best_cost = c
                best_theta = theta
            theta += step

        # Apply correction only if:
        # 1. We have a valid current cost (enough wall data)
        # 2. New yaw is significantly better (15% improvement threshold)
        # 3. Correction is in reasonable range (2°-15°) - not noise, not catastrophic
        if current_cost is not None:
            if best_cost < current_cost * 0.85:  # 15% improvement required
                yaw_err = normalize_angle(self.pose_yaw - best_theta)

                # Safety bounds:
                # - Below 2°: likely just noise, don't correct
                # - Above 15°: too large, might be a misdetection
                if math.radians(2.0) < abs(yaw_err) < math.radians(15.0):
                    old_yaw = self.pose_yaw
                    # Apply 70% of correction (damped to prevent oscillation)
                    self.pose_yaw = normalize_angle(self.pose_yaw - yaw_err * 0.7)
                    yaw_corrected = True
                    corrections_made.append(
                        f"YAW: {math.degrees(old_yaw):.1f}°→{math.degrees(self.pose_yaw):.1f}° "
                        f"(err={math.degrees(yaw_err):+.1f}°, n={len(scan_data)})"
                    )

        # =====================================================================
        # POST-YAW POSITION REFINEMENT
        # =====================================================================
        # After correcting yaw, our position estimates (computed with the OLD
        # yaw) are slightly off. Recalculate X/Y corrections using the NEW yaw.
        # This is a lightweight second pass - we already have wall associations.
        # =====================================================================
        if yaw_corrected:
            x2: list[float] = []
            y2: list[float] = []
            for ray_angle, d_measured, obj_id in scan_data:
                if obj_id not in self.wall_ids:
                    continue
                if obj_id == -1:
                    continue
                if d_measured <= 0.05 or d_measured > self.lidar.max_range + 1e-6:
                    continue

                wall_type = self._wall_id_to_type.get(int(obj_id))
                if wall_type is None:
                    continue
                key = f"wall_{wall_type}"
                if key not in self._wall_landmarks:
                    continue
                wall_pos = float(self._wall_landmarks[key])

                abs_angle = normalize_angle(self.pose_yaw + ray_angle)
                ca = math.cos(abs_angle)
                sa = math.sin(abs_angle)

                if wall_type in ("east", "west"):
                    if abs(ca) < 0.2:
                        continue
                    residual = (self.pose_x + d_measured * ca) - wall_pos
                    if abs(residual) > CELL_SIZE * 1.0:
                        continue
                    x2.append(residual)
                else:
                    if abs(sa) < 0.2:
                        continue
                    residual = (self.pose_y + d_measured * sa) - wall_pos
                    if abs(residual) > CELL_SIZE * 1.0:
                        continue
                    y2.append(residual)

            if x2:
                x2s = sorted(x2)
                dx = x2s[len(x2s) // 2]
                if abs(dx) > 0.005:
                    self.pose_x -= dx * 0.5
            if y2:
                y2s = sorted(y2)
                dy = y2s[len(y2s) // 2]
                if abs(dy) > 0.005:
                    self.pose_y -= dy * 0.5

        if corrections_made:
            print(f"    [RECALIB] {', '.join(corrections_made)}")

            # DEBUG: Show details of wall rays used for recalibration
            total_correction = math.sqrt(
                (self.pose_x - pose_before_x) ** 2 + (self.pose_y - pose_before_y) ** 2
            )
            if total_correction > 0.15:  # Log if correction is large (> 15cm)
                print(
                    f"      [DEBUG] Large correction! Total={total_correction:.3f}m from {len(wall_ray_samples)} wall rays:"
                )
                for i, (wall_name, d_m, d_e) in enumerate(wall_ray_samples[:5]):
                    print(
                        f"        Ray {i + 1}: {wall_name} wall, measured={d_m:.3f}m, expected={d_e:.3f}m, error={(d_m - d_e):+.3f}m"
                    )
                if len(wall_ray_samples) > 5:
                    print(f"        ... and {len(wall_ray_samples) - 5} more rays")
                print(
                    f"      [DEBUG] Landmarks: {dict((k, f'{v:.3f}m') for k, v in self._wall_landmarks.items())}"
                )

    def record_wall_landmarks(self, scan_data: list = None) -> int:
        """
        Record wall positions as landmarks from current (trusted) position.

        Uses proper trigonometry to calculate wall positions from LiDAR rays.
        Call this when robot is at a verified position (e.g., at initialization
        where we KNOW we're at (0,0)). The current pose is assumed accurate.

        For each wall (east/west/north/south), we record its axis-aligned position:
        - East/West walls: X coordinate where the wall exists
        - North/South walls: Y coordinate where the wall exists

        Returns:
            Number of new landmarks recorded.
        """
        if scan_data is None:
            # Account for LiDAR scan time (~50ms)
            simulation.step(LIDAR_SCAN_STEPS)
            scan_data = self.lidar.scan(visualize=True)

        if not scan_data:
            return 0

        # Collect wall position samples for each wall type
        wall_samples = {"east": [], "west": [], "north": [], "south": []}
        # Also remember which wall body IDs contributed to each type.
        wall_id_votes: dict[str, Counter[int]] = {
            "east": Counter(),
            "west": Counter(),
            "north": Counter(),
            "south": Counter(),
        }

        for ray_angle, distance, obj_id in scan_data:
            # Only process wall hits
            if self.wall_ids and obj_id not in self.wall_ids:
                continue
            if obj_id == -1:  # No hit
                continue

            # Calculate absolute angle of this ray
            abs_angle = normalize_angle(self.pose_yaw + ray_angle)

            # Classify which wall this ray is hitting
            wall_type = self._classify_wall(abs_angle)
            if wall_type is None:
                continue
            # Be strict when recording landmarks: only accept rays close to the
            # wall's outward normal direction. This prevents diagonal rays that
            # actually hit a different wall from being misclassified and "poisoning"
            # the landmark set at initialization.
            expected_dirs = {
                "east": 0.0,
                "north": math.pi / 2,
                "west": math.pi,
                "south": -math.pi / 2,
            }
            if abs(
                normalize_angle(abs_angle - expected_dirs[wall_type])
            ) > math.radians(25):
                continue

            # Calculate wall position using trigonometry
            # Hit point: (pose_x + d*cos(abs_angle), pose_y + d*sin(abs_angle))
            hit_x = self.pose_x + distance * math.cos(abs_angle)
            hit_y = self.pose_y + distance * math.sin(abs_angle)

            # For axis-aligned walls, record the relevant coordinate
            if wall_type in ("east", "west"):
                wall_samples[wall_type].append(hit_x)
            else:  # north, south
                wall_samples[wall_type].append(hit_y)
            wall_id_votes[wall_type][int(obj_id)] += 1

        # Record landmarks using median of samples (robust to noise)
        recorded = 0
        for wall_type, samples in wall_samples.items():
            if len(samples) < 3:  # Need enough samples for reliability
                continue

            # Use median for robustness
            samples_sorted = sorted(samples)
            wall_pos = samples_sorted[len(samples_sorted) // 2]

            landmark_key = f"wall_{wall_type}"
            old_val = self._wall_landmarks.get(landmark_key)
            self._wall_landmarks[landmark_key] = wall_pos
            recorded += 1

            # If we have enough votes, bind the dominant wall body id to this type.
            # This makes later recalibration use the wall id directly (no guessing).
            if wall_id_votes[wall_type]:
                wall_id, count = wall_id_votes[wall_type].most_common(1)[0]
                if count >= 3:
                    self._wall_id_to_type[wall_id] = wall_type

            axis = "X" if wall_type in ("east", "west") else "Y"
            if old_val is None:
                print(
                    f"    [LANDMARK] New: {wall_type} wall at {axis}={wall_pos:.3f}m (n={len(samples)})"
                )
            elif abs(old_val - wall_pos) > 0.05:
                print(
                    f"    [LANDMARK] Updated: {wall_type} wall {axis}: {old_val:.3f}→{wall_pos:.3f}m"
                )

        return recorded

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
        # DEBUG: Show before/after snap
        old_x, old_y = self.pose_x, self.pose_y
        actual = self.get_actual_pose()

        self.pose_x = grid_x * CELL_SIZE + CELL_SIZE / 2
        self.pose_y = grid_y * CELL_SIZE + CELL_SIZE / 2
        self.pose_yaw = ORIENTATION_ANGLES[heading]

        # Calculate how much we corrected
        correction = math.sqrt((self.pose_x - old_x) ** 2 + (self.pose_y - old_y) ** 2)
        # Compare against actual pose in INTERNAL frame (world is translated).
        actual_error = math.sqrt(
            (self.pose_x - actual["x_internal"]) ** 2
            + (self.pose_y - actual["y_internal"]) ** 2
        )

        # DEBUG: Show coordinate conversion details if correction is significant
        if correction > 0.02:
            print(
                f"      [SNAP-DEBUG] Conversion: grid({grid_x},{grid_y}) → meters({self.pose_x:.3f},{self.pose_y:.3f})"
            )
            print(
                f"      [SNAP-DEBUG] Internal pose: before=({old_x:.3f},{old_y:.3f}), after=({self.pose_x:.3f},{self.pose_y:.3f})"
            )

        print(
            f"      [SNAP] Grid ({grid_x},{grid_y}) h={heading}: correction={correction:.3f}m, actual_error={actual_error:.3f}m "
            f"[world=({actual['x']:.3f},{actual['y']:.3f})]"
        )

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

    def move_to_waypoint(  # TODO Here I think again more explinations, and I am aslo thinkiing if this shouldnt be refractortd to mission planner, and in robot we just keep the controller? But I am open to leaving this but want to hear your opinion
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

        # DEBUG: Log starting state
        actual = self.get_actual_pose()
        print(
            f"    [NAV-START] Target grid=({target_grid_x},{target_grid_y}) heading={target_heading}"
        )
        print(
            f"      Internal: pos=({self.pose_x:.3f},{self.pose_y:.3f}) yaw={math.degrees(self.pose_yaw):.1f}°"
        )
        print(
            f"      Actual:   pos=({actual['x_internal']:.3f},{actual['y_internal']:.3f}) yaw={actual['yaw_deg']:.1f}° "
            f"[world=({actual['x']:.3f},{actual['y']:.3f})]"
        )
        print(f"      Target:   pos=({target_x:.3f},{target_y:.3f})")

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

            # Check if arrived (within 2% of cell size = 2.5cm)
            if distance < CELL_SIZE * 0.02:
                reached = True
                break

            # Recalculate angle to target (correct for drift)
            target_angle = math.atan2(dy, dx)
            yaw_error = normalize_angle(target_angle - self.pose_yaw)

            # Proportional speed control - slow down more when close
            proportion = min(
                1.0, distance / (CELL_SIZE * 0.5)
            )  # Start slowing at half cell
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

        # Post-motion pose correction using LiDAR landmarks (NO KB updates here).
        # This is critical for turns-in-place, which can introduce translational slip;
        # without this, the mission can falsely conclude it "missed" a waypoint and
        # spiral into repeated replanning.

        # Measure distance BEFORE recalibration
        pre_recalib_dx = target_x - self.pose_x
        pre_recalib_dy = target_y - self.pose_y
        pre_recalib_dist = math.sqrt(
            pre_recalib_dx * pre_recalib_dx + pre_recalib_dy * pre_recalib_dy
        )

        if self._wall_landmarks:
            # Account for LiDAR scan time (~50ms)
            simulation.step(LIDAR_SCAN_STEPS)
            scan_data = self.lidar.scan(visualize=True)
            self._recalibrate_from_lidar(scan_data)

        # DEBUG: Show arrival accuracy - compare internal vs actual
        final_dx = target_x - self.pose_x
        final_dy = target_y - self.pose_y
        final_dist = math.sqrt(final_dx * final_dx + final_dy * final_dy)

        # DEBUG: Warn if recalibration made arrival WORSE
        if abs(final_dist - pre_recalib_dist) > 0.05:
            delta_sign = "worse" if final_dist > pre_recalib_dist else "better"
            print(
                f"      [DEBUG] Recalib changed distance-to-target: {pre_recalib_dist:.3f}m → {final_dist:.3f}m ({delta_sign})"
            )

        # Re-evaluate arrival after recalibration
        reached = final_dist < CELL_SIZE * 0.08

        actual = self.get_actual_pose()
        actual_dx = target_x - actual["x_internal"]
        actual_dy = target_y - actual["y_internal"]
        actual_dist = math.sqrt(actual_dx * actual_dx + actual_dy * actual_dy)

        internal_grid_x = int(round((self.pose_x - CELL_SIZE / 2) / CELL_SIZE))
        internal_grid_y = int(round((self.pose_y - CELL_SIZE / 2) / CELL_SIZE))
        actual_grid_x = actual["grid_x_internal"]
        actual_grid_y = actual["grid_y_internal"]

        print(
            f"    [NAV-END] Waypoint ({target_grid_x},{target_grid_y}): arrived={reached}"
        )
        print(
            f"      Internal: pos=({self.pose_x:.3f},{self.pose_y:.3f}) grid=({internal_grid_x},{internal_grid_y}) error={final_dist:.3f}m"
        )
        print(
            f"      Actual:   pos=({actual['x_internal']:.3f},{actual['y_internal']:.3f}) grid=({actual_grid_x},{actual_grid_y}) error={actual_dist:.3f}m "
            f"[world=({actual['x']:.3f},{actual['y']:.3f}), grid=({actual['grid_x_world']},{actual['grid_y_world']})]"
        )

        # WARN if internal and actual disagree on grid cell
        if (internal_grid_x, internal_grid_y) != (actual_grid_x, actual_grid_y):
            print(
                f"      ⚠️  GRID MISMATCH! Internal thinks ({internal_grid_x},{internal_grid_y}) but actually at ({actual_grid_x},{actual_grid_y})"
            )
        if (internal_grid_x, internal_grid_y) != (target_grid_x, target_grid_y):
            print(
                f"      ⚠️  MISSED TARGET! Target was ({target_grid_x},{target_grid_y})"
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

    def move(
        self, direction: str, realtime=False, snap_yaw=True
    ):  # TODO if this is legacy why do we keep it?
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

    def turn(
        self, direction: str, realtime=False
    ):  # TODO DO we actually need it if we have turn to angle
        """Turn 90° in given direction ('left' or 'right')."""
        # Relative 90° turn using gyro-integrated yaw estimate (no KB dependency).
        delta = math.pi / 2 if direction == "left" else -math.pi / 2
        self._turn_to_angle(normalize_angle(self.pose_yaw + delta), realtime)
        self._wait_and_step(30, realtime=False)  # Extra settling for compact robot
