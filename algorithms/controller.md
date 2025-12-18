# Motion Controller Algorithms

## 1. Differential Drive Kinematics

### 1.1 Forward Kinematics

For a robot with two driven wheels:

**Linear velocity:**
$$
v = \frac{v_L + v_R}{2}
$$

**Angular velocity:**
$$
\omega = \frac{v_R - v_L}{W}
$$

Where:
- $v_L, v_R$ = left/right wheel linear velocities
- $W$ = wheel base (distance between wheels)

### 1.2 Inverse Kinematics

Given desired $(v, \omega)$:

$$
v_L = v - \frac{\omega \cdot W}{2}
$$
$$
v_R = v + \frac{\omega \cdot W}{2}
$$

### 1.3 Wheel Velocity to Motor Command

$$
\omega_{wheel} = \frac{v_{wheel}}{r_{wheel}}
$$

Where $r_{wheel} = 0.05$m.

---

## 2. Proportional Control

### 2.1 P-Controller

$$
u = K_p \cdot e
$$

Where:
- $u$ = control output
- $K_p$ = proportional gain
- $e$ = error (setpoint - measurement)

### 2.2 Clamped P-Controller

$$
u = \text{clamp}(K_p \cdot e, -u_{max}, +u_{max})
$$

```
clamp(x, lo, hi):
    return max(lo, min(hi, x))
```

---

## 3. Heading Hold Controller

### 3.1 Error Calculation

$$
e_\theta = \text{normalize}(\theta_{target} - \theta_{current})
$$

Where normalize wraps to $[-\pi, \pi]$.

### 3.2 Steering Correction

$$
\omega_{correction} = \text{clamp}(K_p \cdot e_\theta, -\omega_{max}, +\omega_{max})
$$

With our parameters:
- $K_p = 6.0$
- $\omega_{max} = 4.0$ rad/s

### 3.3 Drive Command with Steering

$$
v_L = v_{base} - \omega_{correction}
$$
$$
v_R = v_{base} + \omega_{correction}
$$

Note: Sign depends on turn polarity calibration.

---

## 4. Waypoint Navigation

### 4.1 Target Angle Calculation

$$
\theta_{target} = \text{atan2}(y_{goal} - y_{current}, x_{goal} - x_{current})
$$

### 4.2 Distance to Target

$$
d = \sqrt{(x_{goal} - x_{current})^2 + (y_{goal} - y_{current})^2}
$$

### 4.3 Proportional Speed Control

Slow down when approaching target:

$$
\rho = \min\left(1.0, \frac{d}{d_{slowdown}}\right)
$$
$$
v = v_{min} + \rho \cdot (v_{max} - v_{min})
$$

With our parameters:
- $d_{slowdown} = 0.25$m (half cell)
- $v_{min} = 2$ rad/s
- $v_{max} = 10$ rad/s

### 4.4 Arrival Detection

$$
\text{arrived} \iff d < d_{tolerance}
$$

With $d_{tolerance} = 0.01$m (1cm).

---

## 5. Turn-to-Angle Controller

### 5.1 Angular Error

$$
e_\theta = \text{normalize}(\theta_{target} - \theta_{current})
$$

### 5.2 Turn Speed

$$
\omega = \text{clamp}(K_p \cdot |e_\theta|, \omega_{min}, \omega_{max})
$$

With our parameters:
- $K_p = 6.0$
- $\omega_{min} = 1.5$ rad/s
- $\omega_{max} = 5.0$ rad/s

### 5.3 Turn Direction

$$
\text{ccw} = (e_\theta > 0)
$$

Counter-clockwise if error is positive (need to turn left).

### 5.4 Wheel Commands

```
if ccw:
    left = -ω, right = +ω
else:
    left = +ω, right = -ω
```

Note: Sign depends on robot's turn polarity calibration.

### 5.5 Convergence Check

$$
\text{converged} \iff |e_\theta| < \epsilon_\theta
$$

With $\epsilon_\theta = 1.5°$ (0.026 rad).

---

## 6. Turn Polarity Calibration

### 6.1 Problem

Different robots may have opposite wheel conventions. A "CCW command" (left wheel backward, right wheel forward) may result in:
- +yaw (counter-clockwise rotation) on some robots
- -yaw (clockwise rotation) on others

### 6.2 Calibration Algorithm

```
calibrate_turn_polarity():
    # Apply CCW command pattern
    set_wheels(-MIN_SPEED, +MIN_SPEED)
    
    integrated_yaw = 0
    for 10 timesteps:
        step_simulation()
        integrated_yaw += gyroscope() * dt
    
    stop()
    
    # If yaw increased, CCW command → +yaw
    ccw_increases_yaw = (integrated_yaw > 0)
```

### 6.3 Applying Calibration

```
set_turn_wheels(ccw, speed):
    if ccw_increases_yaw:
        if ccw: left, right = -speed, +speed
        else:   left, right = +speed, -speed
    else:
        if ccw: left, right = +speed, -speed
        else:   left, right = -speed, +speed
    
    set_wheel_velocities(left, right)
```

---

## 7. Three-Phase Waypoint Navigation

### Phase 1: Initial Turn

```
dx = target_x - pose_x
dy = target_y - pose_y
target_angle = atan2(dy, dx)

yaw_error = normalize(target_angle - pose_yaw)
if |yaw_error| > ANGLE_TOLERANCE:
    turn_to_angle(target_angle)
```

### Phase 2: Drive with Heading Hold

```
for step in 0..MAX_STEPS:
    update_pose(dt)
    
    dx = target_x - pose_x
    dy = target_y - pose_y
    distance = sqrt(dx² + dy²)
    
    if distance < DISTANCE_TOLERANCE:
        break  # Arrived
    
    target_angle = atan2(dy, dx)
    yaw_error = normalize(target_angle - pose_yaw)
    
    proportion = min(1.0, distance / SLOWDOWN_DIST)
    speed = MIN_SPEED + proportion * (MAX_SPEED - MIN_SPEED)
    
    correction = clamp(HEADING_KP * yaw_error, -MAX_CORRECTION, MAX_CORRECTION)
    
    set_drive_wheels(speed, correction)
    step_simulation()

stop()
```

### Phase 3: Final Heading Correction

```
if target_heading is not None:
    final_yaw = ORIENTATION_ANGLES[target_heading]
    turn_to_angle(final_yaw)
```

---

## 8. Contact-Based Pushing

### 8.1 Approach Phase

Drive toward box until contact:

```
while not in_contact_with(box_id):
    drive_forward(slow_speed)
    if distance_traveled > max_approach:
        break  # Box not where expected
```

### 8.2 Push Phase

Continue driving while maintaining contact:

```
start_pos = (pose_x, pose_y)
while distance_from(start_pos) < CELL_SIZE:
    if not in_contact_with(box_id):
        break  # Lost contact
    
    drive_forward(push_speed)
    maintain_heading(push_direction)
```

### 8.3 Verification

After push, rescan to verify box moved:

```
scan_lidar()
new_box_pos = detect_box()

if new_box_pos != expected_pos:
    replan()
```

---

## 9. Parameter Summary

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Distance tolerance | $d_{tol}$ | 0.005 | m |
| Angle tolerance | $\theta_{tol}$ | 1.5 | degrees |
| Max move speed | $v_{max}$ | 10 | rad/s |
| Min move speed | $v_{min}$ | 2 | rad/s |
| Max turn speed | $\omega_{max}$ | 5 | rad/s |
| Min turn speed | $\omega_{min}$ | 1.5 | rad/s |
| Heading gain | $K_p$ | 6.0 | - |
| Max heading correction | $\omega_{corr,max}$ | 4.0 | rad/s |
| Slowdown distance | $d_{slow}$ | 0.25 | m |
| Cell size | $C$ | 0.5 | m |
| Wheel radius | $r$ | 0.05 | m |
| Timestep | $dt$ | 1/240 | s |

