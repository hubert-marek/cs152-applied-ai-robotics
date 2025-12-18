# Algorithm Reference

This folder contains detailed mathematical specifications for algorithms used in the box-pushing robot.

## Files

| File | Contents |
|------|----------|
| [`controller.md`](controller.md) | Differential drive kinematics, P-control, waypoint navigation, turn polarity calibration |
| [`coordinate_transforms.md`](coordinate_transforms.md) | Grid↔meters, internal↔world frame, robot frame transforms, angle conversions |

See the main [README.md](../README.md) for project setup and configuration.

## Quick Reference

### Key Formulas

**Pose Update (Dead Reckoning):**
```
θ += ω_z · dt
x += v_x · dt  
y += v_y · dt
```

**Grid ↔ Meters:**
```
meters = grid · CELL_SIZE + CELL_SIZE/2
grid = floor(meters / CELL_SIZE)
```

**A* Cost:**
```
f(n) = g(n) + h(n)
g(n) = path_cost + turn_cost × num_turns
h(n) = |goal_x - x| + |goal_y - y|  (Manhattan)
```

**LiDAR Hit Position:**
```
x_hit = x_robot + distance · cos(θ_robot + α_ray)
y_hit = y_robot + distance · sin(θ_robot + α_ray)
```

**Heading Hold:**
```
error = normalize(θ_target - θ_current)
correction = clamp(K_p · error, -max, +max)
```

### Constants

| Constant | Value | Unit |
|----------|-------|------|
| CELL_SIZE | 0.5 | m |
| LIDAR_RANGE | 2.0 | m |
| LIDAR_NUM_RAYS | 72 | - |
| WHEEL_RADIUS | 0.05 | m |
| TIME_STEP | 1/240 | s |
| HEADING_KP | 6.0 | - |
| SLIP_THRESHOLD | 0.10 | (10%) |

### Heading Values

| Direction | Constant | Yaw (rad) | Vector |
|-----------|----------|-----------|--------|
| NORTH | 0 | +π/2 | (0, +1) |
| EAST | 1 | 0 | (+1, 0) |
| SOUTH | 2 | -π/2 | (0, -1) |
| WEST | 3 | π | (-1, 0) |

