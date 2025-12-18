# Coordinate Transformations

## 1. Coordinate Frames

### 1.1 Frame Definitions

| Frame | Origin | X-axis | Y-axis | Usage |
|-------|--------|--------|--------|-------|
| **PyBullet World** | Simulation origin | East | North | Physics engine |
| **Internal** | Robot spawn position | East | North | Planning, KB, estimation |
| **Robot Body** | Robot center | Forward | Left | Sensor readings |
| **Grid** | Internal origin | East (cells) | North (cells) | Discrete planning |

### 1.2 Frame Relationships

```
PyBullet World ←→ Internal: Translation only (no rotation)
Internal ←→ Grid: Scale by CELL_SIZE
Robot Body ←→ Internal: Rotation by robot yaw + translation by robot position
```

---

## 2. Grid ↔ Meters Conversions

### 2.1 Grid Cell to Meters (Cell Center)

$$
x_m = g_x \cdot C + \frac{C}{2}
$$
$$
y_m = g_y \cdot C + \frac{C}{2}
$$

**Example:** Cell $(0, 0)$ with $C = 0.5$m → $(0.25, 0.25)$m

### 2.2 Meters to Grid Cell

$$
g_x = \left\lfloor \frac{x_m}{C} \right\rfloor
$$
$$
g_y = \left\lfloor \frac{y_m}{C} \right\rfloor
$$

**Alternative (cell-centered rounding):**
$$
g_x = \text{round}\left(\frac{x_m - C/2}{C}\right)
$$

### 2.3 Grid Cell Bounds

Cell $(g_x, g_y)$ covers:
$$
x \in [g_x \cdot C, (g_x + 1) \cdot C)
$$
$$
y \in [g_y \cdot C, (g_y + 1) \cdot C)
$$

---

## 3. Internal ↔ World Frame

### 3.1 Translation Definition

The internal frame is defined such that the robot starts at grid $(0, 0)$.

If robot spawns at PyBullet world position $(x_{spawn}, y_{spawn})$:

$$
x_{internal} = x_{world} - x_{spawn} + \frac{C}{2}
$$
$$
y_{internal} = y_{world} - y_{spawn} + \frac{C}{2}
$$

The $+C/2$ offset ensures the spawn position is at cell $(0,0)$ center.

### 3.2 Inverse Transform

$$
x_{world} = x_{internal} + x_{spawn} - \frac{C}{2}
$$
$$
y_{world} = y_{internal} + y_{spawn} - \frac{C}{2}
$$

### 3.3 Using KB Bounds

If KB bounds are set as $(min_x, max_x, min_y, max_y)$:

$$
x_{internal} = x_{world} + min_x \cdot C
$$
$$
y_{internal} = y_{world} + min_y \cdot C
$$

Note: $min_x, min_y$ are typically negative (e.g., $-1$).

---

## 4. Robot Frame ↔ World Frame

### 4.1 Point Transform (Robot → World)

For a point $(x_r, y_r)$ in robot frame:

$$
\begin{pmatrix} x_w \\ y_w \end{pmatrix} = 
\begin{pmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{pmatrix}
\begin{pmatrix} x_r \\ y_r \end{pmatrix} +
\begin{pmatrix} x_{robot} \\ y_{robot} \end{pmatrix}
$$

### 4.2 Point Transform (World → Robot)

$$
\begin{pmatrix} x_r \\ y_r \end{pmatrix} = 
\begin{pmatrix} \cos\theta & \sin\theta \\ -\sin\theta & \cos\theta \end{pmatrix}
\begin{pmatrix} x_w - x_{robot} \\ y_w - y_{robot} \end{pmatrix}
$$

### 4.3 Angle Transform

Robot-frame angle to world-frame:
$$
\alpha_{world} = \alpha_{robot} + \theta_{robot}
$$

World-frame angle to robot-frame:
$$
\alpha_{robot} = \alpha_{world} - \theta_{robot}
$$

---

## 5. LiDAR Ray Transforms

### 5.1 Ray Angle Convention

- LiDAR angle $\alpha = 0$ points forward (robot X-axis)
- Angles increase counter-clockwise
- Range: $\alpha \in [0, 2\pi)$

### 5.2 Ray to World Angle

$$
\alpha_{abs} = \theta_{robot} + \alpha_{lidar}
$$

Then normalize to $[-\pi, \pi]$.

### 5.3 Hit Point Calculation

$$
x_{hit} = x_{robot} + d \cdot \cos(\alpha_{abs})
$$
$$
y_{hit} = y_{robot} + d \cdot \sin(\alpha_{abs})
$$

---

## 6. Discrete Heading ↔ Continuous Yaw

### 6.1 Heading to Yaw

| Heading | Constant | Yaw (radians) |
|---------|----------|---------------|
| NORTH | 0 | $+\frac{\pi}{2}$ |
| EAST | 1 | $0$ |
| SOUTH | 2 | $-\frac{\pi}{2}$ |
| WEST | 3 | $\pi$ or $-\pi$ |

```python
ORIENTATION_ANGLES = {
    NORTH: π/2,
    EAST: 0,
    SOUTH: -π/2,
    WEST: π
}
```

### 6.2 Yaw to Heading

First normalize yaw to $[0, 2\pi)$:
$$
\theta_{norm} = \theta \mod 2\pi
$$

Then:
$$
\text{heading} = \begin{cases}
\text{EAST} & \text{if } \theta_{norm} \in [0, \frac{\pi}{4}) \cup [\frac{7\pi}{4}, 2\pi) \\
\text{NORTH} & \text{if } \theta_{norm} \in [\frac{\pi}{4}, \frac{3\pi}{4}) \\
\text{WEST} & \text{if } \theta_{norm} \in [\frac{3\pi}{4}, \frac{5\pi}{4}) \\
\text{SOUTH} & \text{if } \theta_{norm} \in [\frac{5\pi}{4}, \frac{7\pi}{4})
\end{cases}
$$

---

## 7. Direction Vectors

### 7.1 Heading to Direction

| Heading | Direction $(dx, dy)$ |
|---------|---------------------|
| NORTH | $(0, +1)$ |
| EAST | $(+1, 0)$ |
| SOUTH | $(0, -1)$ |
| WEST | $(-1, 0)$ |

### 7.2 Direction to Heading

| $(dx, dy)$ | Heading |
|------------|---------|
| $(0, +1)$ | NORTH |
| $(+1, 0)$ | EAST |
| $(0, -1)$ | SOUTH |
| $(-1, 0)$ | WEST |

---

## 8. Angle Normalization

### 8.1 Normalize to $[-\pi, \pi]$

```
normalize_angle(θ):
    while θ > π:
        θ = θ - 2π
    while θ < -π:
        θ = θ + 2π
    return θ
```

Efficient version:
$$
\theta_{norm} = \text{atan2}(\sin\theta, \cos\theta)
$$

### 8.2 Normalize to $[0, 2\pi)$

$$
\theta_{norm} = \theta \mod 2\pi
$$

If result is negative, add $2\pi$.

### 8.3 Signed Angle Difference

$$
\Delta\theta = \text{normalize}(\theta_{target} - \theta_{current})
$$

Result is in $[-\pi, \pi]$:
- Positive: turn counter-clockwise
- Negative: turn clockwise

---

## 9. Quaternion ↔ Euler (PyBullet)

### 9.1 Quaternion to Euler

PyBullet uses $(x, y, z, w)$ quaternion convention.

```python
euler = pybullet.getEulerFromQuaternion(quaternion)
roll, pitch, yaw = euler[0], euler[1], euler[2]
```

For 2D navigation, we only use yaw ($\theta_z$).

### 9.2 Euler to Quaternion

```python
quaternion = pybullet.getQuaternionFromEuler([roll, pitch, yaw])
```

For 2D: `[0, 0, yaw]` → quaternion

---

## 10. Transformation Matrices

### 10.1 2D Rigid Transform

$$
T = \begin{pmatrix} \cos\theta & -\sin\theta & t_x \\ \sin\theta & \cos\theta & t_y \\ 0 & 0 & 1 \end{pmatrix}
$$

### 10.2 Apply Transform

$$
\begin{pmatrix} x' \\ y' \\ 1 \end{pmatrix} = T \cdot \begin{pmatrix} x \\ y \\ 1 \end{pmatrix}
$$

### 10.3 Inverse Transform

$$
T^{-1} = \begin{pmatrix} \cos\theta & \sin\theta & -t_x\cos\theta - t_y\sin\theta \\ -\sin\theta & \cos\theta & t_x\sin\theta - t_y\cos\theta \\ 0 & 0 & 1 \end{pmatrix}
$$

### 10.4 Compose Transforms

$$
T_{total} = T_2 \cdot T_1
$$

Apply $T_1$ first, then $T_2$.

---

## 11. Common Conversions Summary

| From | To | Formula |
|------|-----|---------|
| Grid $(g_x, g_y)$ | Meters | $(g_x \cdot C + C/2, g_y \cdot C + C/2)$ |
| Meters $(x, y)$ | Grid | $(\lfloor x/C \rfloor, \lfloor y/C \rfloor)$ |
| Heading | Yaw | `ORIENTATION_ANGLES[heading]` |
| Yaw | Heading | Quadrant check |
| Robot-frame angle | World angle | $\alpha + \theta_{robot}$ |
| Internal pos | World pos | $pos - offset$ |
| Degrees | Radians | $\theta \cdot \pi / 180$ |
| Radians | Degrees | $\theta \cdot 180 / \pi$ |

