# Coordinate System Documentation

This document defines all coordinate systems used in the SMB physics engine and how they relate to each other. Understanding these systems is critical for correct physics implementation.

---

## World Coordinate System

The world uses a **right-handed Y-up coordinate system**.

```
        +Y (Up)
         |
         |
         |_______ +X (Right)
        /
       /
      +Z (Forward/Toward Camera)
```

### Axis Definitions

| Axis | Direction | Range | Notes |
|------|-----------|-------|-------|
| +X | Right | Unbounded | Corresponds to tilting right |
| -X | Left | Unbounded | Corresponds to tilting left |
| +Y | Up | Unbounded | Opposite to gravity |
| -Y | Down | Unbounded | Direction of gravity |
| +Z | Forward (toward camera) | Unbounded | Corresponds to tilting forward |
| -Z | Backward (away from camera) | Unbounded | Corresponds to tilting backward |

### Origin

The world origin (0, 0, 0) is typically placed at the center of the level's starting platform.

### Units

| Quantity | Unit | Notes |
|----------|------|-------|
| Distance | Meters | 1 unit = 1 meter |
| Angle | Radians | Unless noted otherwise |
| Time | Seconds | Physics timestep is 1/60 seconds |
| Mass | Arbitrary | Ball has unit mass |

---

## Physics Coordinate System

Physics calculations use the same coordinate system as the world. Gravity always points in the **-Y direction** in world space.

### Gravity Vector

When the world is level (no tilt):
```
gravity = (0, -9.8, 0)  // m/s² in world space
```

When the world is tilted, the gravity vector is rotated by the inverse of the world tilt matrix. This makes the ball appear to roll "downhill" on the tilted surface.

### Ball Position and Velocity

| Property | Type | Description |
|----------|------|-------------|
| position | Vec3 | Center of ball in world space |
| velocity | Vec3 | Linear velocity in m/s |
| angularVelocity | Vec3 | Rotation axis * radians/second |

---

## Camera Coordinate System

The camera uses a **view space** with:
- **+X**: Right of screen
- **+Y**: Up on screen
- **-Z**: Into the screen (forward viewing direction)

This is the standard OpenGL view space convention.

### Camera Properties

| Property | Description |
|----------|-------------|
| Position | World-space position of camera eye |
| Target | World-space point camera looks at |
| Up | World-space up direction (typically (0, 1, 0)) |
| FOV | Vertical field of view in degrees |

### The Tilt Illusion

**Critical**: The camera does NOT tilt with input. Instead:

1. Input tilts the **world geometry** around the ball
2. Gravity remains world-relative (-Y)
3. Camera stays level, tracking the ball
4. The visual effect is of a tilting floor with a stable horizon

This matches the original Super Monkey Ball behavior.

---

## Screen Coordinate System

Screen space uses standard 2D coordinates:

```
(0,0) ─────────────────── (+X, 0)
  │                           │
  │                           │
  │        SCREEN             │
  │                           │
  │                           │
(0, +Y) ──────────────── (+X, +Y)
```

| Property | Description |
|----------|-------------|
| Origin | Top-left corner |
| +X | Right |
| +Y | Down |
| Units | Pixels |

---

## Input Mapping

Controller/keyboard input maps to world tilt:

| Input | World Tilt Axis | Result |
|-------|-----------------|--------|
| Left analog X+ | Rotation around Z | World tilts right |
| Left analog X- | Rotation around Z | World tilts left |
| Left analog Y+ | Rotation around X | World tilts forward |
| Left analog Y- | Rotation around X | World tilts backward |

Tilt values are normalized to [-1, 1] range by the input system.

---

## Transformation Pipeline

### World to View

```
viewMatrix = lookAt(camera.position, camera.target, camera.up)
viewPosition = viewMatrix * worldPosition
```

### View to Clip (Projection)

```
projMatrix = perspective(fov, aspect, near, far)
clipPosition = projMatrix * viewPosition
```

### Clip to NDC

```
ndcPosition = clipPosition.xyz / clipPosition.w
```

### NDC to Screen

```
screenX = (ndcPosition.x + 1) * 0.5 * screenWidth
screenY = (1 - ndcPosition.y) * 0.5 * screenHeight
```

---

## Matrix Conventions

### Multiplication Order

Matrices are applied **right-to-left**:

```
finalPosition = projection * view * model * localPosition
```

### Rotation Order

When composing rotations for world tilt:
1. First rotate around X axis (forward/backward tilt)
2. Then rotate around Z axis (left/right tilt)

This matches how a physical surface would tilt.

---

## Handedness Reference

| System | Handedness | Notes |
|--------|------------|-------|
| World | Right-handed | Y-up, Z-forward |
| View | Right-handed | -Z into screen |
| NDC | Left-handed | Z 0=near, 1=far |

---

## Debugging Coordinate Axes

In debug mode, coordinate axes are drawn at the origin:
- **Red arrow**: +X axis
- **Green arrow**: +Y axis
- **Blue arrow**: +Z axis

This follows the standard RGB = XYZ convention.
