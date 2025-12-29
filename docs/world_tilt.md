#World Tilt Documentation

This document describes the world tilt mechanics as implemented in Super Monkey Ball, based on analysis of the decompilation project.

---

## Overview

In Super Monkey Ball, the player does not directly control the ball. Instead, the player tilts the entire world, and gravity causes the ball to roll "downhill" on the tilted surface. The camera remains level, creating the illusion of a tilting floor beneath the ball.

---

## Reference Implementation

All values and behaviors are sourced from the SMB decompilation:
https://github.com/camthesaxman/smb-decomp

### Key Source Files

| File | Contents |
|------|----------|
| `src/world.c` | World tilt logic, tilt limits, gravity direction |
| `src/input.c` | Input normalization, stick mapping |
| `src/ball.c` | Ball physics response to gravity |

---

## Tilt Mechanics

### Coordinate System

The world uses a right-handed Y-up coordinate system:
- **+X**: Right
- **+Y**: Up
- **-Y**: Direction of gravity
- **+Z**: Toward camera (forward)
- **-Z**: Away from camera (backward)

See `/docs/coordinates.md` for complete coordinate system documentation.

### Tilt Axes

| Input Direction | Tilt Axis | Rotation | Effect |
|-----------------|-----------|----------|--------|
| Stick Right | Z axis | Positive rotation around Z | Right edge of floor goes down |
| Stick Left | Z axis | Negative rotation around Z | Left edge of floor goes down |
| Stick Forward | X axis | Positive rotation around X | Far edge goes down |
| Stick Backward | X axis | Negative rotation around X | Near edge goes down |

### Maximum Tilt Angles

Source: `world.c`

| Game Mode | Maximum Tilt |
|-----------|--------------|
| Standard gameplay | 23.0 degrees |
| Mini Fight mode | 34.5 degrees |

The maximum tilt is enforced as a limit on the target tilt angle. The actual tilt smoothly approaches this limit.

---

## Input Processing

### Input Normalization

Source: `world.c`, `input.c`

1. Raw analog stick input is read from controller
2. Input is clamped to range [-1.0, 1.0]
3. Input is divided by 60.0 for per-frame normalization
4. Result maps to target tilt angle

### Input to Tilt Mapping

```
target_tilt_x = input_y * MAX_TILT_RADIANS
target_tilt_z = input_x * MAX_TILT_RADIANS
```

Note: X input controls Z-axis rotation (left/right tilt), and Y input controls X-axis rotation (forward/backward tilt). This matches how a physical platform would behave.

---

## Tilt Smoothing

Source: `world.c` - smoothing factor 0.2

The world tilt does not instantly match the target tilt. Instead, it smoothly approaches the target:

```
current_tilt += (target_tilt - current_tilt) * TILT_SMOOTHING
```

Where `TILT_SMOOTHING = 0.2` (per frame at 60 FPS).

This creates a responsive but not instant feel to the controls.

### Damping

In certain modes, additional damping is applied:

Source: `world.c` - damping factor 0.8

```
current_tilt *= TILT_DAMPING
```

This causes the tilt to gradually return to level when input is released.

---

## Gravity Transformation

### Constant Gravity Direction

Gravity always points in the -Y direction in world space:

```
gravity_direction = (0, -1, 0)
```

Source: `world.c` - gravity vector definition

### Effective Gravity with Tilt

When the world tilts, the floor surface tilts relative to gravity. The component of gravity parallel to the tilted floor surface causes the ball to accelerate.

The transformation is computed by rotating the gravity vector by the inverse of the world tilt:

1. Start with gravity pointing down: (0, -1, 0)
2. Apply rotation around X axis by `tilt_x`
3. Apply rotation around Z axis by `tilt_z`
4. Normalize the result

The resulting vector represents the direction of acceleration the ball experiences on the tilted surface.

### Rotation Mathematics

For rotation around X axis by angle θ:
```
| 1    0        0    |
| 0  cos(θ)  -sin(θ) |
| 0  sin(θ)   cos(θ) |
```

For rotation around Z axis by angle φ:
```
| cos(φ)  -sin(φ)  0 |
| sin(φ)   cos(φ)  0 |
|   0        0     1 |
```

---

## Visual Representation

### What Tilts

The world tilt affects:
- Floor geometry (visual rotation)
- Collision surfaces (collision is computed in tilted space)
- Decorative elements (all stage geometry rotates together)

### What Does NOT Tilt

The following remain level:
- Camera orientation (horizon always level)
- HUD elements
- Skybox/background

This separation creates the characteristic SMB visual style where the player perceives a tilting floor beneath a stable camera.

---

## Implementation Details

### WorldTilt Struct

```cpp
struct WorldTilt {
    float tiltX;        // Current X rotation (radians)
    float tiltZ;        // Current Z rotation (radians)
    float targetTiltX;  // Target X rotation from input
    float targetTiltZ;  // Target Z rotation from input
};
```

### Key Constants

| Constant | Value | Source |
|----------|-------|--------|
| MAX_TILT_RADIANS | 0.4014 (23°) | world.c |
| TILT_SMOOTHING | 0.2 | world.c |
| TILT_DAMPING | 0.8 | world.c |
| INPUT_TILT_SCALE | 1/60 | world.c |

### Per-Frame Update

Each physics frame:

1. Read input state
2. Set target tilt from input: `setTargetFromInput(input_x, input_y)`
3. Smooth current tilt toward target: `update()`
4. Compute effective gravity: `computeTiltedGravity()`
5. Apply gravity to ball physics

---

## Determinism

The world tilt system is fully deterministic:
- Same input sequence produces same tilt sequence
- Tilt state is serializable for replay
- No random or time-dependent behavior

For replay systems, both the BallState and WorldTilt must be captured in snapshots.

---

## Testing

### Visual Verification

1. Push stick right → floor should tilt right, ball rolls right
2. Push stick forward → floor tips forward, ball rolls away from camera
3. Release stick → floor gradually returns to level
4. Tilt to maximum → ball accelerates at maximum rate

### Unit Tests

| Test | Verification |
|------|--------------|
| Tilt limits enforced | tilt never exceeds MAX_TILT_RADIANS |
| Smoothing approaches target | after many frames, current ≈ target |
| Input mapping correct | right input produces positive Z tilt |
| Gravity direction changes | effective gravity has horizontal component |

---

## Related Documentation

- `/docs/coordinates.md` - Coordinate system definitions
- `src/physics/ball_physics.hpp` - WorldTilt struct implementation
- `src/physics/physics_constants.hpp` - Tilt constants with decomp references
