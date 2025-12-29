#pragma once

#include <cmath>

// Pi constant - we define our own to avoid platform dependencies with M_PI
// MSVC requires _USE_MATH_DEFINES before cmath, which is fragile.
namespace smb {
namespace physics {
namespace constants {
constexpr double PI = 3.14159265358979323846;
}  // namespace constants
}  // namespace physics
}  // namespace smb

// =============================================================================
// SMB Physics Constants
// =============================================================================
// All values sourced from the Super Monkey Ball decompilation project:
// https://github.com/camthesaxman/smb-decomp
//
// The original game runs at 60 FPS with physics tightly coupled to frame rate.
// We match this with our fixed timestep of 1/60 seconds.
//
// Units:
//   - Distance: game units (1 unit ≈ 1 meter for our purposes)
//   - Time: seconds (converted from per-frame values)
//   - Angles: radians (converted from SMB's fixed-point 0x10000 = 360°)
//   - Mass: normalized (ball has unit mass)
// =============================================================================

namespace smb {
namespace physics {

// =============================================================================
// Timing Constants
// =============================================================================

// Physics runs at 60 Hz to match original SMB
// Source: Game runs at 60fps with physics tightly coupled to frame rate
constexpr double TIMESTEP = 1.0 / 60.0;  // seconds per physics tick
constexpr double TIMESTEP_SQ = TIMESTEP * TIMESTEP;

// =============================================================================
// Fixed-Point Angle Conversion
// =============================================================================
// SMB uses 16-bit integers for angles where 0x10000 = 360 degrees (full rotation)
// Source: mathutil.c - angle representation throughout the codebase

constexpr double SMB_ANGLE_TO_RADIANS = (2.0 * constants::PI) / 65536.0;  // 0x10000 = 2π
constexpr double RADIANS_TO_SMB_ANGLE = 65536.0 / (2.0 * constants::PI);
constexpr double DEGREES_TO_RADIANS = constants::PI / 180.0;
constexpr double RADIANS_TO_DEGREES = 180.0 / constants::PI;

// =============================================================================
// Ball Properties
// =============================================================================
// Source: ball.c - ballPhysicsParams[] array

// Ball radius in game units
// Source: ball.c ballPhysicsParams - radius = 0.5f for both ball types
constexpr float BALL_RADIUS = 0.5f;

// Ball mass - normalized to 1.0 for simplicity
// The original game doesn't expose mass directly, but we need it for physics
constexpr float BALL_MASS = 1.0f;

// =============================================================================
// Gravity and Acceleration
// =============================================================================
// In SMB, gravity is always "down" in world space (-Y). The world tilts to
// make the ball roll. The acceleration value is applied per frame when the
// ball is affected by gravity/tilt.
//
// Source: world.c - gravity vector is (0, -1, 0)
// Source: ball.c - physParams->unk8 values determine acceleration rate

// Gravity direction (normalized, world space)
// Source: world.c - default gravity vector = (0.0, -1.0, 0.0)
constexpr float GRAVITY_DIR_X = 0.0f;
constexpr float GRAVITY_DIR_Y = -1.0f;
constexpr float GRAVITY_DIR_Z = 0.0f;

// Ball acceleration magnitude (per frame in original, converted to per-second)
// Source: ball.c ballPhysicsParams[0].unk8 = 0.009799992f (per frame)
// The basketball type uses this value - it's the standard ball
// Per-second: 0.009799992 * 60 = 0.5879995 units/frame * 60 = 35.28 units/s²
constexpr float BALL_ACCEL_PER_FRAME = 0.009799992f;               // original per-frame value
constexpr float BALL_ACCELERATION = BALL_ACCEL_PER_FRAME * 60.0f;  // units/s²

// Alternate ball (black ball) has different acceleration
// Source: ball.c ballPhysicsParams[1].unk8 = 0.02177776f
constexpr float BALL_ACCEL_HEAVY_PER_FRAME = 0.02177776f;
constexpr float BALL_ACCELERATION_HEAVY = BALL_ACCEL_HEAVY_PER_FRAME * 60.0f;

// =============================================================================
// Friction
// =============================================================================
// Source: ball.c - friction values used for ground contact

// Default friction coefficient
// Source: ball.c - friction = 0.01f (most game modes)
// This is applied as velocity *= (1 - friction) per frame
constexpr float FRICTION_DEFAULT_PER_FRAME = 0.01f;

// For per-second calculations, we need the decay factor per second
// If velocity *= (1 - 0.01) per frame, then per second:
// velocity *= (1 - 0.01)^60 ≈ 0.547
// So effective friction per second is about 0.453
constexpr float FRICTION_DEFAULT = FRICTION_DEFAULT_PER_FRAME;  // keep original

// Mini-target mode uses lower friction
// Source: ball.c - friction = 0.005f (mini target)
constexpr float FRICTION_LOW_PER_FRAME = 0.005f;
constexpr float FRICTION_LOW = FRICTION_LOW_PER_FRAME;

// =============================================================================
// Collision Response
// =============================================================================
// Source: ball.c - collision handling code

// Restitution (bounciness) - how much velocity is retained after bounce
// Source: ball.c ballPhysicsParams[0].restitution = 0.5f (basketball)
constexpr float RESTITUTION_DEFAULT = 0.5f;

// Heavy ball (black ball) has lower restitution - it's "heavier"
// Source: ball.c ballPhysicsParams[1].restitution = 0.1f
constexpr float RESTITUTION_HEAVY = 0.1f;

// Velocity retention factor after collision (separate from restitution)
// Source: ball.c - collision response uses 0.92 factor
constexpr float COLLISION_VELOCITY_RETENTION = 0.92f;

// =============================================================================
// Rotation and Angular Motion
// =============================================================================
// Source: ball.c - rotation calculations

// Rotational damping when ball is airborne (not in contact with stage)
// Source: ball.c - angularVelocity *= 0.95 when airborne
constexpr float ANGULAR_DAMPING_AIRBORNE = 0.95f;

// Angular damping factor for torque calculations
// Source: ball.c - angular damping = 0.15f
constexpr float ANGULAR_DAMPING_FACTOR = 0.15f;

// Velocity damping when ball is in goal state
// Source: ball.c - velocity *= 0.95 in goal state
constexpr float GOAL_VELOCITY_DAMPING = 0.95f;

// Torque calculation uses this conversion from linear to angular
// Source: ball.c - (32768.0 / 3.141592) / (radius * radius)
// This converts linear velocity to angular velocity for rolling
constexpr float TORQUE_FACTOR =
    (32768.0f / static_cast<float>(constants::PI)) / (BALL_RADIUS * BALL_RADIUS);

// =============================================================================
// World Tilt
// =============================================================================
// Source: world.c - tilt limit and smoothing values

// Maximum tilt angle in degrees (standard gameplay)
// Source: world.c - max tilt = 23.0 degrees for most modes
constexpr float MAX_TILT_DEGREES = 23.0f;
constexpr float MAX_TILT_RADIANS = MAX_TILT_DEGREES * static_cast<float>(DEGREES_TO_RADIANS);

// Maximum tilt angle for mini fight mode
// Source: world.c - max tilt = 34.5 degrees for mini fight
constexpr float MAX_TILT_FIGHT_DEGREES = 34.5f;
constexpr float MAX_TILT_FIGHT_RADIANS =
    MAX_TILT_FIGHT_DEGREES * static_cast<float>(DEGREES_TO_RADIANS);

// Input to tilt scaling factor
// Source: world.c - stick input divided by 60.0 for normalization
constexpr float INPUT_TILT_SCALE = 1.0f / 60.0f;

// Tilt smoothing factor (how fast tilt responds to input)
// Source: world.c - 0.2 multiplier applied to rotation deltas
constexpr float TILT_SMOOTHING = 0.2f;

// Tilt damping factor (used in some modes)
// Source: world.c - Z-rotation multiplied by 0.8 each frame
constexpr float TILT_DAMPING = 0.8f;

// =============================================================================
// Speed Thresholds
// =============================================================================
// Source: ball.c - various speed detection thresholds

// Minimum velocity for movement detection (below this, ball is "stopped")
// Source: ball.c - threshold = 0.00027777777f
constexpr float MIN_VELOCITY_THRESHOLD = 0.00027777777f;

// Velocity threshold for roll sound/effects
// Source: ball.c - threshold = 0.23148148148f
constexpr float ROLL_SOUND_THRESHOLD = 0.23148148148f;

// Higher speed threshold (used for visual effects)
// Source: ball.c - threshold = 0.37037037037f
constexpr float HIGH_SPEED_THRESHOLD = 0.37037037037f;

// Critical collision speed (flags special collision handling)
// Source: ball.c - threshold = -0.15f (negative = impact speed)
constexpr float CRITICAL_IMPACT_SPEED = -0.15f;

// =============================================================================
// Numerical Tolerances
// =============================================================================
// These are our tolerances for floating-point comparisons, not from decomp.
// Needed for deterministic behavior across platforms.

// Epsilon for general floating-point comparisons
constexpr float EPSILON = 1e-6f;

// Epsilon for velocity comparisons (when to consider velocity as zero)
constexpr float VELOCITY_EPSILON = 1e-5f;

// Epsilon for position comparisons
constexpr float POSITION_EPSILON = 1e-5f;

// Epsilon for angle comparisons
constexpr float ANGLE_EPSILON = 1e-6f;

// =============================================================================
// Helper Functions
// =============================================================================

// Convert SMB fixed-point angle to radians
inline constexpr float smbAngleToRadians(int smbAngle) {
    return static_cast<float>(smbAngle) * static_cast<float>(SMB_ANGLE_TO_RADIANS);
}

// Convert radians to SMB fixed-point angle
inline constexpr int radiansToSmbAngle(float radians) {
    return static_cast<int>(radians * static_cast<float>(RADIANS_TO_SMB_ANGLE));
}

// Convert degrees to radians
inline constexpr float degreesToRadians(float degrees) {
    return degrees * static_cast<float>(DEGREES_TO_RADIANS);
}

// Convert radians to degrees
inline constexpr float radiansToDegrees(float radians) {
    return radians * static_cast<float>(RADIANS_TO_DEGREES);
}

// Check if a float is effectively zero
inline bool isNearZero(float value, float epsilon = EPSILON) {
    return value > -epsilon && value < epsilon;
}

// Clamp a value to a range
inline float clamp(float value, float minVal, float maxVal) {
    if (value < minVal)
        return minVal;
    if (value > maxVal)
        return maxVal;
    return value;
}

}  // namespace physics
}  // namespace smb
