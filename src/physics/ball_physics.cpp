#include "physics/ball_physics.hpp"

#include <cmath>

// =============================================================================
// Ball Physics Implementation
// =============================================================================
// All physics formulas reference the SMB decompilation:
// https://github.com/camthesaxman/smb-decomp
//
// Key insight from the original game:
// - The world tilts, not the camera
// - Gravity is always "down" in world space (-Y)
// - When the world tilts, the floor tilts relative to gravity
// - This makes the ball roll "downhill" on the tilted floor
//
// The effect is that gravity appears to pull the ball in different directions
// relative to the floor surface, even though gravity itself is constant.
// =============================================================================

namespace smb {
namespace physics {

BallPhysics::BallPhysics() {
    // Nothing to initialize - all state is in BallState
}

// =============================================================================
// Main Update
// =============================================================================

void BallPhysics::update(BallState& state, const WorldTilt& tilt, float dt) {
    // Skip physics for frozen ball (cutscenes, etc.)
    if (state.isFrozen()) {
        return;
    }

    // Update tick counter for determinism verification
    state.tickCount++;

    // Apply forces based on grounded/airborne state
    if (state.isGrounded()) {
        // On ground: apply gravity projected onto surface + friction
        applyGravity(state, tilt, dt);
        applyFriction(state, dt);

        // Update rotation based on rolling
        updateRotation(state, dt);
    } else {
        // In air: apply full gravity, no friction, damp angular velocity
        applyGravity(state, tilt, dt);
        applyAngularDamping(state);

        // Still update rotation but without ground contact effects
        updateRotation(state, dt);
    }

    // Integrate velocity to update position
    integrateVelocity(state, dt);
}

// =============================================================================
// Gravity Application
// =============================================================================
// Source: ball.c - gravity and acceleration handling
// Source: world.c - gravity direction (0, -1, 0)
//
// When grounded, gravity accelerates the ball along the tilted surface.
// When airborne, gravity accelerates the ball straight down.

void BallPhysics::applyGravity(BallState& state, const WorldTilt& tilt, float dt) {
    // Check for special flags that modify acceleration
    if (hasFlag(state.flags, BallFlags::NO_ACCEL)) {
        return;  // Acceleration disabled
    }

    // Get effective gravity direction based on tilt
    Vec3 gravDir = computeTiltedGravity(tilt);

    // Calculate acceleration magnitude
    // Source: ball.c ballPhysicsParams[0].unk8 = 0.009799992f per frame
    // We convert to per-second and multiply by dt
    float accelMagnitude = BALL_ACCELERATION * dt;

    // Check for reverse acceleration flag
    // Source: ball.c - BALL_FLAG_09 reverses acceleration
    if (hasFlag(state.flags, BallFlags::REVERSE_ACCEL)) {
        accelMagnitude = -accelMagnitude;
    }

    // Apply acceleration in gravity direction
    state.velocity += gravDir * accelMagnitude;
}

// =============================================================================
// Friction
// =============================================================================
// Source: ball.c - friction = 0.01f
//
// Friction is applied as a velocity multiplier: velocity *= (1 - friction)
// This happens per frame in the original, so we need to account for dt.
//
// The original applies friction unconditionally when grounded.
// Friction brings the ball to rest over time.

void BallPhysics::applyFriction(BallState& state, float dt) {
    (void)dt;  // Fixed timestep, friction is per-frame

    // Only apply friction when grounded
    if (!state.isGrounded()) {
        return;
    }

    // Get horizontal velocity (X and Z components)
    // In SMB, friction primarily affects horizontal motion on the floor
    float horizontalSpeed =
        std::sqrt(state.velocity.x * state.velocity.x + state.velocity.z * state.velocity.z);

    if (horizontalSpeed < VELOCITY_EPSILON) {
        // Already stopped - zero out any tiny residual velocity
        state.velocity.x = 0.0f;
        state.velocity.z = 0.0f;
        return;
    }

    // Apply friction decay
    // Original: velocity *= (1 - friction) per frame
    // For variable dt, we use: velocity *= (1 - friction)^(dt / frameTime)
    // Since we use fixed timestep at 60Hz, dt = 1/60, so this is just:
    // velocity *= (1 - friction)
    float frictionFactor = 1.0f - state.friction;

    state.velocity.x *= frictionFactor;
    state.velocity.z *= frictionFactor;

    // Check if we've stopped
    float newSpeed =
        std::sqrt(state.velocity.x * state.velocity.x + state.velocity.z * state.velocity.z);

    if (newSpeed < MIN_VELOCITY_THRESHOLD) {
        state.velocity.x = 0.0f;
        state.velocity.z = 0.0f;
    }
}

// =============================================================================
// Velocity Integration
// =============================================================================
// Simple Euler integration: position += velocity * dt
//
// This is straightforward physics. Collision detection/response happens
// separately and may modify position after integration.

void BallPhysics::integrateVelocity(BallState& state, float dt) {
    state.position += state.velocity * dt;
}

// =============================================================================
// Rotation Update
// =============================================================================
// Source: ball.c - rotation calculations
//
// When the ball rolls on the ground, it rotates around an axis perpendicular
// to its velocity direction. The rotation speed is proportional to velocity.
//
// Angular velocity = linear velocity / radius (for rolling without slipping)
// Rotation axis = up cross velocity (perpendicular to motion and up)

void BallPhysics::updateRotation(BallState& state, float dt) {
    // Get horizontal velocity for rolling
    Vec3 horizontalVel(state.velocity.x, 0.0f, state.velocity.z);
    float speed = horizontalVel.length();

    if (speed < MIN_VELOCITY_THRESHOLD) {
        // Not moving - no rotation update needed
        // Angular velocity decays via damping
        return;
    }

    // For a rolling ball: angular_velocity = linear_velocity / radius
    // The rotation axis is perpendicular to velocity and up vector
    Vec3 velocityDir = horizontalVel.normalized();
    Vec3 up = Vec3::up();

    // Rotation axis is perpendicular to both velocity and up
    // Cross product: up x velocity gives axis of rotation
    Vec3 rotationAxis = up.cross(velocityDir);

    // Angular speed for rolling without slipping
    float angularSpeed = speed / state.radius;

    // Update angular velocity
    // Source: ball.c - angular velocity is used for visual rotation
    state.angularVelocity = rotationAxis * angularSpeed;

    // Update quaternion rotation
    // Small angle approximation for incremental rotation
    float rotationAngle = angularSpeed * dt;

    if (rotationAngle > ANGLE_EPSILON) {
        Quaternion deltaRotation = Quaternion::fromAxisAngle(rotationAxis, rotationAngle);
        state.rotation = deltaRotation * state.rotation;

        // Renormalize to prevent drift
        state.rotation = state.rotation.normalized();
    }
}

// =============================================================================
// Angular Damping
// =============================================================================
// Source: ball.c - angularVelocity *= 0.95 when airborne
//
// When the ball is in the air, its angular velocity gradually decreases.

void BallPhysics::applyAngularDamping(BallState& state) {
    if (!state.isAirborne()) {
        return;
    }

    // Apply damping
    state.angularVelocity *= ANGULAR_DAMPING_AIRBORNE;

    // Zero out if very small
    if (state.angularVelocity.lengthSquared() < VELOCITY_EPSILON * VELOCITY_EPSILON) {
        state.angularVelocity = Vec3::zero();
    }
}

// =============================================================================
// Gravity Direction Calculation
// =============================================================================
// When the world tilts, the effective gravity direction changes relative to
// the floor. We compute this by rotating the base gravity vector by the
// inverse of the world tilt.
//
// In practice, this means:
// - If the world tilts forward, gravity pulls the ball forward on the floor
// - If the world tilts right, gravity pulls the ball right on the floor

Vec3 BallPhysics::getEffectiveGravityDirection(const WorldTilt& tilt) const {
    return computeTiltedGravity(tilt);
}

float BallPhysics::getAccelerationFromTilt(const WorldTilt& tilt) const {
    // Calculate total tilt angle
    float totalTilt = std::sqrt(tilt.tiltX * tilt.tiltX + tilt.tiltZ * tilt.tiltZ);

    // Acceleration is proportional to sin(tilt angle)
    // For small angles, sin(x) ≈ x, but we use full sine for accuracy
    return BALL_ACCELERATION * std::sin(totalTilt);
}

Vec3 BallPhysics::computeTiltedGravity(const WorldTilt& tilt) const {
    // Start with gravity pointing down
    Vec3 gravity(GRAVITY_DIR_X, GRAVITY_DIR_Y, GRAVITY_DIR_Z);

    // If no tilt, gravity is straight down
    if (std::fabs(tilt.tiltX) < ANGLE_EPSILON && std::fabs(tilt.tiltZ) < ANGLE_EPSILON) {
        return gravity;
    }

    // Apply rotation to gravity vector
    // Rotate around X axis first (forward/backward tilt)
    // Then rotate around Z axis (left/right tilt)
    //
    // This gives us the gravity direction in the tilted world frame.
    // Since we want gravity relative to the tilted floor, we actually
    // need to think about this as: how does the floor's normal change?
    //
    // When floor tilts:
    // - Tilt around Z (left/right): floor slopes in X direction
    // - Tilt around X (forward/back): floor slopes in Z direction
    //
    // The component of gravity parallel to the tilted floor is what
    // accelerates the ball.

    // Compute sin/cos for tilt angles
    float sinTiltX = std::sin(tilt.tiltX);
    float cosTiltX = std::cos(tilt.tiltX);
    float sinTiltZ = std::sin(tilt.tiltZ);
    float cosTiltZ = std::cos(tilt.tiltZ);

    // Rotation around X axis: affects Y and Z
    // [1    0        0   ] [x]   [x            ]
    // [0  cos(a)  -sin(a)] [y] = [y*cos - z*sin]
    // [0  sin(a)   cos(a)] [z]   [y*sin + z*cos]

    Vec3 afterX;
    afterX.x = gravity.x;
    afterX.y = gravity.y * cosTiltX - gravity.z * sinTiltX;
    afterX.z = gravity.y * sinTiltX + gravity.z * cosTiltX;

    // Rotation around Z axis: affects X and Y
    // [cos(a)  -sin(a)  0] [x]   [x*cos - y*sin]
    // [sin(a)   cos(a)  0] [y] = [x*sin + y*cos]
    // [0        0       1] [z]   [z            ]

    Vec3 afterZ;
    afterZ.x = afterX.x * cosTiltZ - afterX.y * sinTiltZ;
    afterZ.y = afterX.x * sinTiltZ + afterX.y * cosTiltZ;
    afterZ.z = afterX.z;

    return afterZ.normalized();
}

}  // namespace physics
}  // namespace smb
