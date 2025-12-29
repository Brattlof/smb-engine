#pragma once

#include "physics/ball_state.hpp"
#include "physics/physics_constants.hpp"
#include "physics/vec3.hpp"

// =============================================================================
// Ball Physics Simulation
// =============================================================================
// Handles all physics updates for the ball: gravity, friction, acceleration,
// velocity integration, and rotation.
//
// Physics are updated at a fixed timestep (60 Hz) matching the original SMB.
// All calculations are deterministic - same inputs produce same outputs.
//
// This module does NOT handle collision - that's done separately. This only
// handles the ball's response to gravity, friction, and input forces.
// =============================================================================

namespace smb {
namespace physics {

// =============================================================================
// World Tilt State
// =============================================================================
// Represents the current tilt of the world. In SMB, the world tilts based on
// input, and gravity remains constant (-Y). The ball rolls because the floor
// tilts under it.

struct WorldTilt {
    // Current tilt angles in radians
    // tiltX = rotation around X axis (forward/backward tilt)
    // tiltZ = rotation around Z axis (left/right tilt)
    float tiltX;
    float tiltZ;

    // Target tilt from input (before smoothing)
    float targetTiltX;
    float targetTiltZ;

    WorldTilt() : tiltX(0.0f), tiltZ(0.0f), targetTiltX(0.0f), targetTiltZ(0.0f) {}

    // Serialization support
    static constexpr size_t SERIALIZED_SIZE = sizeof(float) * 4;

    size_t serialize(void* buffer) const {
        float* ptr = static_cast<float*>(buffer);
        ptr[0] = tiltX;
        ptr[1] = tiltZ;
        ptr[2] = targetTiltX;
        ptr[3] = targetTiltZ;
        return SERIALIZED_SIZE;
    }

    size_t deserialize(const void* buffer) {
        const float* ptr = static_cast<const float*>(buffer);
        tiltX = ptr[0];
        tiltZ = ptr[1];
        targetTiltX = ptr[2];
        targetTiltZ = ptr[3];
        return SERIALIZED_SIZE;
    }

    bool approxEquals(const WorldTilt& other, float epsilon = EPSILON) const {
        return std::fabs(tiltX - other.tiltX) <= epsilon &&
               std::fabs(tiltZ - other.tiltZ) <= epsilon &&
               std::fabs(targetTiltX - other.targetTiltX) <= epsilon &&
               std::fabs(targetTiltZ - other.targetTiltZ) <= epsilon;
    }

    // Apply smoothing to approach target tilt
    // Source: world.c - 0.2 multiplier applied to rotation deltas
    void update() {
        tiltX += (targetTiltX - tiltX) * TILT_SMOOTHING;
        tiltZ += (targetTiltZ - tiltZ) * TILT_SMOOTHING;
    }

    // Set target tilt from normalized input (-1 to 1)
    // Input is clamped and scaled to max tilt angle
    void setTargetFromInput(float inputX, float inputY) {
        // Clamp input to [-1, 1]
        inputX = clamp(inputX, -1.0f, 1.0f);
        inputY = clamp(inputY, -1.0f, 1.0f);

        // Map to tilt angles
        // X input tilts around Z axis (left/right): right input -> right tilt
        // Y input tilts around X axis (forward/backward): forward input -> front tips down
        //
        // In our coordinate system (+Z toward camera):
        // - Positive X rotation: floor tips so +Z edge goes up, -Z edge goes down
        // - For forward input (Y > 0), we want ball to roll toward -Z (away from camera)
        // - This means the -Z edge (far edge) should be lower
        // - This requires positive X rotation
        targetTiltZ = inputX * MAX_TILT_RADIANS;
        targetTiltX = inputY * MAX_TILT_RADIANS;
    }

    // Reset tilt to level
    void reset() {
        tiltX = 0.0f;
        tiltZ = 0.0f;
        targetTiltX = 0.0f;
        targetTiltZ = 0.0f;
    }
};

// =============================================================================
// Ball Physics Class
// =============================================================================

class BallPhysics {
public:
    BallPhysics();

    // =========================================================================
    // Main Update
    // =========================================================================

    // Update physics for one fixed timestep.
    // This is the main entry point - call once per physics tick.
    // dt should always be TIMESTEP (1/60 second).
    void update(BallState& state, const WorldTilt& tilt, float dt);

    // =========================================================================
    // Individual Physics Steps
    // =========================================================================
    // These are exposed for testing. Normally you just call update().

    // Apply gravity based on world tilt.
    // Gravity is always -Y in world space, but when the world tilts,
    // the effective gravity on the ball changes direction relative to the floor.
    // Source: world.c gravity vector, ball.c acceleration application
    void applyGravity(BallState& state, const WorldTilt& tilt, float dt);

    // Apply friction when ball is grounded.
    // Friction opposes motion and brings the ball to rest.
    // Source: ball.c friction = 0.01f applied as velocity *= (1 - friction)
    void applyFriction(BallState& state, float dt);

    // Integrate velocity to update position.
    // Simple Euler integration: position += velocity * dt
    void integrateVelocity(BallState& state, float dt);

    // Update ball rotation based on velocity.
    // Rolling ball rotates around axis perpendicular to velocity.
    // Source: ball.c rotation calculations with torque factor
    void updateRotation(BallState& state, float dt);

    // Apply angular damping when airborne.
    // Source: ball.c - angularVelocity *= 0.95 when not in contact
    void applyAngularDamping(BallState& state);

    // =========================================================================
    // State Queries
    // =========================================================================

    // Get the effective gravity direction based on world tilt.
    // This is the direction the ball will accelerate when falling/rolling.
    Vec3 getEffectiveGravityDirection(const WorldTilt& tilt) const;

    // Get acceleration magnitude based on tilt angle.
    // Steeper tilt = faster acceleration down the slope.
    float getAccelerationFromTilt(const WorldTilt& tilt) const;

private:
    // Helper: compute rotation matrix for world tilt
    // Returns the gravity direction in tilted world space
    Vec3 computeTiltedGravity(const WorldTilt& tilt) const;
};

}  // namespace physics
}  // namespace smb
