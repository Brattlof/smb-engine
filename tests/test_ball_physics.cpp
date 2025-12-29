#include "physics/ball_physics.hpp"
#include "physics/ball_state.hpp"
#include "physics/physics_constants.hpp"

#include <cmath>

#include "test_framework.hpp"

// =============================================================================
// Ball Physics Tests
// =============================================================================
// Verify physics behavior matches SMB decomp
// =============================================================================

using namespace smb::physics;

// =============================================================================
// Gravity Tests
// =============================================================================

TEST_CASE("BallPhysics", "Gravity applies downward on level surface") {
    BallPhysics physics;
    BallState state;
    WorldTilt tilt;  // Level (no tilt)

    // Start airborne with zero velocity
    state.clearFlag(BallFlags::GROUNDED);
    state.setFlag(BallFlags::AIRBORNE);
    state.velocity = Vec3::zero();

    // Apply one physics tick of gravity
    physics.applyGravity(state, tilt, static_cast<float>(TIMESTEP));

    // Velocity should increase downward
    ASSERT_TRUE(state.velocity.y < 0.0f);
    ASSERT_NEAR(state.velocity.x, 0.0f, 0.0001f);
    ASSERT_NEAR(state.velocity.z, 0.0f, 0.0001f);
}

TEST_CASE("BallPhysics", "Ball falls at correct rate") {
    BallPhysics physics;
    BallState state;
    WorldTilt tilt;

    state.clearFlag(BallFlags::GROUNDED);
    state.setFlag(BallFlags::AIRBORNE);
    state.velocity = Vec3::zero();

    // Run 60 physics ticks (1 second)
    for (int i = 0; i < 60; i++) {
        physics.applyGravity(state, tilt, static_cast<float>(TIMESTEP));
    }

    // After 1 second, velocity should be approximately -BALL_ACCELERATION
    // v = a * t = 0.588 * 1 = 0.588 units/s (but applied to y component)
    // Actually with tilted gravity math, we need to reconsider
    // For level surface, gravity is straight down at full strength
    float expectedVelocity = -BALL_ACCELERATION * 1.0f;
    ASSERT_NEAR(state.velocity.y, expectedVelocity, 0.1f);
}

TEST_CASE("BallPhysics", "Tilted world changes effective gravity direction") {
    BallPhysics physics;
    WorldTilt tilt;

    // Tilt world to the right
    tilt.tiltZ = MAX_TILT_RADIANS;

    Vec3 gravDir = physics.getEffectiveGravityDirection(tilt);

    // Gravity should now have a positive X component (pulling right)
    ASSERT_TRUE(gravDir.x > 0.0f);
    // Should still have downward component
    ASSERT_TRUE(gravDir.y < 0.0f);
}

TEST_CASE("BallPhysics", "No acceleration when NO_ACCEL flag is set") {
    BallPhysics physics;
    BallState state;
    WorldTilt tilt;

    state.setFlag(BallFlags::NO_ACCEL);
    state.velocity = Vec3::zero();

    physics.applyGravity(state, tilt, static_cast<float>(TIMESTEP));

    // Velocity should not change
    ASSERT_TRUE(state.velocity.isNearZero());
}

TEST_CASE("BallPhysics", "Reverse acceleration flips direction") {
    BallPhysics physics;
    BallState state;
    WorldTilt tilt;

    state.clearFlag(BallFlags::GROUNDED);
    state.setFlag(BallFlags::AIRBORNE);
    state.setFlag(BallFlags::REVERSE_ACCEL);
    state.velocity = Vec3::zero();

    physics.applyGravity(state, tilt, static_cast<float>(TIMESTEP));

    // Velocity should increase UPWARD (reversed)
    ASSERT_TRUE(state.velocity.y > 0.0f);
}

// =============================================================================
// Friction Tests
// =============================================================================

TEST_CASE("BallPhysics", "Friction decelerates ball on flat surface") {
    BallPhysics physics;
    BallState state;

    state.setFlag(BallFlags::GROUNDED);
    state.velocity = Vec3(1.0f, 0.0f, 0.0f);  // Moving in X direction

    float initialSpeed = state.getSpeed();

    // Apply friction for one tick
    physics.applyFriction(state, static_cast<float>(TIMESTEP));

    float newSpeed = state.getSpeed();

    // Speed should decrease
    ASSERT_TRUE(newSpeed < initialSpeed);
}

TEST_CASE("BallPhysics", "Ball comes to rest from known velocity") {
    BallPhysics physics;
    BallState state;

    state.setFlag(BallFlags::GROUNDED);
    state.velocity = Vec3(0.5f, 0.0f, 0.0f);  // Moderate speed

    // Apply friction for many ticks until stopped
    for (int i = 0; i < 1000; i++) {
        physics.applyFriction(state, static_cast<float>(TIMESTEP));
        if (state.isStationary()) {
            break;
        }
    }

    // Should have come to rest
    ASSERT_TRUE(state.isStationary());
    ASSERT_NEAR(state.velocity.x, 0.0f, 0.0001f);
    ASSERT_NEAR(state.velocity.z, 0.0f, 0.0001f);
}

TEST_CASE("BallPhysics", "Friction applies correct decay rate") {
    BallPhysics physics;
    BallState state;

    state.setFlag(BallFlags::GROUNDED);
    state.velocity = Vec3(1.0f, 0.0f, 0.0f);

    physics.applyFriction(state, static_cast<float>(TIMESTEP));

    // After one frame: velocity *= (1 - 0.01) = 0.99
    ASSERT_NEAR(state.velocity.x, 0.99f, 0.001f);
}

TEST_CASE("BallPhysics", "Friction does not apply when airborne") {
    BallPhysics physics;
    BallState state;

    state.clearFlag(BallFlags::GROUNDED);
    state.setFlag(BallFlags::AIRBORNE);
    state.velocity = Vec3(1.0f, 0.0f, 0.0f);

    physics.applyFriction(state, static_cast<float>(TIMESTEP));

    // Velocity should not change
    ASSERT_NEAR(state.velocity.x, 1.0f, 0.0001f);
}

// =============================================================================
// Velocity Integration Tests
// =============================================================================

TEST_CASE("BallPhysics", "Ball moves expected distance") {
    BallPhysics physics;
    BallState state;

    state.position = Vec3::zero();
    state.velocity = Vec3(1.0f, 0.0f, 0.0f);  // 1 unit/second in X

    // One physics tick
    physics.integrateVelocity(state, static_cast<float>(TIMESTEP));

    // Position should advance by velocity * dt
    float expectedMove = 1.0f * static_cast<float>(TIMESTEP);
    ASSERT_NEAR(state.position.x, expectedMove, 0.0001f);
}

TEST_CASE("BallPhysics", "Position updates correctly over multiple ticks") {
    BallPhysics physics;
    BallState state;

    state.position = Vec3::zero();
    state.velocity = Vec3(1.0f, 0.0f, 0.0f);

    // 60 ticks = 1 second
    for (int i = 0; i < 60; i++) {
        physics.integrateVelocity(state, static_cast<float>(TIMESTEP));
    }

    // Should have moved approximately 1 unit
    ASSERT_NEAR(state.position.x, 1.0f, 0.01f);
}

// =============================================================================
// Rotation Tests
// =============================================================================

TEST_CASE("BallPhysics", "Rotation direction matches velocity") {
    BallPhysics physics;
    BallState state;

    state.setFlag(BallFlags::GROUNDED);
    state.velocity = Vec3(1.0f, 0.0f, 0.0f);  // Moving in +X direction
    state.rotation = Quaternion::identity();

    physics.updateRotation(state, static_cast<float>(TIMESTEP));

    // Angular velocity should be around Z axis (perpendicular to X motion and Y up)
    // For rolling in +X, ball rotates around -Z axis
    ASSERT_TRUE(std::fabs(state.angularVelocity.z) > 0.0f ||
                std::fabs(state.angularVelocity.x) > 0.0f);
}

TEST_CASE("BallPhysics", "Stationary ball does not rotate") {
    BallPhysics physics;
    BallState state;

    state.setFlag(BallFlags::GROUNDED);
    state.velocity = Vec3::zero();
    state.angularVelocity = Vec3::zero();
    state.rotation = Quaternion::identity();

    physics.updateRotation(state, static_cast<float>(TIMESTEP));

    // Angular velocity should remain zero
    ASSERT_TRUE(state.angularVelocity.isNearZero());
}

TEST_CASE("BallPhysics", "Angular damping reduces spin when airborne") {
    BallPhysics physics;
    BallState state;

    state.clearFlag(BallFlags::GROUNDED);
    state.setFlag(BallFlags::AIRBORNE);
    state.angularVelocity = Vec3(0.0f, 1.0f, 0.0f);  // Spinning around Y

    float initialSpeed = state.getAngularSpeed();

    physics.applyAngularDamping(state);

    float newSpeed = state.getAngularSpeed();

    // Angular speed should decrease
    ASSERT_TRUE(newSpeed < initialSpeed);
    ASSERT_NEAR(newSpeed, initialSpeed * ANGULAR_DAMPING_AIRBORNE, 0.001f);
}

// =============================================================================
// Full Update Tests
// =============================================================================

TEST_CASE("BallPhysics", "Full update increments tick count") {
    BallPhysics physics;
    BallState state;
    WorldTilt tilt;

    uint64_t initialTicks = state.tickCount;

    physics.update(state, tilt, static_cast<float>(TIMESTEP));

    ASSERT_EQ(state.tickCount, initialTicks + 1);
}

TEST_CASE("BallPhysics", "Frozen ball does not update") {
    BallPhysics physics;
    BallState state;
    WorldTilt tilt;

    state.setFlag(BallFlags::FROZEN);
    state.position = Vec3(1.0f, 1.0f, 1.0f);
    state.velocity = Vec3(1.0f, 0.0f, 0.0f);
    uint64_t initialTicks = state.tickCount;

    physics.update(state, tilt, static_cast<float>(TIMESTEP));

    // Nothing should change
    ASSERT_EQ(state.tickCount, initialTicks);
    ASSERT_NEAR(state.position.x, 1.0f, 0.0001f);
}

TEST_CASE("BallPhysics", "Input produces expected acceleration") {
    BallPhysics physics;
    BallState state;
    WorldTilt tilt;

    // Tilt the world forward (should make ball roll in -Z direction)
    tilt.setTargetFromInput(0.0f, 1.0f);  // Full forward input
    tilt.tiltX = tilt.targetTiltX;        // Skip smoothing for test
    tilt.tiltZ = tilt.targetTiltZ;

    state.setFlag(BallFlags::GROUNDED);
    state.velocity = Vec3::zero();

    // Apply several physics ticks
    for (int i = 0; i < 30; i++) {
        physics.applyGravity(state, tilt, static_cast<float>(TIMESTEP));
    }

    // Ball should be moving in -Z direction (forward tilt = ball rolls away)
    ASSERT_TRUE(state.velocity.z < 0.0f);
}

// =============================================================================
// WorldTilt Tests
// =============================================================================

TEST_CASE("WorldTilt", "Smoothing approaches target") {
    WorldTilt tilt;

    tilt.setTargetFromInput(1.0f, 0.0f);  // Full right input

    // Apply several smoothing updates
    for (int i = 0; i < 100; i++) {
        tilt.update();
    }

    // Should be very close to target
    ASSERT_NEAR(tilt.tiltZ, tilt.targetTiltZ, 0.001f);
}

TEST_CASE("WorldTilt", "Input clamping works") {
    WorldTilt tilt;

    // Try to set input beyond limits
    tilt.setTargetFromInput(2.0f, 2.0f);

    // Target should be clamped to max tilt
    ASSERT_TRUE(std::fabs(tilt.targetTiltZ) <= MAX_TILT_RADIANS + EPSILON);
    ASSERT_TRUE(std::fabs(tilt.targetTiltX) <= MAX_TILT_RADIANS + EPSILON);
}

TEST_CASE("WorldTilt", "Reset returns to level") {
    WorldTilt tilt;

    tilt.setTargetFromInput(1.0f, 1.0f);
    tilt.tiltX = 0.5f;
    tilt.tiltZ = 0.5f;

    tilt.reset();

    ASSERT_NEAR(tilt.tiltX, 0.0f, 0.0001f);
    ASSERT_NEAR(tilt.tiltZ, 0.0f, 0.0001f);
    ASSERT_NEAR(tilt.targetTiltX, 0.0f, 0.0001f);
    ASSERT_NEAR(tilt.targetTiltZ, 0.0f, 0.0001f);
}
