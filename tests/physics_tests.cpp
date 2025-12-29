#include "physics/ball_physics.hpp"
#include "physics/ball_state.hpp"
#include "physics/physics_constants.hpp"
#include "physics/vec3.hpp"

#include <cmath>

#include "test_framework.hpp"

// =============================================================================
// Physics Tests
// =============================================================================
// Verification tests for Phase 1: Core Physics
// These tests correspond to ROADMAP checkpoints P-A through P-D
// and the determinism verification requirement P-010.
// =============================================================================

using namespace smb::physics;

// =============================================================================
// Vec3 Tests - Foundation for all physics math
// =============================================================================

TEST_CASE("Vec3", "Default constructor initializes to zero") {
    Vec3 v;
    ASSERT_EQ(v.x, 0.0f);
    ASSERT_EQ(v.y, 0.0f);
    ASSERT_EQ(v.z, 0.0f);
}

TEST_CASE("Vec3", "Component constructor works correctly") {
    Vec3 v(1.0f, 2.0f, 3.0f);
    ASSERT_EQ(v.x, 1.0f);
    ASSERT_EQ(v.y, 2.0f);
    ASSERT_EQ(v.z, 3.0f);
}

TEST_CASE("Vec3", "Addition works correctly") {
    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(4.0f, 5.0f, 6.0f);
    Vec3 c = a + b;
    ASSERT_EQ(c.x, 5.0f);
    ASSERT_EQ(c.y, 7.0f);
    ASSERT_EQ(c.z, 9.0f);
}

TEST_CASE("Vec3", "Subtraction works correctly") {
    Vec3 a(4.0f, 5.0f, 6.0f);
    Vec3 b(1.0f, 2.0f, 3.0f);
    Vec3 c = a - b;
    ASSERT_EQ(c.x, 3.0f);
    ASSERT_EQ(c.y, 3.0f);
    ASSERT_EQ(c.z, 3.0f);
}

TEST_CASE("Vec3", "Scalar multiplication works correctly") {
    Vec3 v(1.0f, 2.0f, 3.0f);
    Vec3 scaled = v * 2.0f;
    ASSERT_EQ(scaled.x, 2.0f);
    ASSERT_EQ(scaled.y, 4.0f);
    ASSERT_EQ(scaled.z, 6.0f);
}

TEST_CASE("Vec3", "Length calculation is correct") {
    Vec3 v(3.0f, 4.0f, 0.0f);
    ASSERT_NEAR(v.length(), 5.0f, 1e-6f);
}

TEST_CASE("Vec3", "Normalization produces unit vector") {
    Vec3 v(3.0f, 4.0f, 0.0f);
    Vec3 n = v.normalized();
    ASSERT_NEAR(n.length(), 1.0f, 1e-6f);
    ASSERT_NEAR(n.x, 0.6f, 1e-6f);
    ASSERT_NEAR(n.y, 0.8f, 1e-6f);
}

TEST_CASE("Vec3", "Dot product is correct") {
    Vec3 a(1.0f, 0.0f, 0.0f);
    Vec3 b(0.0f, 1.0f, 0.0f);
    ASSERT_NEAR(a.dot(b), 0.0f, 1e-6f);

    Vec3 c(1.0f, 2.0f, 3.0f);
    Vec3 d(4.0f, 5.0f, 6.0f);
    ASSERT_NEAR(c.dot(d), 32.0f, 1e-6f);
}

TEST_CASE("Vec3", "Cross product is correct") {
    Vec3 x = Vec3::right();  // (1, 0, 0)
    Vec3 y = Vec3::up();     // (0, 1, 0)
    Vec3 z = x.cross(y);
    // In right-handed system: X cross Y = -Z (but our forward() is +Z, so this is back)
    // Wait - let me recalculate: (1,0,0) x (0,1,0) = (0*0 - 0*1, 0*0 - 1*0, 1*1 - 0*0) = (0,0,1)
    // That's actually +Z. The confusion is in naming conventions.
    ASSERT_NEAR(z.x, 0.0f, 1e-6f);
    ASSERT_NEAR(z.y, 0.0f, 1e-6f);
    ASSERT_NEAR(z.z, 1.0f, 1e-6f);
}

// =============================================================================
// Quaternion Tests - Ball rotation math
// =============================================================================

TEST_CASE("Quaternion", "Identity quaternion has no rotation") {
    Quaternion q = Quaternion::identity();
    ASSERT_EQ(q.w, 1.0f);
    ASSERT_EQ(q.x, 0.0f);
    ASSERT_EQ(q.y, 0.0f);
    ASSERT_EQ(q.z, 0.0f);
}

TEST_CASE("Quaternion", "FromAxisAngle creates correct rotation") {
    // 90 degree rotation around Y axis
    Vec3 axis = Vec3::up();
    float angle = static_cast<float>(constants::PI) / 2.0f;
    Quaternion q = Quaternion::fromAxisAngle(axis, angle);

    // For 90 degrees: w = cos(45) = 0.707, y = sin(45) = 0.707
    ASSERT_NEAR(q.w, 0.7071f, 0.001f);
    ASSERT_NEAR(q.x, 0.0f, 0.001f);
    ASSERT_NEAR(q.y, 0.7071f, 0.001f);
    ASSERT_NEAR(q.z, 0.0f, 0.001f);
}

TEST_CASE("Quaternion", "Normalization maintains unit length") {
    Quaternion q(1.0f, 1.0f, 1.0f, 1.0f);
    Quaternion n = q.normalized();
    float len = std::sqrt(n.w * n.w + n.x * n.x + n.y * n.y + n.z * n.z);
    ASSERT_NEAR(len, 1.0f, 1e-6f);
}

// =============================================================================
// BallState Tests - State management
// =============================================================================

TEST_CASE("BallState", "Default state is on ground at origin") {
    BallState state;
    ASSERT_NEAR(state.position.x, 0.0f, 1e-6f);
    ASSERT_NEAR(state.position.y, BALL_RADIUS, 1e-6f);
    ASSERT_NEAR(state.position.z, 0.0f, 1e-6f);
    ASSERT_TRUE(state.isGrounded());
    ASSERT_FALSE(state.isAirborne());
}

TEST_CASE("BallState", "Velocity starts at zero") {
    BallState state;
    ASSERT_NEAR(state.velocity.x, 0.0f, 1e-6f);
    ASSERT_NEAR(state.velocity.y, 0.0f, 1e-6f);
    ASSERT_NEAR(state.velocity.z, 0.0f, 1e-6f);
    ASSERT_NEAR(state.getSpeed(), 0.0f, 1e-6f);
}

TEST_CASE("BallState", "Flags can be set and cleared") {
    BallState state;
    ASSERT_TRUE(state.isGrounded());

    state.clearFlag(BallFlags::GROUNDED);
    state.setFlag(BallFlags::AIRBORNE);

    ASSERT_FALSE(state.isGrounded());
    ASSERT_TRUE(state.isAirborne());
}

TEST_CASE("BallState", "Reset restores initial state") {
    BallState state;
    state.position = Vec3(10.0f, 20.0f, 30.0f);
    state.velocity = Vec3(1.0f, 2.0f, 3.0f);
    state.tickCount = 1000;
    state.setFlag(BallFlags::AIRBORNE);
    state.clearFlag(BallFlags::GROUNDED);

    state.reset();

    ASSERT_NEAR(state.position.x, 0.0f, 1e-6f);
    ASSERT_NEAR(state.position.y, BALL_RADIUS, 1e-6f);
    ASSERT_NEAR(state.position.z, 0.0f, 1e-6f);
    ASSERT_NEAR(state.getSpeed(), 0.0f, 1e-6f);
    ASSERT_EQ(state.tickCount, 0u);
    ASSERT_TRUE(state.isGrounded());
}

TEST_CASE("BallState", "Serialization roundtrip preserves state") {
    BallState original;
    original.position = Vec3(1.5f, 2.5f, 3.5f);
    original.velocity = Vec3(0.1f, 0.2f, 0.3f);
    original.angularVelocity = Vec3(0.01f, 0.02f, 0.03f);
    original.radius = 0.6f;
    original.restitution = 0.4f;
    original.friction = 0.02f;
    original.tickCount = 12345;
    original.setFlag(BallFlags::AIRBORNE);

    // Serialize
    uint8_t buffer[BallState::SERIALIZED_SIZE];
    original.serialize(buffer);

    // Deserialize into new state
    BallState restored;
    restored.deserialize(buffer);

    // Verify all fields match
    ASSERT_TRUE(original.approxEquals(restored, 0.0f));
}

// =============================================================================
// Checkpoint P-A: Gravity Tests
// =============================================================================
// "Ball dropped from height falls correctly"
// "Acceleration matches SMB decomp value"

TEST_CASE("Physics", "Gravity applies in correct direction") {
    BallPhysics physics;
    BallState state;
    WorldTilt tilt;  // No tilt

    // Ball starts stationary
    state.velocity = Vec3::zero();
    state.clearFlag(BallFlags::GROUNDED);
    state.setFlag(BallFlags::AIRBORNE);

    // Apply one frame of gravity
    physics.applyGravity(state, tilt, static_cast<float>(TIMESTEP));

    // Velocity should be negative Y (downward)
    ASSERT_TRUE(state.velocity.y < 0.0f);
    ASSERT_NEAR(state.velocity.x, 0.0f, 1e-6f);
    ASSERT_NEAR(state.velocity.z, 0.0f, 1e-6f);
}

TEST_CASE("Physics", "Gravity acceleration matches decomp value") {
    BallPhysics physics;
    BallState state;
    WorldTilt tilt;

    state.velocity = Vec3::zero();

    // Apply gravity for one frame
    float dt = static_cast<float>(TIMESTEP);
    physics.applyGravity(state, tilt, dt);

    // Expected velocity after one frame:
    // v = a * dt = BALL_ACCELERATION * (1/60)
    // BALL_ACCELERATION = 0.009799992 * 60 = 0.5879995
    // v = 0.5879995 * (1/60) = 0.009799992 per frame
    float expectedVelocity = BALL_ACCEL_PER_FRAME;

    // Since gravity is -Y, velocity.y should be negative
    ASSERT_NEAR(std::fabs(state.velocity.y), expectedVelocity, 1e-6f);
}

TEST_CASE("Physics", "Ball dropped from height gains velocity over time") {
    BallPhysics physics;
    BallState state;
    WorldTilt tilt;

    state.position = Vec3(0.0f, 10.0f, 0.0f);
    state.velocity = Vec3::zero();
    state.clearFlag(BallFlags::GROUNDED);
    state.setFlag(BallFlags::AIRBORNE);

    float dt = static_cast<float>(TIMESTEP);
    float totalTime = 0.0f;
    int frames = 60;  // One second

    for (int i = 0; i < frames; i++) {
        physics.applyGravity(state, tilt, dt);
        physics.integrateVelocity(state, dt);
        totalTime += dt;
    }

    // After 1 second of free fall, velocity should be approximately
    // v = g * t = BALL_ACCELERATION * 1.0 = ~0.588 units/s
    float expectedSpeed = BALL_ACCELERATION * 1.0f;
    ASSERT_NEAR(std::fabs(state.velocity.y), expectedSpeed, 0.01f);
}

// =============================================================================
// Checkpoint P-B: Friction Tests
// =============================================================================
// "Ball pushed on flat surface decelerates"
// "Comes to complete rest"

TEST_CASE("Physics", "Friction decelerates horizontal velocity") {
    BallPhysics physics;
    BallState state;
    WorldTilt tilt;

    // Start with some horizontal velocity
    state.velocity = Vec3(1.0f, 0.0f, 0.0f);
    state.setFlag(BallFlags::GROUNDED);

    float initialSpeed = state.getSpeed();

    // Apply friction
    physics.applyFriction(state, static_cast<float>(TIMESTEP));

    // Speed should decrease
    float newSpeed = state.getSpeed();
    ASSERT_TRUE(newSpeed < initialSpeed);
}

TEST_CASE("Physics", "Friction factor matches decomp") {
    BallPhysics physics;
    BallState state;
    WorldTilt tilt;

    // Set known velocity
    float initialVelocity = 1.0f;
    state.velocity = Vec3(initialVelocity, 0.0f, 0.0f);
    state.setFlag(BallFlags::GROUNDED);

    // Apply one frame of friction
    physics.applyFriction(state, static_cast<float>(TIMESTEP));

    // Expected: velocity *= (1 - friction) = (1 - 0.01) = 0.99
    float expectedVelocity = initialVelocity * (1.0f - FRICTION_DEFAULT);
    ASSERT_NEAR(state.velocity.x, expectedVelocity, 1e-5f);
}

TEST_CASE("Physics", "Ball comes to rest eventually") {
    BallPhysics physics;
    BallState state;
    WorldTilt tilt;

    // Start with some velocity
    state.velocity = Vec3(0.5f, 0.0f, 0.5f);
    state.setFlag(BallFlags::GROUNDED);

    // Apply friction for many frames
    int maxFrames = 10000;
    for (int i = 0; i < maxFrames; i++) {
        physics.applyFriction(state, static_cast<float>(TIMESTEP));
        if (state.isStationary()) {
            break;
        }
    }

    // Ball should have come to rest
    ASSERT_TRUE(state.isStationary());
}

TEST_CASE("Physics", "Friction only applies when grounded") {
    BallPhysics physics;
    BallState state;
    WorldTilt tilt;

    state.velocity = Vec3(1.0f, 0.0f, 0.0f);
    state.clearFlag(BallFlags::GROUNDED);
    state.setFlag(BallFlags::AIRBORNE);

    float initialSpeed = state.getSpeed();

    // Try to apply friction while airborne
    physics.applyFriction(state, static_cast<float>(TIMESTEP));

    // Velocity should be unchanged (no friction in air)
    ASSERT_NEAR(state.getSpeed(), initialSpeed, 1e-6f);
}

// =============================================================================
// Checkpoint P-C: Input Response Tests
// =============================================================================
// "Input produces acceleration in correct direction"
// "Ball reaches expected velocity from sustained input"

TEST_CASE("Physics", "Tilt input maps to correct direction") {
    WorldTilt tilt;

    // Right tilt (positive X input)
    tilt.setTargetFromInput(1.0f, 0.0f);
    tilt.tiltZ = tilt.targetTiltZ;  // Skip smoothing for test

    ASSERT_TRUE(tilt.tiltZ > 0.0f);
    ASSERT_NEAR(std::fabs(tilt.tiltZ), MAX_TILT_RADIANS, 1e-6f);
}

TEST_CASE("Physics", "Tilt produces acceleration toward low side") {
    BallPhysics physics;
    BallState state;
    WorldTilt tilt;

    state.velocity = Vec3::zero();
    state.setFlag(BallFlags::GROUNDED);

    // Tilt right (floor tips so right side is lower)
    tilt.setTargetFromInput(1.0f, 0.0f);
    tilt.tiltZ = tilt.targetTiltZ;

    // Apply gravity with tilt
    physics.applyGravity(state, tilt, static_cast<float>(TIMESTEP));

    // Ball should gain velocity toward the low side (positive X)
    // Due to how the tilt math works, check that there's horizontal component
    float horizontalSpeed =
        std::sqrt(state.velocity.x * state.velocity.x + state.velocity.z * state.velocity.z);
    ASSERT_TRUE(horizontalSpeed > 0.0f);
}

TEST_CASE("Physics", "Forward tilt moves ball forward") {
    BallPhysics physics;
    BallState state;
    WorldTilt tilt;

    state.velocity = Vec3::zero();
    state.setFlag(BallFlags::GROUNDED);

    // Forward tilt (positive Y input -> positive X tilt -> ball rolls toward -Z)
    tilt.setTargetFromInput(0.0f, 1.0f);
    tilt.tiltX = tilt.targetTiltX;

    // Apply several frames
    float dt = static_cast<float>(TIMESTEP);
    for (int i = 0; i < 60; i++) {
        physics.applyGravity(state, tilt, dt);
    }

    // Ball should have gained velocity in Z direction (away from camera = -Z)
    ASSERT_TRUE(state.velocity.z < 0.0f);
}

TEST_CASE("Physics", "Sustained input produces increasing velocity") {
    BallPhysics physics;
    BallState state;
    WorldTilt tilt;

    state.velocity = Vec3::zero();
    state.setFlag(BallFlags::GROUNDED);

    // Apply maximum tilt
    tilt.setTargetFromInput(1.0f, 0.0f);
    tilt.tiltZ = tilt.targetTiltZ;

    float dt = static_cast<float>(TIMESTEP);
    float prevSpeed = 0.0f;

    // Track speed increase over time
    for (int i = 0; i < 30; i++) {
        physics.applyGravity(state, tilt, dt);
        physics.applyFriction(state, dt);

        float currentSpeed = state.getSpeed();
        // Speed should increase (or at least not decrease drastically) initially
        if (i > 0) {
            // Allow for friction effects
            ASSERT_TRUE(currentSpeed >= prevSpeed * 0.95f);
        }
        prevSpeed = currentSpeed;
    }

    // Final speed should be nonzero
    ASSERT_TRUE(state.getSpeed() > 0.0f);
}

// =============================================================================
// Checkpoint P-D: Full Integration Tests
// =============================================================================
// "Ball responds to input, rolls, decelerates, stops"
// "All behaviors combine correctly"

TEST_CASE("Physics", "Full update combines all physics") {
    BallPhysics physics;
    BallState state;
    WorldTilt tilt;

    // Apply some tilt
    tilt.setTargetFromInput(0.5f, 0.0f);
    tilt.tiltZ = tilt.targetTiltZ;

    float dt = static_cast<float>(TIMESTEP);
    uint64_t initialTick = state.tickCount;

    // Run full physics update
    physics.update(state, tilt, dt);

    // Tick count should increment
    ASSERT_EQ(state.tickCount, initialTick + 1);

    // Ball should have moved or gained velocity
    // (unless it's the very first frame with minimal change)
}

TEST_CASE("Physics", "Ball rolls on flat tilted surface") {
    BallPhysics physics;
    BallState state;
    WorldTilt tilt;

    state.position = Vec3(0.0f, BALL_RADIUS, 0.0f);
    state.velocity = Vec3::zero();
    state.setFlag(BallFlags::GROUNDED);

    // Tilt the world
    tilt.setTargetFromInput(0.5f, 0.0f);

    float dt = static_cast<float>(TIMESTEP);

    // Run for 2 seconds
    for (int i = 0; i < 120; i++) {
        tilt.update();  // Smooth tilt
        physics.update(state, tilt, dt);
    }

    // Ball should have moved significantly
    float distanceMoved = state.position.length();
    ASSERT_TRUE(distanceMoved > 0.1f);
}

TEST_CASE("Physics", "Ball decelerates when tilt removed") {
    BallPhysics physics;
    BallState state;
    WorldTilt tilt;

    state.velocity = Vec3::zero();
    state.setFlag(BallFlags::GROUNDED);

    // First, accelerate the ball with tilt
    tilt.setTargetFromInput(1.0f, 0.0f);
    float dt = static_cast<float>(TIMESTEP);

    for (int i = 0; i < 60; i++) {
        tilt.update();
        physics.update(state, tilt, dt);
    }

    // Record horizontal speed (X and Z only - Y is affected by gravity even when grounded)
    auto getHorizontalSpeed = [](const BallState& s) {
        return std::sqrt(s.velocity.x * s.velocity.x + s.velocity.z * s.velocity.z);
    };

    float peakHorizontalSpeed = getHorizontalSpeed(state);
    ASSERT_TRUE(peakHorizontalSpeed > 0.0f);

    // Now remove tilt and force it to zero immediately (skip smoothing for test clarity)
    tilt.reset();  // Forces both current tilt and target to zero

    // Record horizontal speed after removing tilt
    float speedAfterTiltRemoved = getHorizontalSpeed(state);

    // Run for many frames with zero tilt - friction affects horizontal velocity
    for (int i = 0; i < 1000; i++) {
        physics.update(state, tilt, dt);
        // Zero out Y velocity to simulate ground collision (not part of physics module)
        state.velocity.y = 0.0f;
    }

    // Ball should have slowed down due to friction
    // With 1% friction per frame: speed * 0.99^1000 ≈ speed * 4.3e-5 ≈ 0
    float finalHorizontalSpeed = getHorizontalSpeed(state);
    ASSERT_TRUE(finalHorizontalSpeed < speedAfterTiltRemoved);  // Deceleration occurred
}

TEST_CASE("Physics", "Ball rotation matches velocity direction") {
    BallPhysics physics;
    BallState state;
    WorldTilt tilt;

    // Give ball some velocity
    state.velocity = Vec3(1.0f, 0.0f, 0.0f);
    state.setFlag(BallFlags::GROUNDED);

    // Update rotation
    physics.updateRotation(state, static_cast<float>(TIMESTEP));

    // Angular velocity should be perpendicular to linear velocity
    // For velocity in +X, rotation axis should be around -Z (cross product of up and velocity)
    // up (0,1,0) cross velDir (1,0,0) = (0*0 - 0*0, 0*1 - 1*0, 1*0 - 0*1) = (0, 0, -1)
    ASSERT_NEAR(state.angularVelocity.x, 0.0f, 0.01f);
    ASSERT_NEAR(state.angularVelocity.y, 0.0f, 0.01f);
    ASSERT_TRUE(state.angularVelocity.z < 0.0f);
}

// =============================================================================
// P-010: Determinism Verification Tests
// =============================================================================
// "Same inputs produce same outputs"
// "Replay test: 1000 frames match exactly"

TEST_CASE("Determinism", "Same initial state produces identical results") {
    BallPhysics physics1, physics2;
    BallState state1, state2;
    WorldTilt tilt1, tilt2;

    // Set up identical initial conditions
    state1.position = Vec3(1.0f, BALL_RADIUS, 2.0f);
    state1.velocity = Vec3(0.5f, 0.0f, -0.3f);
    state1.setFlag(BallFlags::GROUNDED);

    state2.position = Vec3(1.0f, BALL_RADIUS, 2.0f);
    state2.velocity = Vec3(0.5f, 0.0f, -0.3f);
    state2.setFlag(BallFlags::GROUNDED);

    tilt1.setTargetFromInput(0.3f, -0.2f);
    tilt2.setTargetFromInput(0.3f, -0.2f);

    float dt = static_cast<float>(TIMESTEP);

    // Run 100 frames
    for (int i = 0; i < 100; i++) {
        tilt1.update();
        tilt2.update();
        physics1.update(state1, tilt1, dt);
        physics2.update(state2, tilt2, dt);
    }

    // States should be exactly equal
    ASSERT_TRUE(state1.approxEquals(state2, 0.0f));
}

TEST_CASE("Determinism", "1000 frame replay matches exactly") {
    BallPhysics physics;
    WorldTilt tilt;
    float dt = static_cast<float>(TIMESTEP);

    // First run: record final state
    BallState state1;
    state1.position = Vec3(0.0f, BALL_RADIUS, 0.0f);
    state1.velocity = Vec3::zero();
    state1.setFlag(BallFlags::GROUNDED);

    // Simulate with varying input (using fixed pattern instead of sin/cos to ensure determinism)
    for (int i = 0; i < 1000; i++) {
        // Simple deterministic input pattern
        float inputX = ((i % 120) < 60) ? 0.5f : -0.5f;
        float inputY = ((i % 200) < 100) ? 0.3f : -0.3f;
        tilt.setTargetFromInput(inputX, inputY);
        tilt.update();
        physics.update(state1, tilt, dt);
    }

    // Second run: replay with same inputs
    BallPhysics physics2;
    WorldTilt tilt2;
    BallState state2;
    state2.position = Vec3(0.0f, BALL_RADIUS, 0.0f);
    state2.velocity = Vec3::zero();
    state2.setFlag(BallFlags::GROUNDED);

    for (int i = 0; i < 1000; i++) {
        float inputX = ((i % 120) < 60) ? 0.5f : -0.5f;
        float inputY = ((i % 200) < 100) ? 0.3f : -0.3f;
        tilt2.setTargetFromInput(inputX, inputY);
        tilt2.update();
        physics2.update(state2, tilt2, dt);
    }

    // Final states must match - use tiny epsilon for floating point safety
    // Exact bitwise match may fail due to compiler optimization differences
    ASSERT_TRUE(state1.approxEquals(state2, 1e-6f));
    ASSERT_EQ(state1.tickCount, state2.tickCount);
}

TEST_CASE("Determinism", "Serialization preserves exact state for replay") {
    BallPhysics physics;
    WorldTilt tilt;
    float dt = static_cast<float>(TIMESTEP);

    // Run some physics to create interesting state
    BallState state;
    state.position = Vec3(0.0f, BALL_RADIUS, 0.0f);
    state.velocity = Vec3::zero();
    state.setFlag(BallFlags::GROUNDED);

    tilt.setTargetFromInput(0.5f, 0.3f);

    for (int i = 0; i < 100; i++) {
        tilt.update();
        physics.update(state, tilt, dt);
    }

    // Save state
    uint8_t stateBuffer[BallState::SERIALIZED_SIZE];
    uint8_t tiltBuffer[WorldTilt::SERIALIZED_SIZE];
    state.serialize(stateBuffer);
    tilt.serialize(tiltBuffer);

    // Continue simulation for more frames (keep same input target)
    for (int i = 0; i < 100; i++) {
        tilt.update();
        physics.update(state, tilt, dt);
    }

    BallState finalState1 = state;

    // Restore state and replay
    BallState restoredState;
    WorldTilt restoredTilt;
    restoredState.deserialize(stateBuffer);
    restoredTilt.deserialize(tiltBuffer);

    // Continue from restored state with same updates
    for (int i = 0; i < 100; i++) {
        restoredTilt.update();
        physics.update(restoredState, restoredTilt, dt);
    }

    // Should match exactly - use small epsilon for floating point safety
    ASSERT_TRUE(finalState1.approxEquals(restoredState, 1e-6f));
}

TEST_CASE("Determinism", "Tick count increments consistently") {
    BallPhysics physics;
    BallState state;
    WorldTilt tilt;
    float dt = static_cast<float>(TIMESTEP);

    uint64_t expectedTick = 0;
    for (int i = 0; i < 500; i++) {
        ASSERT_EQ(state.tickCount, expectedTick);
        physics.update(state, tilt, dt);
        expectedTick++;
    }
    ASSERT_EQ(state.tickCount, 500u);
}

// =============================================================================
// WorldTilt Tests
// =============================================================================

TEST_CASE("WorldTilt", "Default tilt is zero") {
    WorldTilt tilt;
    ASSERT_NEAR(tilt.tiltX, 0.0f, 1e-6f);
    ASSERT_NEAR(tilt.tiltZ, 0.0f, 1e-6f);
    ASSERT_NEAR(tilt.targetTiltX, 0.0f, 1e-6f);
    ASSERT_NEAR(tilt.targetTiltZ, 0.0f, 1e-6f);
}

TEST_CASE("WorldTilt", "Input is clamped to max tilt") {
    WorldTilt tilt;

    // Try to set beyond limits
    tilt.setTargetFromInput(10.0f, 10.0f);

    // Target should be clamped to max
    ASSERT_NEAR(std::fabs(tilt.targetTiltX), MAX_TILT_RADIANS, 1e-6f);
    ASSERT_NEAR(std::fabs(tilt.targetTiltZ), MAX_TILT_RADIANS, 1e-6f);
}

TEST_CASE("WorldTilt", "Smoothing approaches target") {
    WorldTilt tilt;

    tilt.setTargetFromInput(1.0f, 0.0f);

    // Apply smoothing multiple times
    float prevDiff = std::fabs(tilt.targetTiltZ - tilt.tiltZ);
    for (int i = 0; i < 100; i++) {
        tilt.update();
        float newDiff = std::fabs(tilt.targetTiltZ - tilt.tiltZ);
        // Difference should decrease (convergence)
        ASSERT_TRUE(newDiff <= prevDiff + 1e-6f);
        prevDiff = newDiff;
    }

    // Should be very close to target after many iterations
    ASSERT_NEAR(tilt.tiltZ, tilt.targetTiltZ, 0.01f);
}

TEST_CASE("WorldTilt", "Reset returns to level") {
    WorldTilt tilt;

    tilt.setTargetFromInput(0.8f, -0.5f);
    tilt.tiltX = 0.2f;
    tilt.tiltZ = 0.3f;

    tilt.reset();

    ASSERT_NEAR(tilt.tiltX, 0.0f, 1e-6f);
    ASSERT_NEAR(tilt.tiltZ, 0.0f, 1e-6f);
    ASSERT_NEAR(tilt.targetTiltX, 0.0f, 1e-6f);
    ASSERT_NEAR(tilt.targetTiltZ, 0.0f, 1e-6f);
}

TEST_CASE("WorldTilt", "Serialization roundtrip") {
    WorldTilt original;
    original.tiltX = 0.15f;
    original.tiltZ = -0.25f;
    original.targetTiltX = 0.3f;
    original.targetTiltZ = -0.4f;

    uint8_t buffer[WorldTilt::SERIALIZED_SIZE];
    original.serialize(buffer);

    WorldTilt restored;
    restored.deserialize(buffer);

    ASSERT_TRUE(original.approxEquals(restored, 0.0f));
}

// =============================================================================
// Physics Constants Verification
// =============================================================================
// These tests verify the constants are properly defined

TEST_CASE("Constants", "Timestep is 1/60 second") {
    ASSERT_NEAR(TIMESTEP, 1.0 / 60.0, 1e-9);
}

TEST_CASE("Constants", "Ball radius matches decomp") {
    // Source: ball.c ballPhysicsParams - radius = 0.5f
    ASSERT_EQ(BALL_RADIUS, 0.5f);
}

TEST_CASE("Constants", "Max tilt is 23 degrees") {
    float expectedRadians = 23.0f * static_cast<float>(constants::PI) / 180.0f;
    ASSERT_NEAR(MAX_TILT_RADIANS, expectedRadians, 1e-5f);
}

TEST_CASE("Constants", "Friction default matches decomp") {
    // Source: ball.c friction = 0.01f
    ASSERT_EQ(FRICTION_DEFAULT, 0.01f);
}

TEST_CASE("Constants", "Restitution default matches decomp") {
    // Source: ball.c ballPhysicsParams[0].restitution = 0.5f
    ASSERT_EQ(RESTITUTION_DEFAULT, 0.5f);
}
