#include "physics/physics_constants.hpp"

#include "test_framework.hpp"

// =============================================================================
// Physics Constants Tests
// =============================================================================
// Verify physics constants match SMB decomp values and conversions are correct
// =============================================================================

TEST_CASE("PhysicsConstants", "Ball radius matches decomp") {
    // Source: ball.c ballPhysicsParams - radius = 0.5f
    ASSERT_NEAR(smb::physics::BALL_RADIUS, 0.5f, 0.0001f);
}

TEST_CASE("PhysicsConstants", "Ball acceleration matches decomp") {
    // Source: ball.c ballPhysicsParams[0].unk8 = 0.009799992f per frame
    ASSERT_NEAR(smb::physics::BALL_ACCEL_PER_FRAME, 0.009799992f, 0.0000001f);
}

TEST_CASE("PhysicsConstants", "Friction matches decomp") {
    // Source: ball.c - friction = 0.01f
    ASSERT_NEAR(smb::physics::FRICTION_DEFAULT, 0.01f, 0.0001f);
}

TEST_CASE("PhysicsConstants", "Restitution matches decomp") {
    // Source: ball.c ballPhysicsParams[0].restitution = 0.5f
    ASSERT_NEAR(smb::physics::RESTITUTION_DEFAULT, 0.5f, 0.0001f);
}

TEST_CASE("PhysicsConstants", "Max tilt matches decomp") {
    // Source: world.c - max tilt = 23.0 degrees
    ASSERT_NEAR(smb::physics::MAX_TILT_DEGREES, 23.0f, 0.0001f);
}

TEST_CASE("PhysicsConstants", "Tilt smoothing matches decomp") {
    // Source: world.c - 0.2 multiplier applied to rotation deltas
    ASSERT_NEAR(smb::physics::TILT_SMOOTHING, 0.2f, 0.0001f);
}

TEST_CASE("PhysicsConstants", "Angular damping matches decomp") {
    // Source: ball.c - angularVelocity *= 0.95 when airborne
    ASSERT_NEAR(smb::physics::ANGULAR_DAMPING_AIRBORNE, 0.95f, 0.0001f);
}

TEST_CASE("PhysicsConstants", "Collision velocity retention matches decomp") {
    // Source: ball.c - collision response uses 0.92 factor
    ASSERT_NEAR(smb::physics::COLLISION_VELOCITY_RETENTION, 0.92f, 0.0001f);
}

TEST_CASE("PhysicsConstants", "Angle conversion is reversible") {
    using namespace smb::physics;

    // Test angle -> radians -> angle
    int originalAngle = 16384;  // 90 degrees in SMB format
    float radians = smbAngleToRadians(originalAngle);
    int convertedBack = radiansToSmbAngle(radians);

    ASSERT_EQ(originalAngle, convertedBack);
}

TEST_CASE("PhysicsConstants", "90 degrees converts correctly") {
    using namespace smb::physics;

    // 0x4000 (16384) = 90 degrees = π/2 radians
    float radians = smbAngleToRadians(0x4000);
    float expected = static_cast<float>(constants::PI) / 2.0f;

    ASSERT_NEAR(radians, expected, 0.0001f);
}

TEST_CASE("PhysicsConstants", "Degree conversion is accurate") {
    using namespace smb::physics;

    ASSERT_NEAR(degreesToRadians(180.0f), static_cast<float>(constants::PI), 0.0001f);
    ASSERT_NEAR(radiansToDegrees(static_cast<float>(constants::PI)), 180.0f, 0.0001f);
}

TEST_CASE("PhysicsConstants", "isNearZero works correctly") {
    using namespace smb::physics;

    ASSERT_TRUE(isNearZero(0.0f));
    ASSERT_TRUE(isNearZero(0.0000001f));
    ASSERT_TRUE(isNearZero(-0.0000001f));
    ASSERT_FALSE(isNearZero(0.001f));
    ASSERT_FALSE(isNearZero(-0.001f));
}

TEST_CASE("PhysicsConstants", "Clamp works correctly") {
    using namespace smb::physics;

    ASSERT_NEAR(clamp(5.0f, 0.0f, 10.0f), 5.0f, 0.0001f);
    ASSERT_NEAR(clamp(-5.0f, 0.0f, 10.0f), 0.0f, 0.0001f);
    ASSERT_NEAR(clamp(15.0f, 0.0f, 10.0f), 10.0f, 0.0001f);
}

TEST_CASE("PhysicsConstants", "Timestep is 60Hz") {
    ASSERT_NEAR(smb::physics::TIMESTEP, 1.0 / 60.0, 0.0001);
}

TEST_CASE("PhysicsConstants", "Gravity direction is normalized down") {
    using namespace smb::physics;

    // Gravity should point straight down (-Y)
    ASSERT_NEAR(GRAVITY_DIR_X, 0.0f, 0.0001f);
    ASSERT_NEAR(GRAVITY_DIR_Y, -1.0f, 0.0001f);
    ASSERT_NEAR(GRAVITY_DIR_Z, 0.0f, 0.0001f);
}
