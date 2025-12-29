#include "physics/ball_state.hpp"
#include "physics/vec3.hpp"

#include <cstring>

#include "test_framework.hpp"

// =============================================================================
// Vec3 Tests
// =============================================================================

TEST_CASE("Vec3", "Default construction is zero") {
    smb::physics::Vec3 v;
    ASSERT_NEAR(v.x, 0.0f, 0.0001f);
    ASSERT_NEAR(v.y, 0.0f, 0.0001f);
    ASSERT_NEAR(v.z, 0.0f, 0.0001f);
}

TEST_CASE("Vec3", "Component construction works") {
    smb::physics::Vec3 v(1.0f, 2.0f, 3.0f);
    ASSERT_NEAR(v.x, 1.0f, 0.0001f);
    ASSERT_NEAR(v.y, 2.0f, 0.0001f);
    ASSERT_NEAR(v.z, 3.0f, 0.0001f);
}

TEST_CASE("Vec3", "Addition works") {
    smb::physics::Vec3 a(1.0f, 2.0f, 3.0f);
    smb::physics::Vec3 b(4.0f, 5.0f, 6.0f);
    smb::physics::Vec3 c = a + b;

    ASSERT_NEAR(c.x, 5.0f, 0.0001f);
    ASSERT_NEAR(c.y, 7.0f, 0.0001f);
    ASSERT_NEAR(c.z, 9.0f, 0.0001f);
}

TEST_CASE("Vec3", "Subtraction works") {
    smb::physics::Vec3 a(4.0f, 5.0f, 6.0f);
    smb::physics::Vec3 b(1.0f, 2.0f, 3.0f);
    smb::physics::Vec3 c = a - b;

    ASSERT_NEAR(c.x, 3.0f, 0.0001f);
    ASSERT_NEAR(c.y, 3.0f, 0.0001f);
    ASSERT_NEAR(c.z, 3.0f, 0.0001f);
}

TEST_CASE("Vec3", "Scalar multiplication works") {
    smb::physics::Vec3 v(1.0f, 2.0f, 3.0f);
    smb::physics::Vec3 scaled = v * 2.0f;

    ASSERT_NEAR(scaled.x, 2.0f, 0.0001f);
    ASSERT_NEAR(scaled.y, 4.0f, 0.0001f);
    ASSERT_NEAR(scaled.z, 6.0f, 0.0001f);
}

TEST_CASE("Vec3", "Dot product is correct") {
    smb::physics::Vec3 a(1.0f, 2.0f, 3.0f);
    smb::physics::Vec3 b(4.0f, 5.0f, 6.0f);
    float dotResult = a.dot(b);

    // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
    ASSERT_NEAR(dotResult, 32.0f, 0.0001f);
}

TEST_CASE("Vec3", "Cross product is correct") {
    smb::physics::Vec3 x(1.0f, 0.0f, 0.0f);
    smb::physics::Vec3 y(0.0f, 1.0f, 0.0f);
    smb::physics::Vec3 z = x.cross(y);

    ASSERT_NEAR(z.x, 0.0f, 0.0001f);
    ASSERT_NEAR(z.y, 0.0f, 0.0001f);
    ASSERT_NEAR(z.z, 1.0f, 0.0001f);
}

TEST_CASE("Vec3", "Length calculation is correct") {
    smb::physics::Vec3 v(3.0f, 4.0f, 0.0f);
    ASSERT_NEAR(v.length(), 5.0f, 0.0001f);
}

TEST_CASE("Vec3", "Normalization produces unit length") {
    smb::physics::Vec3 v(3.0f, 4.0f, 0.0f);
    smb::physics::Vec3 n = v.normalized();

    ASSERT_NEAR(n.length(), 1.0f, 0.0001f);
    ASSERT_NEAR(n.x, 0.6f, 0.0001f);
    ASSERT_NEAR(n.y, 0.8f, 0.0001f);
}

TEST_CASE("Vec3", "Zero vector normalization returns zero") {
    smb::physics::Vec3 v(0.0f, 0.0f, 0.0f);
    smb::physics::Vec3 n = v.normalized();

    ASSERT_NEAR(n.x, 0.0f, 0.0001f);
    ASSERT_NEAR(n.y, 0.0f, 0.0001f);
    ASSERT_NEAR(n.z, 0.0f, 0.0001f);
}

TEST_CASE("Vec3", "Reflection works correctly") {
    smb::physics::Vec3 incoming(1.0f, -1.0f, 0.0f);
    smb::physics::Vec3 normal(0.0f, 1.0f, 0.0f);
    smb::physics::Vec3 reflected = incoming.reflect(normal);

    ASSERT_NEAR(reflected.x, 1.0f, 0.0001f);
    ASSERT_NEAR(reflected.y, 1.0f, 0.0001f);
    ASSERT_NEAR(reflected.z, 0.0f, 0.0001f);
}

// =============================================================================
// BallState Tests
// =============================================================================

TEST_CASE("BallState", "Default construction has correct initial values") {
    smb::physics::BallState state;

    // Position should be on ground (y = radius)
    ASSERT_NEAR(state.position.x, 0.0f, 0.0001f);
    ASSERT_NEAR(state.position.y, smb::physics::BALL_RADIUS, 0.0001f);
    ASSERT_NEAR(state.position.z, 0.0f, 0.0001f);

    // Velocity should be zero
    ASSERT_NEAR(state.velocity.x, 0.0f, 0.0001f);
    ASSERT_NEAR(state.velocity.y, 0.0f, 0.0001f);
    ASSERT_NEAR(state.velocity.z, 0.0f, 0.0001f);

    // Should be grounded
    ASSERT_TRUE(state.isGrounded());
    ASSERT_FALSE(state.isAirborne());
}

TEST_CASE("BallState", "Radius matches physics constant") {
    smb::physics::BallState state;
    ASSERT_NEAR(state.radius, smb::physics::BALL_RADIUS, 0.0001f);
}

TEST_CASE("BallState", "Restitution matches physics constant") {
    smb::physics::BallState state;
    ASSERT_NEAR(state.restitution, smb::physics::RESTITUTION_DEFAULT, 0.0001f);
}

TEST_CASE("BallState", "Friction matches physics constant") {
    smb::physics::BallState state;
    ASSERT_NEAR(state.friction, smb::physics::FRICTION_DEFAULT, 0.0001f);
}

TEST_CASE("BallState", "Flag operations work correctly") {
    smb::physics::BallState state;

    // Initially grounded
    ASSERT_TRUE(state.isGrounded());

    // Set airborne
    state.clearFlag(smb::physics::BallFlags::GROUNDED);
    state.setFlag(smb::physics::BallFlags::AIRBORNE);

    ASSERT_FALSE(state.isGrounded());
    ASSERT_TRUE(state.isAirborne());
}

TEST_CASE("BallState", "Speed calculation is correct") {
    smb::physics::BallState state;
    state.velocity = smb::physics::Vec3(3.0f, 4.0f, 0.0f);

    ASSERT_NEAR(state.getSpeed(), 5.0f, 0.0001f);
}

TEST_CASE("BallState", "Stationary detection works") {
    smb::physics::BallState state;

    // Zero velocity is stationary
    state.velocity = smb::physics::Vec3(0.0f, 0.0f, 0.0f);
    ASSERT_TRUE(state.isStationary());

    // Large velocity is not stationary
    state.velocity = smb::physics::Vec3(1.0f, 0.0f, 0.0f);
    ASSERT_FALSE(state.isStationary());
}

TEST_CASE("BallState", "Reset clears state correctly") {
    smb::physics::BallState state;

    // Modify state
    state.position = smb::physics::Vec3(10.0f, 5.0f, 3.0f);
    state.velocity = smb::physics::Vec3(1.0f, 2.0f, 3.0f);
    state.tickCount = 1000;
    state.setFlag(smb::physics::BallFlags::AIRBORNE);

    // Reset
    smb::physics::Vec3 startPos(0.0f, smb::physics::BALL_RADIUS, 0.0f);
    state.reset(startPos);

    // Check reset values
    ASSERT_TRUE(state.position.approxEquals(startPos));
    ASSERT_TRUE(state.velocity.isNearZero());
    ASSERT_EQ(state.tickCount, 0u);
    ASSERT_TRUE(state.isGrounded());
    ASSERT_FALSE(state.isAirborne());
}

TEST_CASE("BallState", "Serialization round-trip preserves state") {
    smb::physics::BallState original;

    // Set non-default values
    original.position = smb::physics::Vec3(1.5f, 2.5f, 3.5f);
    original.velocity = smb::physics::Vec3(-0.5f, 1.0f, -1.5f);
    original.angularVelocity = smb::physics::Vec3(0.1f, 0.2f, 0.3f);
    original.rotation = smb::physics::Quaternion::fromAxisAngle(smb::physics::Vec3::up(), 0.5f);
    original.radius = 0.6f;
    original.restitution = 0.7f;
    original.friction = 0.02f;
    original.flags = smb::physics::BallFlags::AIRBORNE;
    original.tickCount = 12345;

    // Serialize
    uint8_t buffer[smb::physics::BallState::SERIALIZED_SIZE];
    size_t written = original.serialize(buffer);
    ASSERT_EQ(written, smb::physics::BallState::SERIALIZED_SIZE);

    // Deserialize into new state
    smb::physics::BallState restored;
    size_t read = restored.deserialize(buffer);
    ASSERT_EQ(read, smb::physics::BallState::SERIALIZED_SIZE);

    // Verify all fields match
    ASSERT_TRUE(restored.position.approxEquals(original.position));
    ASSERT_TRUE(restored.velocity.approxEquals(original.velocity));
    ASSERT_TRUE(restored.angularVelocity.approxEquals(original.angularVelocity));
    ASSERT_NEAR(restored.radius, original.radius, 0.0001f);
    ASSERT_NEAR(restored.restitution, original.restitution, 0.0001f);
    ASSERT_NEAR(restored.friction, original.friction, 0.0001f);
    ASSERT_TRUE(static_cast<uint32_t>(restored.flags) == static_cast<uint32_t>(original.flags));
    ASSERT_EQ(restored.tickCount, original.tickCount);
}

TEST_CASE("BallState", "approxEquals detects matching states") {
    smb::physics::BallState a;
    smb::physics::BallState b;

    // Same default state
    ASSERT_TRUE(a.approxEquals(b));

    // Slightly different but within epsilon
    b.position.x += 0.0000001f;
    ASSERT_TRUE(a.approxEquals(b));

    // Significantly different
    b.position.x = 10.0f;
    ASSERT_FALSE(a.approxEquals(b));
}

// =============================================================================
// Quaternion Tests
// =============================================================================

TEST_CASE("Quaternion", "Identity is correct") {
    smb::physics::Quaternion q = smb::physics::Quaternion::identity();

    ASSERT_NEAR(q.w, 1.0f, 0.0001f);
    ASSERT_NEAR(q.x, 0.0f, 0.0001f);
    ASSERT_NEAR(q.y, 0.0f, 0.0001f);
    ASSERT_NEAR(q.z, 0.0f, 0.0001f);
}

TEST_CASE("Quaternion", "Axis-angle construction works") {
    smb::physics::Vec3 axis(0.0f, 1.0f, 0.0f);                             // Y axis
    float angle = static_cast<float>(smb::physics::constants::PI) / 2.0f;  // 90 degrees

    smb::physics::Quaternion q = smb::physics::Quaternion::fromAxisAngle(axis, angle);

    // For 90 degree rotation around Y:
    // w = cos(45°) ≈ 0.707
    // y = sin(45°) ≈ 0.707
    ASSERT_NEAR(q.w, 0.7071f, 0.001f);
    ASSERT_NEAR(q.x, 0.0f, 0.001f);
    ASSERT_NEAR(q.y, 0.7071f, 0.001f);
    ASSERT_NEAR(q.z, 0.0f, 0.001f);
}

TEST_CASE("Quaternion", "Multiplication combines rotations") {
    smb::physics::Quaternion q1 = smb::physics::Quaternion::identity();
    smb::physics::Quaternion q2 = smb::physics::Quaternion::identity();

    smb::physics::Quaternion result = q1 * q2;

    // Identity * Identity = Identity
    ASSERT_NEAR(result.w, 1.0f, 0.0001f);
    ASSERT_NEAR(result.x, 0.0f, 0.0001f);
    ASSERT_NEAR(result.y, 0.0f, 0.0001f);
    ASSERT_NEAR(result.z, 0.0f, 0.0001f);
}

TEST_CASE("Quaternion", "Normalization produces unit quaternion") {
    smb::physics::Quaternion q(2.0f, 0.0f, 0.0f, 0.0f);
    smb::physics::Quaternion n = q.normalized();

    float len = std::sqrt(n.w * n.w + n.x * n.x + n.y * n.y + n.z * n.z);
    ASSERT_NEAR(len, 1.0f, 0.0001f);
}
