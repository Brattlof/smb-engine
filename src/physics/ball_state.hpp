#pragma once

#include "physics/physics_constants.hpp"
#include "physics/vec3.hpp"

#include <cstdint>
#include <cstring>

// =============================================================================
// Ball State
// =============================================================================
// Complete state of the ball for physics simulation. This struct captures
// everything needed to reproduce ball behavior deterministically.
//
// For replays and save states, the entire state can be serialized and
// restored to produce identical physics outcomes.
//
// Units follow physics_constants.hpp conventions:
//   - Position: game units
//   - Velocity: game units per second
//   - Angular velocity: radians per second
//   - Rotation: quaternion (unit quaternion)
// =============================================================================

namespace smb {
namespace physics {

// =============================================================================
// Quaternion - for ball rotation
// =============================================================================
// Quaternions avoid gimbal lock and interpolate smoothly for visual rotation.
// The ball's angular velocity is still stored as a Vec3 (axis * angular speed).

struct Quaternion {
    float w, x, y, z;

    // Default: identity quaternion (no rotation)
    Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}

    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    // Create identity quaternion
    static Quaternion identity() { return Quaternion(1.0f, 0.0f, 0.0f, 0.0f); }

    // Create quaternion from axis-angle (axis should be normalized)
    static Quaternion fromAxisAngle(const Vec3& axis, float angle) {
        float halfAngle = angle * 0.5f;
        float s = std::sin(halfAngle);
        float c = std::cos(halfAngle);
        return Quaternion(c, axis.x * s, axis.y * s, axis.z * s);
    }

    // Multiply quaternions (combine rotations)
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w * q.w - x * q.x - y * q.y - z * q.z, w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x, w * q.z + x * q.y - y * q.x + z * q.w);
    }

    // Normalize the quaternion (should always be unit length)
    Quaternion normalized() const {
        float len = std::sqrt(w * w + x * x + y * y + z * z);
        if (len < EPSILON) {
            return identity();
        }
        float invLen = 1.0f / len;
        return Quaternion(w * invLen, x * invLen, y * invLen, z * invLen);
    }

    // Check if quaternion is approximately equal to another
    // Uses <= for correct behavior when epsilon is 0 (exact match)
    bool approxEquals(const Quaternion& q, float epsilon = EPSILON) const {
        // Quaternions q and -q represent the same rotation
        float dotPos = w * q.w + x * q.x + y * q.y + z * q.z;
        return std::fabs(std::fabs(dotPos) - 1.0f) <= epsilon;
    }
};

// =============================================================================
// Ball State Flags
// =============================================================================
// Flags matching SMB behavior for special ball states

enum class BallFlags : uint32_t {
    NONE = 0,
    GROUNDED = 1 << 0,       // Ball is in contact with ground
    AIRBORNE = 1 << 1,       // Ball is in the air
    GOAL_REACHED = 1 << 2,   // Ball entered goal
    FELL_OUT = 1 << 3,       // Ball fell off stage
    FROZEN = 1 << 4,         // Ball physics paused (cutscene, etc.)
    REVERSE_ACCEL = 1 << 5,  // Acceleration reversed (matches BALL_FLAG_09)
    NO_ACCEL = 1 << 6,       // Acceleration disabled (matches BALL_FLAG_08)
};

inline BallFlags operator|(BallFlags a, BallFlags b) {
    return static_cast<BallFlags>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline BallFlags operator&(BallFlags a, BallFlags b) {
    return static_cast<BallFlags>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

inline BallFlags operator~(BallFlags a) {
    return static_cast<BallFlags>(~static_cast<uint32_t>(a));
}

inline bool hasFlag(BallFlags flags, BallFlags flag) {
    return (flags & flag) != BallFlags::NONE;
}

// =============================================================================
// Ball State Structure
// =============================================================================

struct BallState {
    // --- Position and Motion ---

    // Center position of ball in world space (game units)
    Vec3 position;

    // Linear velocity (game units per second)
    Vec3 velocity;

    // Angular velocity as axis * angular speed (radians per second)
    // The magnitude is the rotation speed, direction is the axis
    Vec3 angularVelocity;

    // Orientation as quaternion (for rendering the ball's rotation)
    Quaternion rotation;

    // --- Physics Properties ---

    // Ball radius - usually BALL_RADIUS but could vary
    float radius;

    // Restitution (bounciness) - how much velocity is retained on bounce
    float restitution;

    // Current friction coefficient
    float friction;

    // --- State Flags ---
    BallFlags flags;

    // --- Frame Counter ---
    // Used for determinism verification - counts physics ticks since reset
    uint64_t tickCount;

    // =======================================================================
    // Constructors
    // =======================================================================

    BallState()
        : position(0.0f, BALL_RADIUS, 0.0f),  // Start on ground
          velocity(0.0f, 0.0f, 0.0f),
          angularVelocity(0.0f, 0.0f, 0.0f),
          rotation(Quaternion::identity()),
          radius(BALL_RADIUS),
          restitution(RESTITUTION_DEFAULT),
          friction(FRICTION_DEFAULT),
          flags(BallFlags::GROUNDED),
          tickCount(0) {}

    // =======================================================================
    // State Queries
    // =======================================================================

    bool isGrounded() const { return hasFlag(flags, BallFlags::GROUNDED); }

    bool isAirborne() const { return hasFlag(flags, BallFlags::AIRBORNE); }

    bool hasReachedGoal() const { return hasFlag(flags, BallFlags::GOAL_REACHED); }

    bool hasFallenOut() const { return hasFlag(flags, BallFlags::FELL_OUT); }

    bool isFrozen() const { return hasFlag(flags, BallFlags::FROZEN); }

    // Returns the speed (magnitude of velocity)
    float getSpeed() const { return velocity.length(); }

    // Returns the angular speed (magnitude of angular velocity)
    float getAngularSpeed() const { return angularVelocity.length(); }

    // Check if ball is effectively stationary
    bool isStationary() const {
        return velocity.lengthSquared() < MIN_VELOCITY_THRESHOLD * MIN_VELOCITY_THRESHOLD;
    }

    // =======================================================================
    // State Modification
    // =======================================================================

    void setFlag(BallFlags flag) { flags = flags | flag; }

    void clearFlag(BallFlags flag) { flags = flags & (~flag); }

    void toggleFlag(BallFlags flag) {
        if (hasFlag(flags, flag)) {
            clearFlag(flag);
        } else {
            setFlag(flag);
        }
    }

    // Reset to initial state (for level restart)
    void reset(const Vec3& startPos = Vec3(0.0f, BALL_RADIUS, 0.0f)) {
        position = startPos;
        velocity = Vec3(0.0f, 0.0f, 0.0f);
        angularVelocity = Vec3(0.0f, 0.0f, 0.0f);
        rotation = Quaternion::identity();
        flags = BallFlags::GROUNDED;
        tickCount = 0;
    }

    // =======================================================================
    // Serialization
    // =======================================================================
    // For save states and replays. Binary format for determinism.

    // Size of serialized state in bytes
    static constexpr size_t SERIALIZED_SIZE = sizeof(float) * 3 +  // position
                                              sizeof(float) * 3 +  // velocity
                                              sizeof(float) * 3 +  // angularVelocity
                                              sizeof(float) * 4 +  // rotation quaternion
                                              sizeof(float) * 3 +  // radius, restitution, friction
                                              sizeof(uint32_t) +   // flags
                                              sizeof(uint64_t);    // tickCount

    // Serialize to byte buffer. Buffer must be at least SERIALIZED_SIZE bytes.
    // Returns number of bytes written.
    size_t serialize(void* buffer) const {
        uint8_t* ptr = static_cast<uint8_t*>(buffer);

        std::memcpy(ptr, &position.x, sizeof(float));
        ptr += sizeof(float);
        std::memcpy(ptr, &position.y, sizeof(float));
        ptr += sizeof(float);
        std::memcpy(ptr, &position.z, sizeof(float));
        ptr += sizeof(float);

        std::memcpy(ptr, &velocity.x, sizeof(float));
        ptr += sizeof(float);
        std::memcpy(ptr, &velocity.y, sizeof(float));
        ptr += sizeof(float);
        std::memcpy(ptr, &velocity.z, sizeof(float));
        ptr += sizeof(float);

        std::memcpy(ptr, &angularVelocity.x, sizeof(float));
        ptr += sizeof(float);
        std::memcpy(ptr, &angularVelocity.y, sizeof(float));
        ptr += sizeof(float);
        std::memcpy(ptr, &angularVelocity.z, sizeof(float));
        ptr += sizeof(float);

        std::memcpy(ptr, &rotation.w, sizeof(float));
        ptr += sizeof(float);
        std::memcpy(ptr, &rotation.x, sizeof(float));
        ptr += sizeof(float);
        std::memcpy(ptr, &rotation.y, sizeof(float));
        ptr += sizeof(float);
        std::memcpy(ptr, &rotation.z, sizeof(float));
        ptr += sizeof(float);

        std::memcpy(ptr, &radius, sizeof(float));
        ptr += sizeof(float);
        std::memcpy(ptr, &restitution, sizeof(float));
        ptr += sizeof(float);
        std::memcpy(ptr, &friction, sizeof(float));
        ptr += sizeof(float);

        uint32_t flagsVal = static_cast<uint32_t>(flags);
        std::memcpy(ptr, &flagsVal, sizeof(uint32_t));
        ptr += sizeof(uint32_t);

        std::memcpy(ptr, &tickCount, sizeof(uint64_t));
        ptr += sizeof(uint64_t);

        return SERIALIZED_SIZE;
    }

    // Deserialize from byte buffer. Buffer must be at least SERIALIZED_SIZE bytes.
    // Returns number of bytes read.
    size_t deserialize(const void* buffer) {
        const uint8_t* ptr = static_cast<const uint8_t*>(buffer);

        std::memcpy(&position.x, ptr, sizeof(float));
        ptr += sizeof(float);
        std::memcpy(&position.y, ptr, sizeof(float));
        ptr += sizeof(float);
        std::memcpy(&position.z, ptr, sizeof(float));
        ptr += sizeof(float);

        std::memcpy(&velocity.x, ptr, sizeof(float));
        ptr += sizeof(float);
        std::memcpy(&velocity.y, ptr, sizeof(float));
        ptr += sizeof(float);
        std::memcpy(&velocity.z, ptr, sizeof(float));
        ptr += sizeof(float);

        std::memcpy(&angularVelocity.x, ptr, sizeof(float));
        ptr += sizeof(float);
        std::memcpy(&angularVelocity.y, ptr, sizeof(float));
        ptr += sizeof(float);
        std::memcpy(&angularVelocity.z, ptr, sizeof(float));
        ptr += sizeof(float);

        std::memcpy(&rotation.w, ptr, sizeof(float));
        ptr += sizeof(float);
        std::memcpy(&rotation.x, ptr, sizeof(float));
        ptr += sizeof(float);
        std::memcpy(&rotation.y, ptr, sizeof(float));
        ptr += sizeof(float);
        std::memcpy(&rotation.z, ptr, sizeof(float));
        ptr += sizeof(float);

        std::memcpy(&radius, ptr, sizeof(float));
        ptr += sizeof(float);
        std::memcpy(&restitution, ptr, sizeof(float));
        ptr += sizeof(float);
        std::memcpy(&friction, ptr, sizeof(float));
        ptr += sizeof(float);

        uint32_t flagsVal;
        std::memcpy(&flagsVal, ptr, sizeof(uint32_t));
        flags = static_cast<BallFlags>(flagsVal);
        ptr += sizeof(uint32_t);

        std::memcpy(&tickCount, ptr, sizeof(uint64_t));
        ptr += sizeof(uint64_t);

        return SERIALIZED_SIZE;
    }

    // =======================================================================
    // Comparison
    // =======================================================================

    // Check if two states are approximately equal (for determinism tests)
    // Uses <= for correct behavior when epsilon is 0 (exact match)
    bool approxEquals(const BallState& other, float epsilon = EPSILON) const {
        return position.approxEquals(other.position, epsilon) &&
               velocity.approxEquals(other.velocity, epsilon) &&
               angularVelocity.approxEquals(other.angularVelocity, epsilon) &&
               rotation.approxEquals(other.rotation, epsilon) &&
               std::fabs(radius - other.radius) <= epsilon &&
               std::fabs(restitution - other.restitution) <= epsilon &&
               std::fabs(friction - other.friction) <= epsilon && flags == other.flags &&
               tickCount == other.tickCount;
    }
};

}  // namespace physics
}  // namespace smb
