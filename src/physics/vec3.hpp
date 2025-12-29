#pragma once

#include <cmath>

// =============================================================================
// Physics Vec3
// =============================================================================
// 3D vector for physics calculations. This is separate from the rendering
// Vec3 to keep physics code completely independent of raylib.
//
// All operations are designed for deterministic behavior - no undefined
// behavior or platform-dependent results.
// =============================================================================

namespace smb {
namespace physics {

struct Vec3 {
    float x, y, z;

    // =========================================================================
    // Constructors
    // =========================================================================

    Vec3() : x(0.0f), y(0.0f), z(0.0f) {}

    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    explicit Vec3(float scalar) : x(scalar), y(scalar), z(scalar) {}

    // =========================================================================
    // Static Constructors
    // =========================================================================

    static Vec3 zero() { return Vec3(0.0f, 0.0f, 0.0f); }

    static Vec3 one() { return Vec3(1.0f, 1.0f, 1.0f); }

    static Vec3 up() { return Vec3(0.0f, 1.0f, 0.0f); }

    static Vec3 down() { return Vec3(0.0f, -1.0f, 0.0f); }

    static Vec3 right() { return Vec3(1.0f, 0.0f, 0.0f); }

    static Vec3 left() { return Vec3(-1.0f, 0.0f, 0.0f); }

    static Vec3 forward() { return Vec3(0.0f, 0.0f, 1.0f); }

    static Vec3 back() { return Vec3(0.0f, 0.0f, -1.0f); }

    // =========================================================================
    // Arithmetic Operators
    // =========================================================================

    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }

    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }

    Vec3 operator*(float scalar) const { return Vec3(x * scalar, y * scalar, z * scalar); }

    Vec3 operator/(float scalar) const {
        float inv = 1.0f / scalar;
        return Vec3(x * inv, y * inv, z * inv);
    }

    Vec3 operator-() const { return Vec3(-x, -y, -z); }

    // Compound assignment
    Vec3& operator+=(const Vec3& v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    Vec3& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    Vec3& operator/=(float scalar) {
        float inv = 1.0f / scalar;
        x *= inv;
        y *= inv;
        z *= inv;
        return *this;
    }

    // =========================================================================
    // Vector Operations
    // =========================================================================

    // Dot product
    float dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }

    // Cross product
    Vec3 cross(const Vec3& v) const {
        return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }

    // Length squared (faster than length when you just need comparison)
    float lengthSquared() const { return x * x + y * y + z * z; }

    // Length (magnitude)
    float length() const { return std::sqrt(lengthSquared()); }

    // Returns normalized vector (unit length)
    // If vector is near-zero, returns zero vector to avoid NaN
    Vec3 normalized() const {
        float len = length();
        if (len < 1e-8f) {
            return Vec3::zero();
        }
        return *this / len;
    }

    // Normalize in place
    Vec3& normalize() {
        float len = length();
        if (len < 1e-8f) {
            x = y = z = 0.0f;
        } else {
            float inv = 1.0f / len;
            x *= inv;
            y *= inv;
            z *= inv;
        }
        return *this;
    }

    // Component-wise absolute value
    Vec3 abs() const { return Vec3(std::fabs(x), std::fabs(y), std::fabs(z)); }

    // Component-wise minimum
    static Vec3 min(const Vec3& a, const Vec3& b) {
        return Vec3(std::fmin(a.x, b.x), std::fmin(a.y, b.y), std::fmin(a.z, b.z));
    }

    // Component-wise maximum
    static Vec3 max(const Vec3& a, const Vec3& b) {
        return Vec3(std::fmax(a.x, b.x), std::fmax(a.y, b.y), std::fmax(a.z, b.z));
    }

    // Linear interpolation
    static Vec3 lerp(const Vec3& a, const Vec3& b, float t) {
        return Vec3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);
    }

    // Distance between two points
    static float distance(const Vec3& a, const Vec3& b) { return (a - b).length(); }

    // Distance squared between two points
    static float distanceSquared(const Vec3& a, const Vec3& b) { return (a - b).lengthSquared(); }

    // =========================================================================
    // Projection
    // =========================================================================

    // Project this vector onto another vector
    Vec3 projectOnto(const Vec3& onto) const {
        float ontoLenSq = onto.lengthSquared();
        if (ontoLenSq < 1e-8f) {
            return Vec3::zero();
        }
        return onto * (this->dot(onto) / ontoLenSq);
    }

    // Get component of this vector perpendicular to another
    Vec3 rejectFrom(const Vec3& from) const { return *this - projectOnto(from); }

    // Reflect vector off a surface with given normal
    Vec3 reflect(const Vec3& normal) const { return *this - normal * (2.0f * this->dot(normal)); }

    // =========================================================================
    // Comparison
    // =========================================================================

    bool operator==(const Vec3& v) const { return x == v.x && y == v.y && z == v.z; }

    bool operator!=(const Vec3& v) const { return !(*this == v); }

    // Approximate equality with tolerance
    // Uses <= for correct behavior when epsilon is 0 (exact match)
    bool approxEquals(const Vec3& v, float epsilon = 1e-6f) const {
        return std::fabs(x - v.x) <= epsilon && std::fabs(y - v.y) <= epsilon &&
               std::fabs(z - v.z) <= epsilon;
    }

    // Check if near zero
    bool isNearZero(float epsilon = 1e-6f) const { return lengthSquared() < epsilon * epsilon; }
};

// Scalar * vector (for commutativity)
inline Vec3 operator*(float scalar, const Vec3& v) {
    return Vec3(v.x * scalar, v.y * scalar, v.z * scalar);
}

// Dot product as free function
inline float dot(const Vec3& a, const Vec3& b) {
    return a.dot(b);
}

// Cross product as free function
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return a.cross(b);
}

}  // namespace physics
}  // namespace smb
