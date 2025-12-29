#pragma once

#include "rendering/renderer.hpp"

#include <string>
#include <vector>

// =============================================================================
// Debug Drawing System
// =============================================================================
// Provides debug visualization primitives for physics development.
// All debug draws are queued and rendered at the end of the frame.
// Toggle-able at runtime with F1 (or via setEnabled).
//
// Use this for:
//   - Velocity vectors
//   - Contact points and normals
//   - Collision wireframes
//   - Physics state visualization
//   - Coordinate axes
//
// The debug drawer uses immediate-mode style API. Call the draw functions
// every frame for things you want to see - nothing persists automatically.
// =============================================================================

namespace smb {

// Different categories of debug info - can toggle individually
enum class DebugCategory : uint32_t {
    NONE = 0,
    PHYSICS = 1 << 0,    // Ball physics, velocity, forces
    COLLISION = 1 << 1,  // Collision shapes, contacts
    CAMERA = 1 << 2,     // Camera frustum, target
    WORLD = 1 << 3,      // World tilt, gravity direction
    INPUT = 1 << 4,      // Input visualization
    TIMING = 1 << 5,     // Frame timing, physics ticks
    ALL = 0xFFFFFFFF
};

inline DebugCategory operator|(DebugCategory a, DebugCategory b) {
    return static_cast<DebugCategory>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline DebugCategory operator&(DebugCategory a, DebugCategory b) {
    return static_cast<DebugCategory>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

class DebugDraw {
public:
    explicit DebugDraw(Renderer* renderer);

    // Enable/disable all debug drawing
    void setEnabled(bool enabled);
    bool isEnabled() const;

    // Enable/disable specific categories
    void setCategoryEnabled(DebugCategory category, bool enabled);
    bool isCategoryEnabled(DebugCategory category) const;

    // Toggle a category
    void toggleCategory(DebugCategory category);

    // =========================================================================
    // 3D Debug Primitives
    // =========================================================================

    // Draw a point (rendered as small sphere)
    void point(Vec3 pos, Color color, DebugCategory category = DebugCategory::PHYSICS);

    // Draw a line
    void line(Vec3 start, Vec3 end, Color color, DebugCategory category = DebugCategory::PHYSICS);

    // Draw an arrow (line with arrowhead) - great for vectors
    void arrow(Vec3 start, Vec3 end, Color color, DebugCategory category = DebugCategory::PHYSICS);

    // Draw a ray (arrow that starts at origin, shows direction)
    void ray(Vec3 origin, Vec3 direction, float length, Color color,
             DebugCategory category = DebugCategory::PHYSICS);

    // Draw a wireframe sphere
    void sphere(Vec3 center, float radius, Color color,
                DebugCategory category = DebugCategory::COLLISION);

    // Draw a wireframe box
    void box(Vec3 center, Vec3 size, Color color,
             DebugCategory category = DebugCategory::COLLISION);

    // Draw an axis-aligned bounding box
    void aabb(Vec3 min, Vec3 max, Color color, DebugCategory category = DebugCategory::COLLISION);

    // Draw coordinate axes at a position
    void axes(Vec3 origin, float length = 1.0f, DebugCategory category = DebugCategory::WORLD);

    // Draw a ground plane grid
    void groundGrid(float spacing, int lines, Color color,
                    DebugCategory category = DebugCategory::WORLD);

    // Draw a contact point with normal
    void contact(Vec3 point, Vec3 normal, Color color = Color::yellow(),
                 DebugCategory category = DebugCategory::COLLISION);

    // =========================================================================
    // 2D Debug Overlays
    // =========================================================================

    // Draw text at screen position
    void text2D(int x, int y, const char* text, Color color = Color::white(),
                DebugCategory category = DebugCategory::TIMING);

    // Draw text in 3D space (always faces camera - billboard)
    void text3D(Vec3 pos, const char* text, Color color = Color::white(),
                DebugCategory category = DebugCategory::PHYSICS);

    // =========================================================================
    // Rendering
    // =========================================================================

    // Called at start of 3D rendering
    void begin3D(const CameraData& camera);

    // Render all queued 3D debug primitives
    void render3D();

    // Called at end of 3D rendering
    void end3D();

    // Render all queued 2D debug overlays
    void render2D();

    // Clear all queued primitives (called automatically after render)
    void clear();

private:
    Renderer* m_renderer;
    bool m_enabled;
    DebugCategory m_enabledCategories;

    // We store the camera for text3D billboarding
    CameraData m_camera;

    // Queued primitives
    struct DebugLine {
        Vec3 start, end;
        Color color;
        DebugCategory category;
    };

    struct DebugSphere {
        Vec3 center;
        float radius;
        Color color;
        DebugCategory category;
    };

    struct DebugBox {
        Vec3 center;
        Vec3 size;
        Color color;
        DebugCategory category;
    };

    struct DebugText2D {
        int x, y;
        std::string text;
        Color color;
        DebugCategory category;
    };

    std::vector<DebugLine> m_lines;
    std::vector<DebugSphere> m_spheres;
    std::vector<DebugBox> m_boxes;
    std::vector<DebugText2D> m_texts2D;

    bool shouldDraw(DebugCategory category) const;
};

}  // namespace smb
