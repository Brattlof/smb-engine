#include "rendering/debug_draw.hpp"

#include <cmath>

namespace smb {

DebugDraw::DebugDraw(Renderer* renderer)
    : m_renderer(renderer), m_enabled(true), m_enabledCategories(DebugCategory::ALL), m_camera() {}

void DebugDraw::setEnabled(bool enabled) {
    m_enabled = enabled;
}

bool DebugDraw::isEnabled() const {
    return m_enabled;
}

void DebugDraw::setCategoryEnabled(DebugCategory category, bool enabled) {
    if (enabled) {
        m_enabledCategories = m_enabledCategories | category;
    } else {
        m_enabledCategories = static_cast<DebugCategory>(
            static_cast<uint32_t>(m_enabledCategories) & ~static_cast<uint32_t>(category));
    }
}

bool DebugDraw::isCategoryEnabled(DebugCategory category) const {
    return (m_enabledCategories & category) != DebugCategory::NONE;
}

void DebugDraw::toggleCategory(DebugCategory category) {
    setCategoryEnabled(category, !isCategoryEnabled(category));
}

bool DebugDraw::shouldDraw(DebugCategory category) const {
    return m_enabled && isCategoryEnabled(category);
}

void DebugDraw::point(Vec3 pos, Color color, DebugCategory category) {
    // Render point as small sphere
    sphere(pos, 0.05f, color, category);
}

void DebugDraw::line(Vec3 start, Vec3 end, Color color, DebugCategory category) {
    if (!shouldDraw(category))
        return;
    m_lines.push_back({start, end, color, category});
}

void DebugDraw::arrow(Vec3 start, Vec3 end, Color color, DebugCategory category) {
    if (!shouldDraw(category))
        return;

    // Main line
    line(start, end, color, category);

    // Calculate arrowhead
    Vec3 dir = {end.x - start.x, end.y - start.y, end.z - start.z};
    float len = std::sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);

    if (len < 0.001f)
        return;  // Too short to draw

    // Normalize direction
    dir.x /= len;
    dir.y /= len;
    dir.z /= len;

    // Arrowhead size proportional to length but capped
    float headLen = std::fmin(len * 0.2f, 0.3f);
    float headWidth = headLen * 0.5f;

    // Find perpendicular vector (cross with up, or right if parallel to up)
    Vec3 up = {0, 1, 0};
    Vec3 perp;

    float dot = dir.x * up.x + dir.y * up.y + dir.z * up.z;
    if (std::fabs(dot) > 0.99f) {
        // Direction is nearly vertical, use right vector
        up = {1, 0, 0};
    }

    // Cross product: dir x up
    perp.x = dir.y * up.z - dir.z * up.y;
    perp.y = dir.z * up.x - dir.x * up.z;
    perp.z = dir.x * up.y - dir.y * up.x;

    // Normalize
    float perpLen = std::sqrt(perp.x * perp.x + perp.y * perp.y + perp.z * perp.z);
    perp.x /= perpLen;
    perp.y /= perpLen;
    perp.z /= perpLen;

    // Arrowhead base point
    Vec3 base = {end.x - dir.x * headLen, end.y - dir.y * headLen, end.z - dir.z * headLen};

    // Arrowhead wings
    Vec3 wing1 = {base.x + perp.x * headWidth, base.y + perp.y * headWidth,
                  base.z + perp.z * headWidth};
    Vec3 wing2 = {base.x - perp.x * headWidth, base.y - perp.y * headWidth,
                  base.z - perp.z * headWidth};

    line(end, wing1, color, category);
    line(end, wing2, color, category);
}

void DebugDraw::ray(Vec3 origin, Vec3 direction, float length, Color color,
                    DebugCategory category) {
    Vec3 end = {origin.x + direction.x * length, origin.y + direction.y * length,
                origin.z + direction.z * length};
    arrow(origin, end, color, category);
}

void DebugDraw::sphere(Vec3 center, float radius, Color color, DebugCategory category) {
    if (!shouldDraw(category))
        return;
    m_spheres.push_back({center, radius, color, category});
}

void DebugDraw::box(Vec3 center, Vec3 size, Color color, DebugCategory category) {
    if (!shouldDraw(category))
        return;
    m_boxes.push_back({center, size, color, category});
}

void DebugDraw::aabb(Vec3 min, Vec3 max, Color color, DebugCategory category) {
    Vec3 center = {(min.x + max.x) * 0.5f, (min.y + max.y) * 0.5f, (min.z + max.z) * 0.5f};
    Vec3 size = {max.x - min.x, max.y - min.y, max.z - min.z};
    box(center, size, color, category);
}

void DebugDraw::axes(Vec3 origin, float length, DebugCategory category) {
    if (!shouldDraw(category))
        return;

    // X axis - red
    arrow(origin, {origin.x + length, origin.y, origin.z}, Color::red(), category);
    // Y axis - green
    arrow(origin, {origin.x, origin.y + length, origin.z}, Color::green(), category);
    // Z axis - blue
    arrow(origin, {origin.x, origin.y, origin.z + length}, Color::blue(), category);
}

void DebugDraw::groundGrid(float spacing, int lines, Color color, DebugCategory category) {
    if (!shouldDraw(category))
        return;

    float halfSize = spacing * lines * 0.5f;

    // Draw lines along X
    for (int i = -lines / 2; i <= lines / 2; i++) {
        float z = i * spacing;
        line({-halfSize, 0, z}, {halfSize, 0, z}, color, category);
    }

    // Draw lines along Z
    for (int i = -lines / 2; i <= lines / 2; i++) {
        float x = i * spacing;
        line({x, 0, -halfSize}, {x, 0, halfSize}, color, category);
    }
}

void DebugDraw::contact(Vec3 point, Vec3 normal, Color color, DebugCategory category) {
    if (!shouldDraw(category))
        return;

    // Draw contact point
    sphere(point, 0.05f, color, category);

    // Draw normal as arrow
    ray(point, normal, 0.5f, color, category);
}

void DebugDraw::text2D(int x, int y, const char* text, Color color, DebugCategory category) {
    if (!shouldDraw(category))
        return;
    m_texts2D.push_back({x, y, std::string(text), color, category});
}

void DebugDraw::text3D(Vec3 pos, const char* text, Color color, DebugCategory category) {
    // For now, just draw at a fixed 2D position - proper billboarding later
    // This is a placeholder - full 3D text requires more work
    (void)pos;
    (void)text;
    (void)color;
    (void)category;
    // TODO: Implement proper 3D text with camera projection
}

void DebugDraw::begin3D(const CameraData& camera) {
    m_camera = camera;
}

void DebugDraw::render3D() {
    if (!m_enabled)
        return;

    // Render lines
    for (const auto& line : m_lines) {
        m_renderer->drawLine3D(line.start, line.end, line.color);
    }

    // Render spheres
    for (const auto& sphere : m_spheres) {
        m_renderer->drawSphereWires(sphere.center, sphere.radius, sphere.color);
    }

    // Render boxes
    for (const auto& box : m_boxes) {
        m_renderer->drawCubeWires(box.center, box.size.x, box.size.y, box.size.z, box.color);
    }
}

void DebugDraw::end3D() {
    // Nothing special needed here for now
}

void DebugDraw::render2D() {
    if (!m_enabled)
        return;

    for (const auto& text : m_texts2D) {
        m_renderer->drawText(text.text.c_str(), text.x, text.y, 14, text.color);
    }
}

void DebugDraw::clear() {
    m_lines.clear();
    m_spheres.clear();
    m_boxes.clear();
    m_texts2D.clear();
}

}  // namespace smb
