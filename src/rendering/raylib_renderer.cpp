#include "rendering/raylib_renderer.hpp"

#include "util/logger.hpp"

#include <raylib.h>

namespace smb {

// Helper to convert our Color to raylib Color
static ::Color toRaylibColor(smb::Color c) {
    return ::Color{c.r, c.g, c.b, c.a};
}

// Helper to convert our Vec3 to raylib Vector3
static Vector3 toRaylibVec3(Vec3 v) {
    return Vector3{v.x, v.y, v.z};
}

RaylibRenderer::RaylibRenderer() : m_initialized(false), m_width(0), m_height(0) {}

RaylibRenderer::~RaylibRenderer() {
    if (m_initialized) {
        shutdown();
    }
}

bool RaylibRenderer::initialize(int width, int height, const char* title) {
    if (m_initialized) {
        Logger::warn("renderer", "Already initialized");
        return true;
    }

    // Don't let raylib set its own target FPS - we handle timing ourselves
    SetConfigFlags(FLAG_VSYNC_HINT);

    InitWindow(width, height, title);

    if (!IsWindowReady()) {
        Logger::error("renderer", "Failed to create window");
        return false;
    }

    m_width = width;
    m_height = height;
    m_initialized = true;

    Logger::info("renderer", "Window created: %dx%d - %s", width, height, title);
    return true;
}

void RaylibRenderer::shutdown() {
    if (!m_initialized) {
        return;
    }

    CloseWindow();
    m_initialized = false;
    Logger::info("renderer", "Window closed");
}

bool RaylibRenderer::isValid() const {
    return m_initialized && IsWindowReady();
}

void RaylibRenderer::beginFrame() {
    BeginDrawing();
}

void RaylibRenderer::endFrame() {
    EndDrawing();
}

void RaylibRenderer::clear(Color color) {
    ClearBackground(toRaylibColor(color));
}

void RaylibRenderer::begin3D(const CameraData& camera) {
    Camera3D cam = {};
    cam.position = toRaylibVec3(camera.position);
    cam.target = toRaylibVec3(camera.target);
    cam.up = toRaylibVec3(camera.up);
    cam.fovy = camera.fovY;
    cam.projection = CAMERA_PERSPECTIVE;

    BeginMode3D(cam);
}

void RaylibRenderer::end3D() {
    EndMode3D();
}

void RaylibRenderer::drawSphere(Vec3 center, float radius, Color color) {
    // 16 rings and 16 slices - decent quality for a ball
    DrawSphere(toRaylibVec3(center), radius, toRaylibColor(color));
}

void RaylibRenderer::drawSphereWires(Vec3 center, float radius, Color color) {
    DrawSphereWires(toRaylibVec3(center), radius, 8, 8, toRaylibColor(color));
}

void RaylibRenderer::drawCube(Vec3 center, float width, float height, float length, Color color) {
    DrawCube(toRaylibVec3(center), width, height, length, toRaylibColor(color));
}

void RaylibRenderer::drawCubeWires(Vec3 center, float width, float height, float length,
                                   Color color) {
    DrawCubeWires(toRaylibVec3(center), width, height, length, toRaylibColor(color));
}

void RaylibRenderer::drawPlane(Vec3 center, float width, float length, Color color) {
    DrawPlane(toRaylibVec3(center), Vector2{width, length}, toRaylibColor(color));
}

void RaylibRenderer::drawGrid(int slices, float spacing) {
    DrawGrid(slices, spacing);
}

void RaylibRenderer::drawLine3D(Vec3 start, Vec3 end, Color color) {
    DrawLine3D(toRaylibVec3(start), toRaylibVec3(end), toRaylibColor(color));
}

void RaylibRenderer::drawText(const char* text, int x, int y, int fontSize, Color color) {
    DrawText(text, x, y, fontSize, toRaylibColor(color));
}

void RaylibRenderer::drawRect(int x, int y, int width, int height, Color color) {
    DrawRectangle(x, y, width, height, toRaylibColor(color));
}

int RaylibRenderer::getScreenWidth() const {
    return m_width;
}

int RaylibRenderer::getScreenHeight() const {
    return m_height;
}

}  // namespace smb
