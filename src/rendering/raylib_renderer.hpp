#pragma once

#include "rendering/renderer.hpp"

// =============================================================================
// Raylib Renderer Implementation
// =============================================================================
// Concrete implementation of Renderer using raylib.
// Handles window creation, 3D/2D rendering, and cleanup.
// =============================================================================

namespace smb {

class RaylibRenderer : public Renderer {
public:
    RaylibRenderer();
    ~RaylibRenderer() override;

    // Lifecycle
    bool initialize(int width, int height, const char* title) override;
    void shutdown() override;
    bool isValid() const override;

    // Frame management
    void beginFrame() override;
    void endFrame() override;
    void clear(Color color) override;

    // 3D Camera
    void begin3D(const CameraData& camera) override;
    void end3D() override;

    // 3D Drawing
    void drawSphere(Vec3 center, float radius, Color color) override;
    void drawSphereWires(Vec3 center, float radius, Color color) override;
    void drawCube(Vec3 center, float width, float height, float length, Color color) override;
    void drawCubeWires(Vec3 center, float width, float height, float length, Color color) override;
    void drawPlane(Vec3 center, float width, float length, Color color) override;
    void drawGrid(int slices, float spacing) override;
    void drawLine3D(Vec3 start, Vec3 end, Color color) override;

    // 2D Drawing
    void drawText(const char* text, int x, int y, int fontSize, Color color) override;
    void drawRect(int x, int y, int width, int height, Color color) override;

    // Screen dimensions
    int getScreenWidth() const override;
    int getScreenHeight() const override;

private:
    bool m_initialized;
    int m_width;
    int m_height;
};

}  // namespace smb
