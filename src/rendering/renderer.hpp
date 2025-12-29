#pragma once

#include <cstdint>

// =============================================================================
// Renderer Interface
// =============================================================================
// Abstract interface for rendering. Game and physics code use this instead
// of calling raylib directly. All rendering goes through here.
//
// Note: This is a fairly thin abstraction for now. As the project grows,
// we might want to batch draw calls, implement a scene graph, etc.
// But YAGNI - keep it simple until we need more.
// =============================================================================

namespace smb {

// Simple color struct - we don't want raylib types leaking into game code
struct Color {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t a;

    static Color rgb(uint8_t r, uint8_t g, uint8_t b) { return Color{r, g, b, 255}; }

    static Color rgba(uint8_t r, uint8_t g, uint8_t b, uint8_t a) { return Color{r, g, b, a}; }

    // Some common colors
    static Color white() { return rgb(255, 255, 255); }
    static Color black() { return rgb(0, 0, 0); }
    static Color red() { return rgb(230, 41, 55); }
    static Color green() { return rgb(0, 228, 48); }
    static Color blue() { return rgb(0, 121, 241); }
    static Color yellow() { return rgb(253, 249, 0); }
    static Color orange() { return rgb(255, 161, 0); }
    static Color purple() { return rgb(200, 122, 255); }
    static Color gray() { return rgb(130, 130, 130); }
    static Color darkGray() { return rgb(80, 80, 80); }
};

// Basic 3D vector for positions - keep our own to avoid raylib dependency
struct Vec3 {
    float x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

// Basic 2D vector for screen positions
struct Vec2 {
    float x, y;

    Vec2() : x(0), y(0) {}
    Vec2(float x_, float y_) : x(x_), y(y_) {}
};

// Camera data for 3D rendering
struct CameraData {
    Vec3 position;
    Vec3 target;
    Vec3 up;
    float fovY;  // Field of view in degrees

    CameraData() : position(0, 10, 10), target(0, 0, 0), up(0, 1, 0), fovY(45.0f) {}
};

class Renderer {
public:
    virtual ~Renderer() = default;

    // =========================================================================
    // Lifecycle
    // =========================================================================

    // Initialize the renderer (create window, etc.)
    virtual bool initialize(int width, int height, const char* title) = 0;

    // Shutdown and cleanup
    virtual void shutdown() = 0;

    // Returns true if the window is still valid
    virtual bool isValid() const = 0;

    // =========================================================================
    // Frame Management
    // =========================================================================

    // Begin a new frame
    virtual void beginFrame() = 0;

    // End the current frame and present to screen
    virtual void endFrame() = 0;

    // Clear the screen with a color
    virtual void clear(Color color) = 0;

    // =========================================================================
    // 3D Camera
    // =========================================================================

    // Begin 3D rendering mode with the given camera
    virtual void begin3D(const CameraData& camera) = 0;

    // End 3D rendering mode
    virtual void end3D() = 0;

    // =========================================================================
    // 3D Drawing
    // =========================================================================

    // Draw a sphere (used for the ball)
    virtual void drawSphere(Vec3 center, float radius, Color color) = 0;

    // Draw a wireframe sphere (for debug)
    virtual void drawSphereWires(Vec3 center, float radius, Color color) = 0;

    // Draw a cube
    virtual void drawCube(Vec3 center, float width, float height, float length, Color color) = 0;

    // Draw a wireframe cube (for debug)
    virtual void drawCubeWires(Vec3 center, float width, float height, float length,
                               Color color) = 0;

    // Draw a plane (finite rectangle on the ground)
    virtual void drawPlane(Vec3 center, float width, float length, Color color) = 0;

    // Draw a grid on the XZ plane (for debug)
    virtual void drawGrid(int slices, float spacing) = 0;

    // Draw a line in 3D space
    virtual void drawLine3D(Vec3 start, Vec3 end, Color color) = 0;

    // =========================================================================
    // 2D Drawing (for HUD, debug text, etc.)
    // =========================================================================

    // Draw text on screen
    virtual void drawText(const char* text, int x, int y, int fontSize, Color color) = 0;

    // Draw a 2D rectangle
    virtual void drawRect(int x, int y, int width, int height, Color color) = 0;

    // Get screen dimensions
    virtual int getScreenWidth() const = 0;
    virtual int getScreenHeight() const = 0;
};

}  // namespace smb
