#include "audio/audio_player.hpp"
#include "game/game_loop.hpp"
#include "input/raylib_input_reader.hpp"
#include "physics/ball_physics.hpp"
#include "physics/ball_state.hpp"
#include "platform/platform.hpp"
#include "rendering/debug_draw.hpp"
#include "rendering/raylib_renderer.hpp"
#include "util/logger.hpp"

#include <cmath>
#include <cstdio>

// =============================================================================
// SMB Physics Engine - Entry Point
// =============================================================================
// This sets up the game infrastructure and runs the main loop.
// Physics timestep is fixed at 60Hz per SMB original.
// =============================================================================

// Forward declarations for game callbacks
static void onPhysicsUpdate(double dt);
static void onRender(double alpha);

// Global systems - yeah, globals are gross, but this is the main entry point
// and these are true singletons. Fight me.
static smb::RaylibRenderer* g_renderer = nullptr;
static smb::RaylibInputReader* g_inputReader = nullptr;
static smb::NullAudioPlayer* g_audioPlayer = nullptr;
static smb::GameLoop* g_gameLoop = nullptr;
static smb::DebugDraw* g_debugDraw = nullptr;

// Physics state - the actual simulation
static smb::physics::BallState g_ballState;
static smb::physics::BallPhysics g_ballPhysics;
static smb::physics::WorldTilt g_worldTilt;

int main() {
    using namespace smb;

    // Initialize logger first
    Logger::setLevel(LogLevel::DEBUG);
    Logger::info("main", "SMB Physics Engine starting up...");
    Logger::info("main", "Platform: %s", platform::getPlatformName());

    // Create systems
    g_renderer = new RaylibRenderer();
    g_inputReader = new RaylibInputReader();
    g_audioPlayer = new NullAudioPlayer();
    g_gameLoop = new GameLoop();

    // Initialize renderer (creates window)
    if (!g_renderer->initialize(1280, 720, "SMB Physics Engine")) {
        Logger::fatal("main", "Failed to initialize renderer");
        return 1;
    }

    // Create debug drawer after renderer
    g_debugDraw = new DebugDraw(g_renderer);

    // Initialize other systems
    g_audioPlayer->initialize();
    g_gameLoop->initialize();

    // Set up callbacks
    g_gameLoop->setUpdateCallback(onPhysicsUpdate);
    g_gameLoop->setRenderCallback(onRender);

    Logger::info("main", "All systems initialized. Entering main loop...");

    // Main loop
    while (g_gameLoop->tick()) {
        // Poll input
        g_inputReader->poll();

        // Check for close request
        if (g_inputReader->isCloseRequested()) {
            g_gameLoop->requestExit();
        }

        // Toggle debug display
        if (g_inputReader->getState().debugTogglePressed) {
            g_debugDraw->setEnabled(!g_debugDraw->isEnabled());
            Logger::debug("main", "Debug display: %s", g_debugDraw->isEnabled() ? "ON" : "OFF");
        }
    }

    Logger::info("main", "Main loop exited. Shutting down...");

    // Cleanup in reverse order
    g_audioPlayer->shutdown();
    g_renderer->shutdown();

    delete g_debugDraw;
    delete g_gameLoop;
    delete g_audioPlayer;
    delete g_inputReader;
    delete g_renderer;

    Logger::info("main", "Shutdown complete. Goodbye!");
    return 0;
}

// Called at fixed 60Hz for physics
static void onPhysicsUpdate(double dt) {
    using namespace smb;

    // Get input and update world tilt target
    const auto& input = g_inputReader->getState();
    g_worldTilt.setTargetFromInput(input.tiltX, input.tiltY);

    // Smooth tilt towards target
    g_worldTilt.update();

    // Run physics simulation
    g_ballPhysics.update(g_ballState, g_worldTilt, static_cast<float>(dt));

    // Simple ground collision: keep ball on the floor (y >= radius)
    // This is placeholder until proper collision is implemented
    if (g_ballState.position.y < g_ballState.radius) {
        g_ballState.position.y = g_ballState.radius;

        // If we hit the ground, zero vertical velocity and set grounded
        if (g_ballState.velocity.y < 0.0f) {
            g_ballState.velocity.y = 0.0f;
        }

        g_ballState.clearFlag(physics::BallFlags::AIRBORNE);
        g_ballState.setFlag(physics::BallFlags::GROUNDED);
    }

    // Simple boundary: reset if ball goes too far
    const float BOUNDARY = 15.0f;
    if (std::fabs(g_ballState.position.x) > BOUNDARY ||
        std::fabs(g_ballState.position.z) > BOUNDARY ||
        g_ballState.position.y < -5.0f) {
        Logger::debug("physics", "Ball out of bounds, resetting");
        g_ballState.reset();
        g_worldTilt.reset();
    }

    // Reset ball on R key
    if (g_inputReader->getState().debugResetPressed) {
        Logger::info("physics", "Manual reset triggered");
        g_ballState.reset();
        g_worldTilt.reset();
    }
}

// Called every frame for rendering
static void onRender(double alpha) {
    using namespace smb;

    (void)alpha;  // Will use for interpolation later

    g_renderer->beginFrame();
    g_renderer->clear(Color::rgb(40, 44, 52));  // Dark background

    // Set up camera
    CameraData camera;
    camera.position = Vec3(0.0f, 10.0f, 15.0f);
    camera.target = Vec3(0.0f, 0.0f, 0.0f);
    camera.up = Vec3(0.0f, 1.0f, 0.0f);
    camera.fovY = 45.0f;

    g_renderer->begin3D(camera);

    // Draw a ground plane
    g_renderer->drawPlane(Vec3(0, 0, 0), 20.0f, 20.0f, Color::rgb(60, 60, 70));

    // Convert physics position to rendering position
    Vec3 ballRenderPos(g_ballState.position.x, g_ballState.position.y, g_ballState.position.z);

    // Draw the ball at its physics position
    g_renderer->drawSphere(ballRenderPos, g_ballState.radius, Color::red());

    // Debug drawing (in 3D context)
    g_debugDraw->begin3D(camera);

    // Draw coordinate axes at origin using debug system
    g_debugDraw->axes(Vec3(0, 0, 0), 3.0f, DebugCategory::WORLD);

    // Draw wireframe around ball showing collision volume
    g_debugDraw->sphere(ballRenderPos, g_ballState.radius + 0.02f, Color::white(),
                        DebugCategory::PHYSICS);

    // Draw velocity vector from ball
    if (g_ballState.getSpeed() > 0.01f) {
        Vec3 velEnd(ballRenderPos.x + g_ballState.velocity.x,
                    ballRenderPos.y + g_ballState.velocity.y,
                    ballRenderPos.z + g_ballState.velocity.z);
        g_debugDraw->arrow(ballRenderPos, velEnd, Color::yellow(), DebugCategory::PHYSICS);
    }

    // Draw tilt visualization - show effective gravity direction
    physics::Vec3 physGravDir = g_ballPhysics.getEffectiveGravityDirection(g_worldTilt);
    Vec3 gravEnd(0.0f + physGravDir.x * 2.0f, 2.0f + physGravDir.y * 2.0f,
                 0.0f + physGravDir.z * 2.0f);
    g_debugDraw->arrow(Vec3(0, 2, 0), gravEnd, Color::green(), DebugCategory::WORLD);

    // Draw grid using debug system
    g_debugDraw->groundGrid(1.0f, 20, Color::gray(), DebugCategory::WORLD);

    // Render all debug 3D stuff
    g_debugDraw->render3D();

    g_debugDraw->end3D();

    g_renderer->end3D();

    // 2D overlay
    g_renderer->drawText("SMB Physics Engine", 10, 10, 20, Color::white());

    if (g_debugDraw->isEnabled()) {
        const auto& stats = g_gameLoop->getStats();
        const auto& input = g_inputReader->getState();

        char buffer[256];

        std::snprintf(buffer, sizeof(buffer), "FPS: %.1f", stats.fps);
        g_renderer->drawText(buffer, 10, 40, 16, Color::green());

        std::snprintf(buffer, sizeof(buffer), "Physics: %.1f Hz (%d ticks/frame)", stats.physicsHz,
                      stats.physicsTicksThisFrame);
        g_renderer->drawText(buffer, 10, 60, 16, Color::green());

        std::snprintf(buffer, sizeof(buffer), "Frame: %llu  Ticks: %llu", stats.frameCount,
                      stats.physicsTickCount);
        g_renderer->drawText(buffer, 10, 80, 16, Color::gray());

        std::snprintf(buffer, sizeof(buffer), "Input: X=%.2f Y=%.2f", input.tiltX, input.tiltY);
        g_renderer->drawText(buffer, 10, 100, 16, Color::yellow());

        // Physics state display
        std::snprintf(buffer, sizeof(buffer), "Position: (%.2f, %.2f, %.2f)",
                      g_ballState.position.x, g_ballState.position.y, g_ballState.position.z);
        g_renderer->drawText(buffer, 10, 130, 14, Color::white());

        std::snprintf(buffer, sizeof(buffer), "Velocity: (%.2f, %.2f, %.2f) Speed: %.2f",
                      g_ballState.velocity.x, g_ballState.velocity.y, g_ballState.velocity.z,
                      g_ballState.getSpeed());
        g_renderer->drawText(buffer, 10, 150, 14, Color::white());

        std::snprintf(buffer, sizeof(buffer), "Tilt: X=%.2f deg  Z=%.2f deg",
                      g_worldTilt.tiltX * 180.0f / 3.14159f,
                      g_worldTilt.tiltZ * 180.0f / 3.14159f);
        g_renderer->drawText(buffer, 10, 170, 14, Color::white());

        std::snprintf(buffer, sizeof(buffer), "Physics Tick: %llu  %s",
                      g_ballState.tickCount,
                      g_ballState.isGrounded() ? "[GROUNDED]" : "[AIRBORNE]");
        g_renderer->drawText(buffer, 10, 190, 14, Color::white());

        g_renderer->drawText("[F1] Toggle debug | [WASD/Arrows] Tilt | [F5] Reset", 10,
                             g_renderer->getScreenHeight() - 30, 14, Color::gray());

        // Render debug 2D text
        g_debugDraw->render2D();
    }

    // Clear debug draw queue for next frame
    g_debugDraw->clear();

    g_renderer->endFrame();
}
