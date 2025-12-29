#pragma once

#include <cstdint>
#include <functional>

// =============================================================================
// Fixed Timestep Game Loop
// =============================================================================
// Implements the classic fixed-timestep accumulator pattern. This is absolutely
// critical for deterministic physics - we MUST update physics at a fixed rate
// regardless of frame rate variations.
//
// SMB on GameCube ran at 60Hz for physics. We match that exactly.
//
// The pattern works like this:
// 1. Measure elapsed real time since last frame
// 2. Add that time to an "accumulator"
// 3. While accumulator >= fixed timestep:
//      - Run physics update
//      - Subtract timestep from accumulator
// 4. Render (can interpolate using leftover accumulator for smoothness)
//
// This decouples physics rate from render rate, which is essential for:
// - Determinism (same inputs = same outputs)
// - Replay systems
// - Netcode
// - Not exploding when you alt-tab and come back 5 seconds later
// =============================================================================

namespace smb {

// Physics runs at 60 Hz to match original SMB
// Source: Game runs at 60fps with physics tightly coupled to frame rate
constexpr double PHYSICS_TIMESTEP = 1.0 / 60.0;

// Maximum accumulated time before we start dropping frames.
// Prevents spiral of death when game can't keep up.
// If we're more than 8 frames behind, just give up and slow down.
constexpr double MAX_ACCUMULATOR = PHYSICS_TIMESTEP * 8.0;

// Stats tracked for debugging and performance monitoring
struct GameLoopStats {
    uint64_t frameCount;        // Total frames rendered
    uint64_t physicsTickCount;  // Total physics updates
    double totalTime;           // Total elapsed time in seconds
    double lastFrameTime;       // Time of last frame in seconds
    double lastPhysicsTime;     // Time spent in physics last frame
    double accumulator;         // Current accumulator value
    int physicsTicksThisFrame;  // How many physics updates last frame
    double fps;                 // Smoothed frames per second
    double physicsHz;           // Smoothed physics updates per second
};

class GameLoop {
public:
    // Callback types - the game provides these
    using UpdateCallback = std::function<void(double dt)>;
    using RenderCallback = std::function<void(double alpha)>;

    GameLoop();

    // Sets the function called for each physics tick.
    // dt is always PHYSICS_TIMESTEP (fixed).
    void setUpdateCallback(UpdateCallback callback);

    // Sets the function called for rendering.
    // alpha is interpolation factor [0, 1] for smoothing.
    void setRenderCallback(RenderCallback callback);

    // Call this once at the start of the program.
    void initialize();

    // Call this every frame. Returns false if game should exit.
    // This internally handles the accumulator and calls update/render.
    bool tick();

    // Manually request exit (e.g., from ESC key handler)
    void requestExit();

    // Get current loop statistics for debugging
    const GameLoopStats& getStats() const;

    // Reset the loop timing. Call when unpausing or after loading.
    // Prevents massive accumulator buildup from pause time.
    void resetTiming();

private:
    UpdateCallback m_updateCallback;
    RenderCallback m_renderCallback;
    GameLoopStats m_stats;

    double m_lastTime;
    double m_accumulator;
    double m_fpsAccumulator;
    int m_fpsFrameCount;
    bool m_exitRequested;
    bool m_initialized;

    void updateFPSCounter(double dt);
};

}  // namespace smb
