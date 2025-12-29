#include "game/game_loop.hpp"

#include "platform/platform.hpp"
#include "util/logger.hpp"

namespace smb {

GameLoop::GameLoop()
    : m_updateCallback(nullptr),
      m_renderCallback(nullptr),
      m_stats{},
      m_lastTime(0.0),
      m_accumulator(0.0),
      m_fpsAccumulator(0.0),
      m_fpsFrameCount(0),
      m_exitRequested(false),
      m_initialized(false) {}

void GameLoop::setUpdateCallback(UpdateCallback callback) {
    m_updateCallback = callback;
}

void GameLoop::setRenderCallback(RenderCallback callback) {
    m_renderCallback = callback;
}

void GameLoop::initialize() {
    if (m_initialized) {
        Logger::warn("game_loop", "initialize() called twice - ignoring");
        return;
    }

    m_lastTime = platform::getTimeSeconds();
    m_accumulator = 0.0;
    m_fpsAccumulator = 0.0;
    m_fpsFrameCount = 0;
    m_exitRequested = false;
    m_stats = {};

    m_initialized = true;

    Logger::info("game_loop", "Fixed timestep loop initialized at %.2f Hz", 1.0 / PHYSICS_TIMESTEP);
    Logger::info("game_loop", "Platform: %s, Timer frequency: %lld Hz", platform::getPlatformName(),
                 platform::getTimerFrequency());
}

bool GameLoop::tick() {
    if (!m_initialized) {
        Logger::error("game_loop", "tick() called before initialize()!");
        return false;
    }

    if (m_exitRequested) {
        return false;
    }

    // Measure elapsed time
    double currentTime = platform::getTimeSeconds();
    double frameTime = currentTime - m_lastTime;
    m_lastTime = currentTime;

    // Clamp frame time to prevent spiral of death
    // If we somehow get a massive dt (alt-tab, breakpoint, etc.), just cap it
    if (frameTime > MAX_ACCUMULATOR) {
        Logger::warn("game_loop", "Frame time clamped: %.3fs -> %.3fs (did you alt-tab?)",
                     frameTime, MAX_ACCUMULATOR);
        frameTime = MAX_ACCUMULATOR;
    }

    // Negative time? That's bad. Probably a timer bug.
    if (frameTime < 0.0) {
        Logger::error("game_loop", "Negative frame time detected: %.6fs - resetting", frameTime);
        frameTime = PHYSICS_TIMESTEP;
    }

    m_stats.lastFrameTime = frameTime;
    m_stats.totalTime += frameTime;

    // Add to accumulator
    m_accumulator += frameTime;

    // Run physics updates until we've caught up
    int physicsUpdates = 0;
    double physicsStartTime = platform::getTimeSeconds();

    while (m_accumulator >= PHYSICS_TIMESTEP) {
        if (m_updateCallback) {
            m_updateCallback(PHYSICS_TIMESTEP);
        }

        m_accumulator -= PHYSICS_TIMESTEP;
        m_stats.physicsTickCount++;
        physicsUpdates++;

        // Safety valve - if we're doing way too many updates, something is wrong
        if (physicsUpdates > 10) {
            Logger::warn("game_loop", "Too many physics updates (%d) - clamping accumulator",
                         physicsUpdates);
            m_accumulator = 0.0;
            break;
        }
    }

    m_stats.lastPhysicsTime = platform::getTimeSeconds() - physicsStartTime;
    m_stats.physicsTicksThisFrame = physicsUpdates;
    m_stats.accumulator = m_accumulator;

    // Render with interpolation alpha
    // alpha = how far into the next physics frame we are
    double alpha = m_accumulator / PHYSICS_TIMESTEP;
    if (m_renderCallback) {
        m_renderCallback(alpha);
    }

    m_stats.frameCount++;
    updateFPSCounter(frameTime);

    return true;
}

void GameLoop::requestExit() {
    m_exitRequested = true;
    Logger::info("game_loop", "Exit requested");
}

const GameLoopStats& GameLoop::getStats() const {
    return m_stats;
}

void GameLoop::resetTiming() {
    m_lastTime = platform::getTimeSeconds();
    m_accumulator = 0.0;
    Logger::debug("game_loop", "Timing reset");
}

void GameLoop::updateFPSCounter(double dt) {
    m_fpsAccumulator += dt;
    m_fpsFrameCount++;

    // Update FPS every half second for smoother display
    if (m_fpsAccumulator >= 0.5) {
        m_stats.fps = static_cast<double>(m_fpsFrameCount) / m_fpsAccumulator;
        m_stats.physicsHz = m_stats.fps * m_stats.physicsTicksThisFrame;
        m_fpsAccumulator = 0.0;
        m_fpsFrameCount = 0;
    }
}

}  // namespace smb
