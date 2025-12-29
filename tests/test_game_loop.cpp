#include "game/game_loop.hpp"
#include "platform/platform.hpp"

#include "test_framework.hpp"

// =============================================================================
// Game Loop Tests
// =============================================================================
// Verify fixed timestep loop behavior is correct
// =============================================================================

TEST_CASE("GameLoop", "Physics timestep is 60Hz") {
    // 60 Hz = 1/60 seconds per tick
    ASSERT_NEAR(smb::PHYSICS_TIMESTEP, 1.0 / 60.0, 0.0001);
}

TEST_CASE("GameLoop", "Max accumulator prevents spiral of death") {
    // Should be several frames worth to handle hiccups but not unlimited
    ASSERT_TRUE(smb::MAX_ACCUMULATOR > smb::PHYSICS_TIMESTEP * 2.0);
    ASSERT_TRUE(smb::MAX_ACCUMULATOR < smb::PHYSICS_TIMESTEP * 20.0);
}

TEST_CASE("GameLoop", "Initial stats are zeroed") {
    smb::GameLoop loop;
    loop.initialize();

    const auto& stats = loop.getStats();
    ASSERT_EQ(stats.frameCount, 0u);
    ASSERT_EQ(stats.physicsTickCount, 0u);
}

TEST_CASE("GameLoop", "Exit can be requested") {
    smb::GameLoop loop;
    loop.initialize();

    // Should return true initially
    bool running = loop.tick();
    ASSERT_TRUE(running);

    // Request exit
    loop.requestExit();

    // Should return false after exit requested
    running = loop.tick();
    ASSERT_FALSE(running);
}

TEST_CASE("GameLoop", "Callbacks are called") {
    smb::GameLoop loop;

    int updateCount = 0;
    int renderCount = 0;

    loop.setUpdateCallback([&](double dt) {
        (void)dt;
        updateCount++;
    });

    loop.setRenderCallback([&](double alpha) {
        (void)alpha;
        renderCount++;
    });

    loop.initialize();

    // Run for one frame - should call render at least once
    loop.tick();

    ASSERT_TRUE(renderCount >= 1);
    // Update might be called 0, 1, or more times depending on timing
}

TEST_CASE("GameLoop", "Update receives fixed timestep") {
    smb::GameLoop loop;

    double receivedDt = 0.0;

    loop.setUpdateCallback([&](double dt) { receivedDt = dt; });

    loop.initialize();

    // Force some time to pass so we get at least one physics tick
    smb::platform::sleepSeconds(0.02);

    loop.tick();

    // If update was called, dt should be the fixed timestep
    if (receivedDt > 0.0) {
        ASSERT_NEAR(receivedDt, smb::PHYSICS_TIMESTEP, 0.0001);
    }
}
