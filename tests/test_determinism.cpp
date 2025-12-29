#include "physics/ball_physics.hpp"
#include "physics/ball_state.hpp"
#include "physics/physics_constants.hpp"

#include <vector>

#include "test_framework.hpp"

// =============================================================================
// Determinism Tests
// =============================================================================
// Verify that identical inputs produce identical outputs.
// This is critical for replay systems and netcode.
//
// Per ROADMAP.md P-010: Same inputs must produce same outputs
// =============================================================================

using namespace smb::physics;

// Helper: run a simulation with given inputs and return final state
// Each call creates fresh state - no carryover from previous calls
static BallState runSimulation(const std::vector<std::pair<float, float>>& inputs) {
    BallState state;
    BallPhysics physics;
    WorldTilt tilt;

    // Ensure clean initial state
    state.reset();
    tilt.reset();

    // Apply each input for one physics tick
    for (const auto& input : inputs) {
        tilt.setTargetFromInput(input.first, input.second);
        tilt.update();
        physics.update(state, tilt, static_cast<float>(TIMESTEP));

        // Simple ground collision
        if (state.position.y < state.radius) {
            state.position.y = state.radius;
            if (state.velocity.y < 0.0f) {
                state.velocity.y = 0.0f;
            }
            state.clearFlag(BallFlags::AIRBORNE);
            state.setFlag(BallFlags::GROUNDED);
        }
    }

    return state;
}

// =============================================================================
// Basic Determinism Tests
// =============================================================================

TEST_CASE("Determinism", "Same inputs produce identical position") {
    std::vector<std::pair<float, float>> inputs;

    // Generate some input sequence
    for (int i = 0; i < 120; i++) {
        float t = static_cast<float>(i) / 60.0f;
        inputs.push_back({std::sin(t), std::cos(t)});
    }

    // Run twice
    BallState result1 = runSimulation(inputs);
    BallState result2 = runSimulation(inputs);

    // Results must be exactly equal
    ASSERT_TRUE(result1.position.approxEquals(result2.position, 0.0f));
    ASSERT_TRUE(result1.velocity.approxEquals(result2.velocity, 0.0f));
}

TEST_CASE("Determinism", "Same inputs produce identical velocity") {
    std::vector<std::pair<float, float>> inputs;

    // Different input pattern
    for (int i = 0; i < 180; i++) {
        float x = (i % 60 < 30) ? 0.5f : -0.5f;
        float y = (i % 120 < 60) ? 0.3f : -0.3f;
        inputs.push_back({x, y});
    }

    BallState result1 = runSimulation(inputs);
    BallState result2 = runSimulation(inputs);

    ASSERT_TRUE(result1.velocity.approxEquals(result2.velocity, 0.0f));
}

TEST_CASE("Determinism", "1000 frames match exactly") {
    std::vector<std::pair<float, float>> inputs;

    // Generate 1000 frames of varying input
    for (int i = 0; i < 1000; i++) {
        float t = static_cast<float>(i) * 0.017f;  // Roughly 60Hz sim of input change
        float x = std::sin(t * 2.3f) * 0.8f;
        float y = std::cos(t * 1.7f) * 0.8f;
        inputs.push_back({x, y});
    }

    BallState result1 = runSimulation(inputs);
    BallState result2 = runSimulation(inputs);

    // Must be identical - using small epsilon for floating point comparison
    // Same code path should produce identical results
    ASSERT_TRUE(result1.position == result2.position);
    ASSERT_TRUE(result1.velocity == result2.velocity);
    ASSERT_EQ(result1.tickCount, result2.tickCount);
}

TEST_CASE("Determinism", "Tick count matches input count") {
    std::vector<std::pair<float, float>> inputs(500, {0.0f, 0.0f});

    BallState result = runSimulation(inputs);

    ASSERT_EQ(result.tickCount, 500u);
}

TEST_CASE("Determinism", "Serialization produces identical results") {
    std::vector<std::pair<float, float>> inputs;

    for (int i = 0; i < 300; i++) {
        inputs.push_back(
            {static_cast<float>(i % 7 - 3) * 0.2f, static_cast<float>(i % 5 - 2) * 0.3f});
    }

    BallState original = runSimulation(inputs);

    // Serialize
    uint8_t buffer[BallState::SERIALIZED_SIZE];
    original.serialize(buffer);

    // Deserialize
    BallState restored;
    restored.deserialize(buffer);

    // Continue simulation from restored state
    std::vector<std::pair<float, float>> moreInputs;
    for (int i = 0; i < 200; i++) {
        moreInputs.push_back({0.3f, -0.2f});
    }

    // Continue from original with fresh tilt state
    BallPhysics physics;
    WorldTilt tilt1;
    tilt1.reset();
    BallState fromOriginal = original;
    for (const auto& input : moreInputs) {
        tilt1.setTargetFromInput(input.first, input.second);
        tilt1.update();
        physics.update(fromOriginal, tilt1, static_cast<float>(TIMESTEP));
        if (fromOriginal.position.y < fromOriginal.radius) {
            fromOriginal.position.y = fromOriginal.radius;
            if (fromOriginal.velocity.y < 0.0f) {
                fromOriginal.velocity.y = 0.0f;
            }
            fromOriginal.clearFlag(BallFlags::AIRBORNE);
            fromOriginal.setFlag(BallFlags::GROUNDED);
        }
    }

    // Continue from restored with fresh tilt state
    WorldTilt tilt2;
    tilt2.reset();
    BallState fromRestored = restored;
    for (const auto& input : moreInputs) {
        tilt2.setTargetFromInput(input.first, input.second);
        tilt2.update();
        physics.update(fromRestored, tilt2, static_cast<float>(TIMESTEP));
        if (fromRestored.position.y < fromRestored.radius) {
            fromRestored.position.y = fromRestored.radius;
            if (fromRestored.velocity.y < 0.0f) {
                fromRestored.velocity.y = 0.0f;
            }
            fromRestored.clearFlag(BallFlags::AIRBORNE);
            fromRestored.setFlag(BallFlags::GROUNDED);
        }
    }

    // Must be identical - using exact comparison since same inputs from same state
    ASSERT_TRUE(fromOriginal.position == fromRestored.position);
    ASSERT_TRUE(fromOriginal.velocity == fromRestored.velocity);
    ASSERT_EQ(fromOriginal.tickCount, fromRestored.tickCount);
}

TEST_CASE("Determinism", "Zero input produces stationary ball") {
    std::vector<std::pair<float, float>> inputs(600, {0.0f, 0.0f});

    BallState result = runSimulation(inputs);

    // With no tilt, ball should be stationary
    ASSERT_TRUE(result.velocity.isNearZero(0.0001f));
}

TEST_CASE("Determinism", "Forward input always moves ball in same direction") {
    for (int trial = 0; trial < 5; trial++) {
        std::vector<std::pair<float, float>> inputs(60, {0.0f, 1.0f});

        BallState result = runSimulation(inputs);

        // Forward input should move ball in -Z direction
        ASSERT_TRUE(result.velocity.z < 0.0f);
    }
}

TEST_CASE("Determinism", "Multiple runs are byte-identical") {
    std::vector<std::pair<float, float>> inputs;
    for (int i = 0; i < 600; i++) {
        inputs.push_back({std::sin(i * 0.1f) * 0.7f, std::cos(i * 0.13f) * 0.7f});
    }

    BallState run1 = runSimulation(inputs);
    BallState run2 = runSimulation(inputs);
    BallState run3 = runSimulation(inputs);

    // All runs must produce identical results
    uint8_t buf1[BallState::SERIALIZED_SIZE];
    uint8_t buf2[BallState::SERIALIZED_SIZE];
    uint8_t buf3[BallState::SERIALIZED_SIZE];

    run1.serialize(buf1);
    run2.serialize(buf2);
    run3.serialize(buf3);

    // Compare byte-by-byte
    bool allMatch = true;
    for (size_t i = 0; i < BallState::SERIALIZED_SIZE; i++) {
        if (buf1[i] != buf2[i] || buf2[i] != buf3[i]) {
            allMatch = false;
            break;
        }
    }

    ASSERT_TRUE(allMatch);
}

// =============================================================================
// WorldTilt Serialization Tests
// =============================================================================

TEST_CASE("Determinism", "WorldTilt serialization round-trip") {
    WorldTilt original;
    original.tiltX = 0.3f;
    original.tiltZ = -0.2f;
    original.targetTiltX = 0.35f;
    original.targetTiltZ = -0.25f;

    // Serialize
    uint8_t buffer[WorldTilt::SERIALIZED_SIZE];
    original.serialize(buffer);

    // Deserialize
    WorldTilt restored;
    restored.deserialize(buffer);

    // Verify
    ASSERT_TRUE(original.approxEquals(restored, 0.0f));
}

TEST_CASE("Determinism", "WorldTilt preserves exact values") {
    WorldTilt original;
    original.setTargetFromInput(0.75f, -0.5f);
    for (int i = 0; i < 50; i++) {
        original.update();
    }

    // Serialize
    uint8_t buffer[WorldTilt::SERIALIZED_SIZE];
    original.serialize(buffer);

    // Deserialize
    WorldTilt restored;
    restored.deserialize(buffer);

    // Values should be exactly equal
    ASSERT_NEAR(original.tiltX, restored.tiltX, 0.0f);
    ASSERT_NEAR(original.tiltZ, restored.tiltZ, 0.0f);
    ASSERT_NEAR(original.targetTiltX, restored.targetTiltX, 0.0f);
    ASSERT_NEAR(original.targetTiltZ, restored.targetTiltZ, 0.0f);
}
