#include "platform/platform.hpp"

#include "test_framework.hpp"

// =============================================================================
// Platform Layer Tests
// =============================================================================
// Verify the timing functions work correctly and provide deterministic behavior
// =============================================================================

TEST_CASE("Platform", "Timer frequency is positive") {
    int64_t freq = smb::platform::getTimerFrequency();
    ASSERT_TRUE(freq > 0);
}

TEST_CASE("Platform", "Time is monotonic") {
    double t1 = smb::platform::getTimeSeconds();
    double t2 = smb::platform::getTimeSeconds();
    double t3 = smb::platform::getTimeSeconds();

    ASSERT_TRUE(t2 >= t1);
    ASSERT_TRUE(t3 >= t2);
}

TEST_CASE("Platform", "Time advances") {
    double start = smb::platform::getTimeSeconds();

    // Busy wait for a tiny bit
    volatile int counter = 0;
    for (int i = 0; i < 100000; i++) {
        counter++;
    }

    double end = smb::platform::getTimeSeconds();
    ASSERT_TRUE(end > start);
}

TEST_CASE("Platform", "Timer ticks are increasing") {
    int64_t t1 = smb::platform::getTimerTicks();
    int64_t t2 = smb::platform::getTimerTicks();

    ASSERT_TRUE(t2 >= t1);
}

TEST_CASE("Platform", "Platform name is valid") {
    const char* name = smb::platform::getPlatformName();
    ASSERT_TRUE(name != nullptr);

    // Should be one of Windows, macOS, or Linux
    bool validName = (std::strcmp(name, "Windows") == 0 || std::strcmp(name, "macOS") == 0 ||
                      std::strcmp(name, "Linux") == 0);
    ASSERT_TRUE(validName);
}

TEST_CASE("Platform", "Sleep does not go backwards in time") {
    double before = smb::platform::getTimeSeconds();
    smb::platform::sleepSeconds(0.001);  // 1ms
    double after = smb::platform::getTimeSeconds();

    ASSERT_TRUE(after >= before);
}
