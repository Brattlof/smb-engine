#include "platform/platform.hpp"

#include <chrono>
#include <thread>

// =============================================================================
// Platform-specific includes
// =============================================================================

#if defined(SMB_PLATFORM_WINDOWS)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#elif defined(SMB_PLATFORM_MACOS) || defined(SMB_PLATFORM_LINUX)
#include <time.h>
#include <unistd.h>
#if defined(SMB_PLATFORM_MACOS)
#include <mach/mach_time.h>
#endif
#endif

namespace smb {
namespace platform {

// =============================================================================
// Windows Implementation
// =============================================================================

#if defined(SMB_PLATFORM_WINDOWS)

// Cache the frequency - it doesn't change at runtime
static int64_t s_timerFrequency = 0;
static int64_t s_startTicks = 0;
static bool s_timerInitialized = false;

static void initializeTimer() {
    if (s_timerInitialized)
        return;

    LARGE_INTEGER freq;
    QueryPerformanceFrequency(&freq);
    s_timerFrequency = freq.QuadPart;

    LARGE_INTEGER start;
    QueryPerformanceCounter(&start);
    s_startTicks = start.QuadPart;

    s_timerInitialized = true;
}

int64_t getTimerFrequency() {
    initializeTimer();
    return s_timerFrequency;
}

int64_t getTimerTicks() {
    initializeTimer();
    LARGE_INTEGER ticks;
    QueryPerformanceCounter(&ticks);
    return ticks.QuadPart;
}

double getTimeSeconds() {
    initializeTimer();
    int64_t current = getTimerTicks();
    return static_cast<double>(current - s_startTicks) / static_cast<double>(s_timerFrequency);
}

void sleepSeconds(double seconds) {
    if (seconds <= 0.0)
        return;
    // Windows Sleep takes milliseconds
    DWORD ms = static_cast<DWORD>(seconds * 1000.0);
    if (ms > 0) {
        Sleep(ms);
    }
}

void debugBreak() {
    if (IsDebuggerPresent()) {
        DebugBreak();
    }
}

// =============================================================================
// macOS Implementation
// =============================================================================

#elif defined(SMB_PLATFORM_MACOS)

static mach_timebase_info_data_t s_timebaseInfo;
static uint64_t s_startTicks = 0;
static bool s_timerInitialized = false;

static void initializeTimer() {
    if (s_timerInitialized)
        return;

    mach_timebase_info(&s_timebaseInfo);
    s_startTicks = mach_absolute_time();

    s_timerInitialized = true;
}

int64_t getTimerFrequency() {
    initializeTimer();
    // Convert nanoseconds to "frequency" - this is a bit weird on macOS
    // since mach_absolute_time uses a ratio, not a frequency
    return static_cast<int64_t>(1e9 * s_timebaseInfo.denom / s_timebaseInfo.numer);
}

int64_t getTimerTicks() {
    initializeTimer();
    return static_cast<int64_t>(mach_absolute_time());
}

double getTimeSeconds() {
    initializeTimer();
    uint64_t elapsed = mach_absolute_time() - s_startTicks;
    // Convert to nanoseconds then to seconds
    uint64_t nanos = elapsed * s_timebaseInfo.numer / s_timebaseInfo.denom;
    return static_cast<double>(nanos) / 1e9;
}

void sleepSeconds(double seconds) {
    if (seconds <= 0.0)
        return;
    usleep(static_cast<useconds_t>(seconds * 1e6));
}

void debugBreak() {
    // macOS: raise SIGTRAP
    __builtin_trap();
}

// =============================================================================
// Linux Implementation
// =============================================================================

#elif defined(SMB_PLATFORM_LINUX)

static int64_t s_startNanos = 0;
static bool s_timerInitialized = false;

static int64_t getMonotonicNanos() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

static void initializeTimer() {
    if (s_timerInitialized)
        return;
    s_startNanos = getMonotonicNanos();
    s_timerInitialized = true;
}

int64_t getTimerFrequency() {
    // Linux CLOCK_MONOTONIC is in nanoseconds
    return 1000000000LL;
}

int64_t getTimerTicks() {
    return getMonotonicNanos();
}

double getTimeSeconds() {
    initializeTimer();
    int64_t elapsed = getMonotonicNanos() - s_startNanos;
    return static_cast<double>(elapsed) / 1e9;
}

void sleepSeconds(double seconds) {
    if (seconds <= 0.0)
        return;
    usleep(static_cast<useconds_t>(seconds * 1e6));
}

void debugBreak() {
    // Linux: raise SIGTRAP
    __builtin_trap();
}

#endif

// =============================================================================
// Common Implementation
// =============================================================================

const char* getPlatformName() {
    return SMB_PLATFORM_NAME;
}

}  // namespace platform
}  // namespace smb
