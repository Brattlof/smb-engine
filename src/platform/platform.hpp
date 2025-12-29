#pragma once

#include <cstdint>

// =============================================================================
// Platform Abstraction Layer
// =============================================================================
// Abstracts platform-specific functionality. The physics engine and game logic
// MUST NOT include platform headers directly - use this interface instead.
//
// Currently wraps:
//   - High-resolution timing (for fixed timestep game loop)
//   - Platform detection macros
//
// We keep this thin - raylib already handles most platform stuff for us.
// This is mainly for deterministic timing independent of raylib's internals.
// =============================================================================

namespace smb {
namespace platform {

// =============================================================================
// Platform Detection
// =============================================================================
// One and only one of these should be defined

#if defined(_WIN32) || defined(_WIN64)
#define SMB_PLATFORM_WINDOWS 1
#define SMB_PLATFORM_NAME "Windows"
#elif defined(__APPLE__) && defined(__MACH__)
#define SMB_PLATFORM_MACOS 1
#define SMB_PLATFORM_NAME "macOS"
#elif defined(__linux__)
#define SMB_PLATFORM_LINUX 1
#define SMB_PLATFORM_NAME "Linux"
#else
#error "Unsupported platform - we need Windows, macOS, or Linux"
#endif

// =============================================================================
// High-Resolution Timer
// =============================================================================
// We roll our own timing instead of relying on raylib's GetTime() because:
// 1. We need guaranteed precision for deterministic physics
// 2. We want to control exactly when time is sampled
// 3. Easier to test and mock
//
// All times are in seconds as double-precision floating point.

// Returns current time in seconds since some fixed point (usually program start).
// Monotonic - never goes backwards.
double getTimeSeconds();

// Returns the frequency of the high-resolution timer in ticks per second.
// Used internally, exposed mainly for debugging.
int64_t getTimerFrequency();

// Returns raw timer ticks. Use for computing deltas when you need max precision.
int64_t getTimerTicks();

// Sleep for approximately the given duration in seconds.
// Not guaranteed to be exact - don't use this for physics timing.
// Useful for reducing CPU usage when waiting.
void sleepSeconds(double seconds);

// =============================================================================
// Debug Break
// =============================================================================
// Triggers a debugger breakpoint if attached, otherwise does nothing useful.
// Call this when you hit an "impossible" state you want to investigate.

void debugBreak();

// =============================================================================
// Platform Info
// =============================================================================
// Returns the platform name as a string (Windows, macOS, Linux)
const char* getPlatformName();

}  // namespace platform
}  // namespace smb
