#pragma once

#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <string>

// =============================================================================
// Logger - Severity-based logging with context tagging
// =============================================================================
// Provides centralized logging with severity levels and module context.
// All physics and game systems should use this for consistent debug output.
//
// Usage:
//   Logger::info("physics", "Ball velocity: %.2f", vel);
//   Logger::warn("camera", "FOV clamped to maximum");
//   Logger::error("collision", "Degenerate triangle detected");
// =============================================================================

namespace smb {

enum class LogLevel {
    TRACE = 0,  // Extremely verbose, frame-by-frame data
    DEBUG = 1,  // Development info, not for release
    INFO = 2,   // General runtime information
    WARN = 3,   // Something unexpected but recoverable
    ERROR = 4,  // Something went wrong
    FATAL = 5,  // Unrecoverable, application will terminate
    OFF = 6     // Disable all logging
};

// The minimum level that will actually be printed.
// Anything below this gets compiled out in release builds.
#ifdef NDEBUG
constexpr LogLevel COMPILE_TIME_MIN_LEVEL = LogLevel::INFO;
#else
constexpr LogLevel COMPILE_TIME_MIN_LEVEL = LogLevel::TRACE;
#endif

class Logger {
public:
    // Sets the runtime minimum log level. Messages below this are discarded.
    static void setLevel(LogLevel level) { s_runtimeLevel = level; }

    static LogLevel getLevel() { return s_runtimeLevel; }

    // Logs a TRACE message - extremely verbose, frame-by-frame data
    template <typename... Args>
    static void trace(const char* context, const char* fmt, Args... args) {
        log(LogLevel::TRACE, context, fmt, args...);
    }

    // Logs a DEBUG message - development info
    template <typename... Args>
    static void debug(const char* context, const char* fmt, Args... args) {
        log(LogLevel::DEBUG, context, fmt, args...);
    }

    // Logs an INFO message - general runtime information
    template <typename... Args>
    static void info(const char* context, const char* fmt, Args... args) {
        log(LogLevel::INFO, context, fmt, args...);
    }

    // Logs a WARN message - unexpected but recoverable
    template <typename... Args>
    static void warn(const char* context, const char* fmt, Args... args) {
        log(LogLevel::WARN, context, fmt, args...);
    }

    // Logs an ERROR message - something went wrong
    template <typename... Args>
    static void error(const char* context, const char* fmt, Args... args) {
        log(LogLevel::ERROR, context, fmt, args...);
    }

    // Logs a FATAL message - unrecoverable, probably about to crash
    template <typename... Args>
    static void fatal(const char* context, const char* fmt, Args... args) {
        log(LogLevel::FATAL, context, fmt, args...);
    }

private:
    static inline LogLevel s_runtimeLevel = LogLevel::TRACE;

    static const char* levelToString(LogLevel level) {
        switch (level) {
        case LogLevel::TRACE:
            return "TRACE";
        case LogLevel::DEBUG:
            return "DEBUG";
        case LogLevel::INFO:
            return "INFO ";
        case LogLevel::WARN:
            return "WARN ";
        case LogLevel::ERROR:
            return "ERROR";
        case LogLevel::FATAL:
            return "FATAL";
        default:
            return "?????";
        }
    }

    // Core logging function. Printf-style formatting.
    template <typename... Args>
    static void log(LogLevel level, const char* context, const char* fmt, Args... args) {
        // Compile-time check - lower levels get optimized out in release
        if constexpr (static_cast<int>(COMPILE_TIME_MIN_LEVEL) >
                      static_cast<int>(LogLevel::TRACE)) {
            if (level < COMPILE_TIME_MIN_LEVEL)
                return;
        }

        // Runtime check
        if (level < s_runtimeLevel)
            return;

        // Get current time for timestamp
        std::time_t now = std::time(nullptr);
        std::tm localTimeBuf;
#if defined(_WIN32)
        localtime_s(&localTimeBuf, &now);
#else
        localtime_r(&now, &localTimeBuf);
#endif
        std::tm* localTime = &localTimeBuf;

        // Format: [HH:MM:SS] [LEVEL] [context] message
        std::fprintf(stderr, "[%02d:%02d:%02d] [%s] [%s] ", localTime->tm_hour, localTime->tm_min,
                     localTime->tm_sec, levelToString(level), context);

        // Print the actual message
        if constexpr (sizeof...(args) > 0) {
            std::fprintf(stderr, fmt, args...);
        } else {
            std::fprintf(stderr, "%s", fmt);
        }
        std::fprintf(stderr, "\n");

        // Flush immediately for FATAL so we see it before crash
        if (level == LogLevel::FATAL) {
            std::fflush(stderr);
        }
    }
};

}  // namespace smb

// =============================================================================
// Convenience macros for lazy people (like me)
// =============================================================================
// These auto-fill the context with file info in debug builds.
// Use the class methods directly if you want custom context strings.

#define SMB_LOG_TRACE(fmt, ...) smb::Logger::trace(__func__, fmt, ##__VA_ARGS__)
#define SMB_LOG_DEBUG(fmt, ...) smb::Logger::debug(__func__, fmt, ##__VA_ARGS__)
#define SMB_LOG_INFO(fmt, ...) smb::Logger::info(__func__, fmt, ##__VA_ARGS__)
#define SMB_LOG_WARN(fmt, ...) smb::Logger::warn(__func__, fmt, ##__VA_ARGS__)
#define SMB_LOG_ERROR(fmt, ...) smb::Logger::error(__func__, fmt, ##__VA_ARGS__)
#define SMB_LOG_FATAL(fmt, ...) smb::Logger::fatal(__func__, fmt, ##__VA_ARGS__)
