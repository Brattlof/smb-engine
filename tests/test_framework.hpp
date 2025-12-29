#pragma once

#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

// =============================================================================
// Minimal Test Framework
// =============================================================================
// Simple test framework for SMB physics engine. No external dependencies.
// Supports basic assertions, test registration, and result reporting.
//
// Usage:
//   TEST_CASE("Physics", "Gravity applies correctly") {
//       Vec3 vel = {0, 0, 0};
//       applyGravity(vel, 1.0/60.0);
//       ASSERT_NEAR(vel.y, -0.163, 0.001);
//   }
//
// Run with: xmake run tests
// =============================================================================

namespace test {

// Stores info about a test failure
struct TestFailure {
    std::string file;
    int line;
    std::string expression;
    std::string message;
};

// Represents a single test case
struct TestCase {
    std::string category;
    std::string name;
    std::function<void()> func;
};

// Test registry - collects all tests
class TestRegistry {
public:
    static TestRegistry& instance() {
        static TestRegistry reg;
        return reg;
    }

    void addTest(const char* category, const char* name, std::function<void()> func) {
        m_tests.push_back({category, name, func});
    }

    // Run all tests and return number of failures
    int runAll() {
        int passed = 0;
        int failed = 0;

        std::printf("\n========================================\n");
        std::printf("  SMB Physics Engine - Test Suite\n");
        std::printf("========================================\n\n");

        for (const auto& test : m_tests) {
            std::printf("[RUN ] %s :: %s\n", test.category.c_str(), test.name.c_str());

            m_currentFailures.clear();

            try {
                test.func();
            } catch (const std::exception& e) {
                recordFailure(__FILE__, __LINE__, "exception", e.what());
            } catch (...) {
                recordFailure(__FILE__, __LINE__, "exception", "Unknown exception");
            }

            if (m_currentFailures.empty()) {
                std::printf("[PASS] %s :: %s\n", test.category.c_str(), test.name.c_str());
                passed++;
            } else {
                std::printf("[FAIL] %s :: %s\n", test.category.c_str(), test.name.c_str());
                for (const auto& f : m_currentFailures) {
                    std::printf("       %s:%d: %s\n", f.file.c_str(), f.line, f.message.c_str());
                    std::printf("       Expression: %s\n", f.expression.c_str());
                }
                failed++;
            }
        }

        std::printf("\n========================================\n");
        std::printf("  Results: %d passed, %d failed\n", passed, failed);
        std::printf("========================================\n\n");

        return failed;
    }

    void recordFailure(const char* file, int line, const char* expr, const char* msg) {
        // Extract just filename from path
        const char* lastSlash = std::strrchr(file, '/');
        const char* lastBackslash = std::strrchr(file, '\\');
        const char* filename = file;
        if (lastSlash && lastSlash > filename)
            filename = lastSlash + 1;
        if (lastBackslash && lastBackslash > filename)
            filename = lastBackslash + 1;

        m_currentFailures.push_back({filename, line, expr, msg});
    }

private:
    std::vector<TestCase> m_tests;
    std::vector<TestFailure> m_currentFailures;
};

// Auto-registration helper
struct TestRegistrar {
    TestRegistrar(const char* category, const char* name, std::function<void()> func) {
        TestRegistry::instance().addTest(category, name, func);
    }
};

}  // namespace test

// =============================================================================
// Macros
// =============================================================================

// Helper macros for generating unique names with __LINE__
#define TEST_CONCAT_IMPL(a, b) a##b
#define TEST_CONCAT(a, b) TEST_CONCAT_IMPL(a, b)

// Define a test case
#define TEST_CASE(category, name)                                                                  \
    static void TEST_CONCAT(test_func_, __LINE__)();                                               \
    static test::TestRegistrar TEST_CONCAT(registrar_, __LINE__)(                                  \
        category, name, TEST_CONCAT(test_func_, __LINE__));                                        \
    static void TEST_CONCAT(test_func_, __LINE__)()

// Basic assertions
#define ASSERT_TRUE(expr)                                                                          \
    do {                                                                                           \
        if (!(expr)) {                                                                             \
            test::TestRegistry::instance().recordFailure(__FILE__, __LINE__, #expr,                \
                                                         "Expected true");                         \
        }                                                                                          \
    } while (0)

#define ASSERT_FALSE(expr)                                                                         \
    do {                                                                                           \
        if (expr) {                                                                                \
            test::TestRegistry::instance().recordFailure(__FILE__, __LINE__, #expr,                \
                                                         "Expected false");                        \
        }                                                                                          \
    } while (0)

#define ASSERT_EQ(a, b)                                                                            \
    do {                                                                                           \
        if ((a) != (b)) {                                                                          \
            char msg[256];                                                                         \
            std::snprintf(msg, sizeof(msg), "Expected equality");                                  \
            test::TestRegistry::instance().recordFailure(__FILE__, __LINE__, #a " == " #b, msg);   \
        }                                                                                          \
    } while (0)

#define ASSERT_NE(a, b)                                                                            \
    do {                                                                                           \
        if ((a) == (b)) {                                                                          \
            test::TestRegistry::instance().recordFailure(__FILE__, __LINE__, #a " != " #b,         \
                                                         "Expected inequality");                   \
        }                                                                                          \
    } while (0)

// Float comparison with tolerance
#define ASSERT_NEAR(a, b, epsilon)                                                                 \
    do {                                                                                           \
        double diff = std::fabs((double)(a) - (double)(b));                                        \
        if (diff > (epsilon)) {                                                                    \
            char msg[256];                                                                         \
            std::snprintf(msg, sizeof(msg), "Expected |%.6f - %.6f| <= %.6f, got %.6f",            \
                          (double)(a), (double)(b), (double)(epsilon), diff);                      \
            test::TestRegistry::instance().recordFailure(__FILE__, __LINE__, #a " ~= " #b, msg);   \
        }                                                                                          \
    } while (0)

// Check that something is within a range
#define ASSERT_RANGE(val, min, max)                                                                \
    do {                                                                                           \
        if ((val) < (min) || (val) > (max)) {                                                      \
            char msg[256];                                                                         \
            std::snprintf(msg, sizeof(msg), "Expected %.6f in range [%.6f, %.6f]", (double)(val),  \
                          (double)(min), (double)(max));                                           \
            test::TestRegistry::instance().recordFailure(__FILE__, __LINE__, #val " in range",     \
                                                         msg);                                     \
        }                                                                                          \
    } while (0)

// Fail unconditionally
#define FAIL(message)                                                                              \
    test::TestRegistry::instance().recordFailure(__FILE__, __LINE__, "FAIL", message)
