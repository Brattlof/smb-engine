#include "test_framework.hpp"

// Entry point for test runner
int main() {
    return test::TestRegistry::instance().runAll();
}
