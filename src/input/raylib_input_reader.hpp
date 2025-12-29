#pragma once

#include "input/input_reader.hpp"

// =============================================================================
// Raylib Input Reader
// =============================================================================
// Concrete implementation of InputReader using raylib.
// Handles keyboard, gamepad, and window close detection.
// =============================================================================

namespace smb {

class RaylibInputReader : public InputReader {
public:
    RaylibInputReader();
    ~RaylibInputReader() override = default;

    void poll() override;
    const InputState& getState() const override;
    bool isCloseRequested() const override;

private:
    InputState m_currentState;
    InputState m_previousState;
    bool m_closeRequested;

    // Keyboard input helpers
    void pollKeyboard();

    // Gamepad input helpers
    void pollGamepad();

    // Applies deadzone and normalizes analog input
    float applyDeadzone(float value, float deadzone) const;
};

}  // namespace smb
