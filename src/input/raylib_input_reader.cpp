#include "input/raylib_input_reader.hpp"

#include "util/logger.hpp"

#include <raylib.h>

#include <cmath>

namespace smb {

// Analog stick deadzone. GameCube controllers had pretty loose sticks,
// so we use a moderate deadzone to avoid drift.
constexpr float GAMEPAD_DEADZONE = 0.15f;

// Keyboard input simulates analog at full magnitude
constexpr float KEYBOARD_MAGNITUDE = 1.0f;

RaylibInputReader::RaylibInputReader()
    : m_currentState{}, m_previousState{}, m_closeRequested(false) {
    m_currentState.clear();
    m_previousState.clear();
}

void RaylibInputReader::poll() {
    // Store previous state for edge detection (pressed vs held)
    m_previousState = m_currentState;
    m_currentState.clear();

    // Check window close
    m_closeRequested = WindowShouldClose();

    // Poll all input sources, gamepad takes priority for analog
    pollKeyboard();
    pollGamepad();

    // Clamp tilt to unit circle (don't allow > 1.0 magnitude)
    float mag = std::sqrt(m_currentState.tiltX * m_currentState.tiltX +
                          m_currentState.tiltY * m_currentState.tiltY);
    if (mag > 1.0f) {
        m_currentState.tiltX /= mag;
        m_currentState.tiltY /= mag;
    }

    // Edge detection for "pressed" states
    m_currentState.pausePressed = m_currentState.pauseHeld && !m_previousState.pauseHeld;
    m_currentState.confirmPressed = m_currentState.confirmHeld && !m_previousState.confirmHeld;
    m_currentState.cancelPressed = m_currentState.cancelHeld && !m_previousState.cancelHeld;
}

const InputState& RaylibInputReader::getState() const {
    return m_currentState;
}

bool RaylibInputReader::isCloseRequested() const {
    return m_closeRequested;
}

void RaylibInputReader::pollKeyboard() {
    // WASD or Arrow keys for tilt
    // These simulate analog at full magnitude

    if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) {
        m_currentState.tiltX -= KEYBOARD_MAGNITUDE;
    }
    if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) {
        m_currentState.tiltX += KEYBOARD_MAGNITUDE;
    }
    if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP)) {
        m_currentState.tiltY += KEYBOARD_MAGNITUDE;
    }
    if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN)) {
        m_currentState.tiltY -= KEYBOARD_MAGNITUDE;
    }

    // Button mappings
    // ESC or P = Pause
    m_currentState.pauseHeld = IsKeyDown(KEY_ESCAPE) || IsKeyDown(KEY_P);

    // Enter or Space = Confirm
    m_currentState.confirmHeld = IsKeyDown(KEY_ENTER) || IsKeyDown(KEY_SPACE);

    // Backspace or X = Cancel
    m_currentState.cancelHeld = IsKeyDown(KEY_BACKSPACE) || IsKeyDown(KEY_X);

// Debug keys (only in debug builds, but we check anyway)
#ifndef NDEBUG
    m_currentState.debugTogglePressed = IsKeyPressed(KEY_F1);
    m_currentState.debugResetPressed = IsKeyPressed(KEY_F5);
#endif
}

void RaylibInputReader::pollGamepad() {
    // Check first gamepad only (player 1)
    if (!IsGamepadAvailable(0)) {
        return;
    }

    // Left stick for tilt - this is analog so it overrides keyboard
    float stickX = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_X);
    float stickY =
        -GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_Y);  // Invert Y for our coord system

    stickX = applyDeadzone(stickX, GAMEPAD_DEADZONE);
    stickY = applyDeadzone(stickY, GAMEPAD_DEADZONE);

    // If gamepad has input, use it instead of keyboard
    if (std::fabs(stickX) > 0.0f || std::fabs(stickY) > 0.0f) {
        m_currentState.tiltX = stickX;
        m_currentState.tiltY = stickY;
    }

    // Button mappings for gamepad
    // Start = Pause
    if (IsGamepadButtonDown(0, GAMEPAD_BUTTON_MIDDLE_RIGHT)) {
        m_currentState.pauseHeld = true;
    }

    // A button = Confirm
    if (IsGamepadButtonDown(0, GAMEPAD_BUTTON_RIGHT_FACE_DOWN)) {
        m_currentState.confirmHeld = true;
    }

    // B button = Cancel
    if (IsGamepadButtonDown(0, GAMEPAD_BUTTON_RIGHT_FACE_RIGHT)) {
        m_currentState.cancelHeld = true;
    }
}

float RaylibInputReader::applyDeadzone(float value, float deadzone) const {
    if (std::fabs(value) < deadzone) {
        return 0.0f;
    }

    // Rescale so that edge of deadzone is 0 and full range is 1
    float sign = value > 0.0f ? 1.0f : -1.0f;
    float magnitude = std::fabs(value);
    float rescaled = (magnitude - deadzone) / (1.0f - deadzone);

    return sign * rescaled;
}

}  // namespace smb
