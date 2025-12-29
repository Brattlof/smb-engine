#pragma once

// =============================================================================
// InputReader Interface
// =============================================================================
// Abstract interface for reading input. Game logic uses this instead of
// calling raylib directly. This lets us:
//   - Run physics without a window (for testing/replay)
//   - Mock inputs for deterministic replay
//   - Swap input backends if needed
//
// Input is sampled once per frame and stored in InputState. Physics code
// reads from InputState, never from this interface directly.
// =============================================================================

namespace smb {

// Normalized input state - what the game logic sees.
// All values are normalized and platform-independent.
struct InputState {
    // Tilt input from analog stick or keyboard, range [-1, 1] on each axis.
    // Positive X = tilt right, Positive Y = tilt forward (away from camera)
    float tiltX;
    float tiltY;

    // Digital button states (pressed this frame)
    bool pausePressed;
    bool confirmPressed;
    bool cancelPressed;

    // Digital button states (held)
    bool pauseHeld;
    bool confirmHeld;
    bool cancelHeld;

    // Debug inputs (only available in debug builds)
    bool debugTogglePressed;
    bool debugResetPressed;

    // Returns true if any tilt input is non-zero
    bool hasTiltInput() const {
        constexpr float DEADZONE = 0.01f;
        return (tiltX > DEADZONE || tiltX < -DEADZONE || tiltY > DEADZONE || tiltY < -DEADZONE);
    }

    // Clears all input state to default (no input)
    void clear() {
        tiltX = 0.0f;
        tiltY = 0.0f;
        pausePressed = false;
        confirmPressed = false;
        cancelPressed = false;
        pauseHeld = false;
        confirmHeld = false;
        cancelHeld = false;
        debugTogglePressed = false;
        debugResetPressed = false;
    }
};

// Abstract interface - implement this to provide input
class InputReader {
public:
    virtual ~InputReader() = default;

    // Called once per frame before physics update.
    // Implementation should poll input devices and fill out InputState.
    virtual void poll() = 0;

    // Returns the current input state (from last poll).
    virtual const InputState& getState() const = 0;

    // Returns true if the window close was requested.
    // (X button, Alt+F4, etc.)
    virtual bool isCloseRequested() const = 0;
};

}  // namespace smb
