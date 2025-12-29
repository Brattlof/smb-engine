#pragma once

// =============================================================================
// AudioPlayer Interface
// =============================================================================
// Abstract interface for audio playback. Keeps raylib audio calls isolated
// from game logic.
//
// Currently using a minimal interface with NullAudioPlayer for silent operation.
// The real raylib-based implementation will be added when sound effects are needed.
// =============================================================================

namespace smb {

// Sound effect identifiers - we'll add more as needed
enum class SoundEffect {
    NONE = 0,
    ROLL,
    BOUNCE,
    GOAL,
    FALL_OUT,
    TIMER_WARNING,
    MENU_SELECT,
    MENU_CONFIRM
};

class AudioPlayer {
public:
    virtual ~AudioPlayer() = default;

    // Initialize audio system
    virtual bool initialize() = 0;

    // Shutdown audio system
    virtual void shutdown() = 0;

    // Play a sound effect
    virtual void playSound(SoundEffect effect) = 0;

    // Set master volume (0.0 to 1.0)
    virtual void setVolume(float volume) = 0;

    // Mute/unmute all audio
    virtual void setMuted(bool muted) = 0;

    // Returns true if audio is muted
    virtual bool isMuted() const = 0;
};

// Null implementation for when we don't want audio (testing, etc.)
class NullAudioPlayer : public AudioPlayer {
public:
    bool initialize() override { return true; }
    void shutdown() override {}
    void playSound(SoundEffect) override {}
    void setVolume(float) override {}
    void setMuted(bool muted) override { m_muted = muted; }
    bool isMuted() const override { return m_muted; }

private:
    bool m_muted = false;
};

}  // namespace smb
