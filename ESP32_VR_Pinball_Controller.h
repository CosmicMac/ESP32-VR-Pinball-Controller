#pragma once
#include "config.h"

// Enums
//@formatter: off
enum class ControllerMode : uint8_t { FX, CLASSIC, VPX, count }; // Meta Quest Pinball FX VR (keyboard), Meta Quest Pinball VR Classic (gamepad), Virtual Pinball X (keyboard)

enum class LedColor : uint8_t { OFF, RED, GREEN, BLUE, YELLOW, PURPLE, CYAN, WHITE, FX_MODE, CLASSIC_MODE, VPX_MODE };

enum class ButtonType : uint8_t { BUTTON, DPAD };

enum class ActionType : uint8_t { NONE, KEYBOARD_KEY, GAMEPAD_BUTTON, GAMEPAD_DPAD, GAMEPAD_LT, GAMEPAD_RT };

//@formatter: on


// Constants
constexpr LedColor MODE_COLORS[] = {LedColor::FX_MODE, LedColor::CLASSIC_MODE, LedColor::VPX_MODE};


// Structs
struct ButtonAction
{
    ActionType type;

    union
    {
        uint8_t keyCode;     // Keyboard
        uint16_t buttonCode; // Gamepad button
        uint8_t dpadValue;   // DPAD
    };
};

struct ButtonInfo
{
    uint8_t pin;                    // GPIO pin number for the button
    ButtonType type;                // Type of input (BUTTON or DPAD)
    ClassicBtn classicCode;         // Input mapping for Classic (gamepad button or DPAD direction)
    FxKey fxKey;                    // Key mapping for FX
    VpxKey vpxKey;                  // Key mapping for VPX
    int state;                      // Current debounced state (HIGH or LOW)
    unsigned long lastDebounceTime; // Timestamp of the last debounce event (in ms)
};

struct NudgeState
{
    unsigned long lastNudgeMillis = 0;
    bool isNudging                = false;
    uint8_t nudgeKey              = 0;
};

struct NudgeProcess
{
    // Filter parameters
    static constexpr float ACCEL_G_RANGE        = 2.0f;
    static constexpr int JITTER_WINDOW          = 100;
    static constexpr float DC_ADAPT_TIME_s      = 0.3f; // DC removal adaptation time — 200-500 ms recommended
    static constexpr float FRICTION_HALF_LIFE_s = 2.0f; // Velocity half-life in seconds — 2 s recommended

    // Optimization: Pre-calculate inverses to use multiplication instead of division (faster on microcontrollers without FPU)
    static constexpr float INV_DC_ADAPT_TIME      = 1.0f / DC_ADAPT_TIME_s;
    static constexpr float INV_FRICTION_HALF_LIFE = 1.0f / FRICTION_HALF_LIFE_s;

    // Internal state
    bool initialized    = false;
    int windowMin       = 0;    // Jitter filter window minimum
    int windowMax       = 0;    // Jitter filter window maximum
    float dcValue       = 0.0f; // DC Blocker (moving average)
    float acceleration  = 0.0f; // Filtered acceleration in raw sensor units
    float velocity      = 0.0f; // Integrated velocity in mm/s
    uint32_t lastMicros = 0;    // Timestamp of the last processed sample in microseconds

    // Conversion factor for converting raw units to mm/s²
    static constexpr float ACCEL_CONV_FACTOR = (ACCEL_G_RANGE / 32768.0f) * 9806.65f;

    void reset() {
        initialized  = false;
        windowMin    = 0;
        windowMax    = 0;
        dcValue      = 0.0f;
        acceleration = 0.0f;
        velocity     = 0.0f;
        lastMicros   = 0;
    }

    void process(int raw, uint32_t nowMicros) {
        if (!initialized) {
            initialized  = true;
            windowMin    = raw;
            windowMax    = raw;
            dcValue      = static_cast<float>(raw);
            acceleration = 0.0f;
            velocity     = 0.0f;
            lastMicros   = nowMicros;
            return;
        }

        // Calculation of Delta Time (dt) in seconds
        float dt = static_cast<float>(nowMicros - lastMicros) * 1e-6f;

        // Safety: Cap dt to prevent physics explosions if the loop lags significantly (e.g. > 100ms)
        if (dt > 0.1f) {
            dt = 0.1f;
        }

        lastMicros = nowMicros;

        // Jitter Filter (Hysteresis)
        // Stabilizes the signal before DC processing
        if (raw < windowMin) {
            windowMin = raw;
            windowMax = raw + JITTER_WINDOW;
        }
        else if (raw > windowMax) {
            windowMax = raw;
            windowMin = raw - JITTER_WINDOW;
        }

        const int stable = (windowMin + windowMax) / 2;

        // Dynamic DC Blocker (Gravity/Tilt Cancellation)
        // Use dt for consistent scaling regardless of the sample rate
        // Optimized: Used multiplication by inverse
        const float dcAlpha = std::clamp(dt * INV_DC_ADAPT_TIME, 0.0f, 1.0f);
        dcValue             += dcAlpha * (static_cast<float>(stable) - dcValue);
        acceleration        = static_cast<float>(stable) - dcValue;

        // Conversion to mm/s²
        const float accel_mm_s2 = acceleration * ACCEL_CONV_FACTOR;

        // Friction Filter (Velocity Attenuation)
        // Calculates the damping factor based on the actual elapsed time
        // Optimized: Used multiplication by inverse
        float friction = powf(0.5f, dt * INV_FRICTION_HALF_LIFE);
        velocity       *= friction;

        // Integration: v = v + (a * dt)
        // The acceleration is added to the velocity for this time step
        velocity += accel_mm_s2 * dt;
    }
};

// Function declarations
void handleButton(ButtonInfo& button);
void setMode(ControllerMode newMode, bool initialConfig = false);
void setLedColor(LedColor color);
void setupAccelerometer();
void readAccelRaw(int16_t& x, int16_t& y);
void handleAnalogNudge();
void handleDigitalNudge();
void performButtonAction(const ButtonAction& action, bool isPressed);
ButtonAction getButtonAction(const ButtonInfo& button);
