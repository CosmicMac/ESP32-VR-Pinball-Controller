#pragma once
#include "config.h"

// Enums
//@formatter: off
enum class ControllerMode : uint8_t { FX, CLASSIC, VPX, count }; // Meta Quest Pinball FX VR (keyboard), Meta Quest Pinball VR Classic (gamepad), Virtual Pinball X (keyboard)

enum class LedColor : uint8_t { OFF, RED, GREEN, BLUE, YELLOW, PURPLE, CYAN, WHITE, FX_MODE, CLASSIC_MODE, VPX_MODE };

enum class ButtonType : uint8_t { BUTTON, DPAD };

enum class ActionType : uint8_t { NONE, KEYBOARD_KEY, GAMEPAD_BUTTON, GAMEPAD_DPAD };

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

// Function declarations
void handleButton(ButtonInfo& button);
void setMode(ControllerMode newMode, bool initialConfig = false);
void setLedColor(LedColor color);
void setupAccelerometer();
bool readAccelRaw(int16_t& x, int16_t& y);
void handleAnalogNudge();
void handleDigitalNudge();
void performButtonAction(const ButtonAction& action, bool isPressed);
ButtonAction getButtonAction(const ButtonInfo& button, ControllerMode mode);
