#pragma once
#include "config.h"
#include "ESP32_VR_Pinball_Controller.h"

// Forward declaration
class BleHidController;

// Enums
enum class ButtonType : uint8_t { BUTTON, DPAD };

enum class ActionType : uint8_t { NONE, KEYBOARD_KEY, GAMEPAD_BUTTON, GAMEPAD_DPAD, GAMEPAD_LT, GAMEPAD_RT };

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
    uint8_t pin;               // GPIO pin number for the button
    ButtonType type;           // Type of input (BUTTON or DPAD)
    ClassicBtn classicCode;    // Input mapping for Classic (gamepad button or DPAD direction)
    FxKey fxKey;               // Key mapping for FX
    VpxKey vpxKey;             // Key mapping for VPX
    int state;                 // Current debounced state (HIGH or LOW)
    uint32_t lastDebounceTime; // Timestamp of the last debounce event (in ms)
};

class ButtonManager
{
public:
    ButtonManager();
    ~ButtonManager() = default;

    void begin(BleHidController& hidController);
    void handle(ControllerMode mode);

private:
    void handleButton(ButtonInfo& button, ControllerMode mode);
    void performButtonAction(const ButtonAction& action, bool isPressed);
    ButtonAction getButtonAction(const ButtonInfo& button, ControllerMode mode);

    static constexpr uint8_t NUM_BUTTONS = 15;
    ButtonInfo buttons[NUM_BUTTONS];
    BleHidController* m_hidController;
};
