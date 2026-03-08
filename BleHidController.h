#pragma once

#include "NimBLEDevice.h"
#include "NimBLEHIDDevice.h"

constexpr int8_t BLE_TX_POWER        = 9;  // dBm
constexpr uint8_t BATTERY_LEVEL      = 66; // %
constexpr uint8_t PNP_VENDOR_SRC_USB = 0x02;

// Report IDs
#define REPORT_ID_KEYBOARD  1
#define REPORT_ID_GAMEPAD   2

// Key codes
#define KEY_NONE            0x00
#define KEY_ERROR_ROLLOVER  0x01
#define KEY_POST_FAIL       0x02
#define KEY_ERROR_UNDEFINED 0x03
#define KEY_A               0x04
#define KEY_B               0x05
#define KEY_C               0x06
#define KEY_D               0x07
#define KEY_E               0x08
#define KEY_F               0x09
#define KEY_G               0x0A
#define KEY_H               0x0B
#define KEY_I               0x0C
#define KEY_J               0x0D
#define KEY_K               0x0E
#define KEY_L               0x0F
#define KEY_M               0x10
#define KEY_N               0x11
#define KEY_O               0x12
#define KEY_P               0x13
#define KEY_Q               0x14
#define KEY_R               0x15
#define KEY_S               0x16
#define KEY_T               0x17
#define KEY_U               0x18
#define KEY_V               0x19
#define KEY_W               0x1A
#define KEY_X               0x1B
#define KEY_Y               0x1C
#define KEY_Z               0x1D
#define KEY_1               0x1E
#define KEY_2               0x1F
#define KEY_3               0x20
#define KEY_4               0x21
#define KEY_5               0x22
#define KEY_6               0x23
#define KEY_7               0x24
#define KEY_8               0x25
#define KEY_9               0x26
#define KEY_0               0x27
#define KEY_ENTER           0x28
#define KEY_ESC             0x29
#define KEY_BACKSPACE       0x2A
#define KEY_TAB             0x2B
#define KEY_SPACE           0x2C
#define KEY_MINUS           0x2D
#define KEY_EQUAL           0x2E
#define KEY_LEFTBRACE       0x2F
#define KEY_RIGHTBRACE      0x30
#define KEY_BACKSLASH       0x31
#define KEY_HASHTILDE       0x32
#define KEY_SEMICOLON       0x33
#define KEY_APOSTROPHE      0x34
#define KEY_GRAVE           0x35
#define KEY_COMMA           0x36
#define KEY_DOT             0x37
#define KEY_SLASH           0x38
#define KEY_CAPSLOCK        0x39
#define KEY_F1              0x3A
#define KEY_F2              0x3B
#define KEY_F3              0x3C
#define KEY_F4              0x3D
#define KEY_F5              0x3E
#define KEY_F6              0x3F
#define KEY_F7              0x40
#define KEY_F8              0x41
#define KEY_F9              0x42
#define KEY_F10             0x43
#define KEY_F11             0x44
#define KEY_F12             0x45
#define KEY_SYSRQ           0x46
#define KEY_SCROLLLOCK      0x47
#define KEY_PAUSE           0x48
#define KEY_INSERT          0x49
#define KEY_HOME            0x4A
#define KEY_PAGEUP          0x4B
#define KEY_DELETE          0x4C
#define KEY_END             0x4D
#define KEY_PAGEDOWN        0x4E
#define KEY_RIGHT           0x4F
#define KEY_LEFT            0x50
#define KEY_DOWN            0x51
#define KEY_UP              0x52
#define KEY_NUMLOCK         0x53
#define KEY_KPSLASH         0x54
#define KEY_KPASTERISK      0x55
#define KEY_KPMINUS         0x56
#define KEY_KPPLUS          0x57
#define KEY_KPENTER         0x58
#define KEY_KP1             0x59
#define KEY_KP2             0x5A
#define KEY_KP3             0x5B
#define KEY_KP4             0x5C
#define KEY_KP5             0x5D
#define KEY_KP6             0x5E
#define KEY_KP7             0x5F
#define KEY_KP8             0x60
#define KEY_KP9             0x61
#define KEY_KP0             0x62
#define KEY_KPDOT           0x63
#define KEY_102ND           0x64
#define KEY_COMPOSE         0x65
#define KEY_POWER           0x66
#define KEY_KPEQUAL         0x67
#define KEY_SECTIONSIGN     0x68

// Modifier key codes (support not guaranteed)
#define KEY_LEFT_CTRL       0xE0
#define KEY_LEFT_SHIFT      0xE1
#define KEY_LEFT_ALT        0xE2
#define KEY_LEFT_META       0xE3 // Windows/Command/Meta
#define KEY_RIGHT_CTRL      0xE4
#define KEY_RIGHT_SHIFT     0xE5
#define KEY_RIGHT_ALT       0xE6
#define KEY_RIGHT_META      0xE7

// Modifier codes
#define MOD_NONE            0x00
#define MOD_LEFT_CTRL       0x01
#define MOD_LEFT_SHIFT      0x02
#define MOD_LEFT_ALT        0x04
#define MOD_LEFT_META       0x08 // Windows/Command/Meta
#define MOD_RIGHT_CTRL      0x10
#define MOD_RIGHT_SHIFT     0x20
#define MOD_RIGHT_ALT       0x40
#define MOD_RIGHT_META      0x80

// Gamepad buttons
#define BTN_NONE        0x00
#define BTN_A           0x01
#define BTN_B           0x02
#define BTN_X           0x08
#define BTN_Y           0x10
#define BTN_LB          0x40
#define BTN_RB          0x80
#define BTN_SELECT      0x400
#define BTN_START       0x800
#define BTN_HOME        0x1000
#define BTN_LS          0x2000 // Left stick
#define BTN_RS          0x4000 // Right stick

// D-Pad
#define DPAD_CENTERED   0x0F
#define DPAD_UP         0x00
#define DPAD_UP_RIGHT   0x01
#define DPAD_RIGHT      0x02
#define DPAD_DOWN_RIGHT 0x03
#define DPAD_DOWN       0x04
#define DPAD_DOWN_LEFT  0x05
#define DPAD_LEFT       0x06
#define DPAD_UP_LEFT    0x07


class BleHidController
{
public:
    BleHidController();

    void begin(
        const char* deviceName         = "BLE HID Controller",
        const char* deviceManufacturer = "CosmicMac",
        uint16_t vendorId              = 0x045E, // Microsoft VID, for compatibility
        uint16_t productId             = 0x0B13, // XBox Bluetooth gamepad PID, for compatibility
        uint16_t version               = 0x0100
    );

    bool isConnected() const { return _deviceConnected; }

    // Keyboard API
    void keyModPress(uint8_t modifier);
    void keyModRelease(uint8_t modifier);
    void keyPress(uint8_t keycode);
    void keyRelease(uint8_t keycode);
    void keyReleaseAll();

    // Gamepad API
    void sendGamepad(uint16_t buttons, uint8_t dpad, int16_t lx, int16_t ly, int16_t rx, int16_t ry, uint16_t lt = 0, uint16_t rt = 0);
    void buttonPress(uint16_t button);
    void buttonRelease(uint16_t button);
    void dpadPress(uint8_t dpad);
    void dpadRelease();
    void setLeftStick(int16_t lx, int16_t ly);
    void setRightStick(int16_t rx, int16_t ry);

private:
    void sendGamepadState();
    void sendKeyboardState();

    struct KeyReport
    {
        uint8_t modifiers = 0;  // Modifier keys bitfield
        uint8_t keys[6]   = {}; // Up to 6 simultaneous key presses
    };


    /*
     * The __attribute__((packed)) directive is a GCC extension that tells the compiler not to add any padding
     * between the members of the struct. This ensures the struct layout in memory matches exactly the order
     * and size of its fields, which is important for binary communication protocols like HID reports.
     * Without this directive, the compiler would insert a padding byte after dpad for alignment purposes,
     * which would break the expected report format.
     */
    struct __attribute__((packed)) GamepadReport
    {
        uint16_t buttons = 0;             // 16 buttons bitfield
        uint8_t dpad     = DPAD_CENTERED; // D-Pad 4 bits (up, right, down, left) + padding 4 bits
        int16_t leftX    = 0;             // Left stick X
        int16_t leftY    = 0;             // Left stick Y
        int16_t rightX   = 0;             // Right stick X
        int16_t rightY   = 0;             // Right stick Y
        uint16_t lt      = 0;             // L eft trigger (0-255), padding (8 bits)
        uint16_t rt      = 0;             // Right trigger (0-255), padding (8 bits)
    };

    static bool _deviceConnected;

    NimBLEServer* _server{};
    NimBLEHIDDevice* _hidDevice{};
    NimBLECharacteristic* _kbInputReport{};
    NimBLECharacteristic* _gpInputReport{};

    KeyReport _kbState{};
    GamepadReport _gpState{};

    class ServerCallbacks;
    friend class ServerCallbacks;
};
