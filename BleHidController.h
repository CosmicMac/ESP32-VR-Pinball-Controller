#pragma once

#include "NimBLEDevice.h"
#include "NimBLEHIDDevice.h"
#include "HidConstants.h"

constexpr int8_t BLE_TX_POWER        = 9;  // dBm
constexpr uint8_t BATTERY_LEVEL      = 66; // %
constexpr uint8_t PNP_VENDOR_SRC_USB = 0x02;


class BleHidController
{
public:
    BleHidController();

    void begin(
        const char* deviceName         = "BLE HID Controller",
        const char* deviceManufacturer = "CosmicMac",
        uint16_t vendorId              = 0x1234,
        uint16_t productId             = 0x5678,
        uint16_t version               = 0x0100
    );

    static void deleteAllBonds();

    static bool isConnected() { return _deviceConnected; }

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
    void setLeftStick(int16_t lx, int16_t ly, bool sendState = true);
    void setRightStick(int16_t rx, int16_t ry, bool sendState = true);
    void setLeftTrigger(uint16_t lt, bool sendState = true);
    void setRightTrigger(uint16_t rt, bool sendState = true);
    void sendGamepadState();

private:
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
        uint16_t buttons = 0;                                             // 16 buttons bitfield
        uint8_t dpad     = static_cast<uint8_t>(DpadDirection::CENTERED); // D-Pad 4 bits (up, right, down, left) + padding 4 bits
        int16_t leftX    = 0;                                             // Left stick X (X axis, -32768–32767)
        int16_t leftY    = 0;                                             // Left stick Y (Y axis, -32768–32767)
        int16_t rightX   = 0;                                             // Right stick X (Rx axis, -32768–32767)
        int16_t rightY   = 0;                                             // Right stick Y (Ry axis, -32768–32767)
        uint16_t lt      = 0;                                             // Left trigger  (Z  axis, 0–32767)
        uint16_t rt      = 0;                                             // Right trigger (Rz axis, 0–32767)
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
