#pragma once

#include "NimBLEDevice.h"
#include "NimBLEHIDDevice.h"
#include "NimBLEServer.h"
#include "HidConstants.h"
#include "config.h"

class BleHidController : public NimBLEServerCallbacks
{
public:
    BleHidController();
    ~BleHidController();

    // Rule of Five: prevent copying
    BleHidController(const BleHidController&) = delete;
    BleHidController& operator=(const BleHidController&) = delete;

    bool begin(
        const char* deviceName         = DEVICE_NAME,
        const char* deviceManufacturer = DEVICE_MANUFACTURER,
        uint16_t vendorId              = 0x1234,
        uint16_t productId             = 0x5678,
        uint16_t version               = 0x0100
    );

    static void deleteAllBonds();

    bool isConnected() const { return m_deviceConnected; }

    // Keyboard API
    bool keyModPress(uint8_t modifier);
    bool keyModRelease(uint8_t modifier);
    bool keyPress(uint8_t keycode);
    bool keyRelease(uint8_t keycode);
    bool keyReleaseAll();

    // Gamepad API
    bool sendGamepad(uint16_t buttons, uint8_t dpad, int16_t lx, int16_t ly, int16_t rx, int16_t ry, int16_t lt = 0, int16_t rt = 0);
    bool buttonPress(uint16_t button);
    bool buttonRelease(uint16_t button);
    bool dpadPress(uint8_t dpad);
    bool dpadRelease();
    bool setLeftStick(int16_t lx, int16_t ly, bool sendState = true);
    bool setRightStick(int16_t rx, int16_t ry, bool sendState = true);
    bool setLeftTrigger(int16_t lt, bool sendState = true);
    bool setRightTrigger(int16_t rt, bool sendState = true);
    bool sendGamepadState();

    // NimBLEServerCallbacks
    void onConnect(NimBLEServer* pSrv, NimBLEConnInfo& connInfo) override;
    void onDisconnect(NimBLEServer* pSrv, NimBLEConnInfo& connInfo, int reason) override;

private:
    bool sendKeyboardState();

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
        int16_t lt       = 0;                                             // Left trigger  (Z  axis, 0–32767)
        int16_t rt       = 0;                                             // Right trigger (Rz axis, 0–32767)
    };

    bool m_deviceConnected{false};

    NimBLEServer* m_server{};
    NimBLEHIDDevice* m_hidDevice{};
    NimBLECharacteristic* m_kbInputReport{};
    NimBLECharacteristic* m_gpInputReport{};

    KeyReport m_kbState{};
    GamepadReport m_gpState{};
};
