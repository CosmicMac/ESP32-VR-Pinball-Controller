#include "BleHidController.h"

namespace {
    constexpr int8_t BLE_TX_POWER        = 9;  // dBm
    constexpr uint8_t BATTERY_LEVEL      = 66; // %
    constexpr uint8_t PNP_VENDOR_SRC_USB = 0x02;
}

//@formatter:off
// Combined report map (keyboard + gamepad)
static const uint8_t hidReportMapData[] PROGMEM = {

    // **** Keyboard (Report ID 1) ****

    0x05, 0x01,               // Usage Page (Generic Desktop)
    0x09, 0x06,               // Usage (Keyboard)
    0xA1, 0x01,               // Collection (Application)
    0x85, static_cast<uint8_t>(REPORT_ID_KEYBOARD), // Report ID

    // Modifiers (Ctrl, Shift, Alt, Meta)
    0x05, 0x07,               // Usage Page (Keyboard)
    0x19, 0xE0,               // Usage Minimum (Left Control)
    0x29, 0xE7,               // Usage Maximum (Right Meta)
    0x15, 0x00,               // Logical Minimum 0
    0x25, 0x01,               // Logical Maximum 1
    0x95, 0x08,               // Report Count 8
    0x75, 0x01,               // Report Size 1 bit
    0x81, 0x02,               // Input (Data, Variable, Absolute)

    // 6 simultaneous keys (keycode table)
    0x05, 0x07,               // Usage Page (Keyboard)
    0x19, 0x00,               // Usage Minimum (0 = No events)
    0x29, 0xFF,               // Usage Maximum (0xFF)
    0x15, 0x00,               // Logical Minimum 0
    0x26, 0xFF, 0x00,         // Logical Maximum 0x00FF
    0x95, 0x06,               // Report Count 6
    0x75, 0x08,               // Report Size 8 bits
    0x81, 0x00,               // Input (Data, Array, Absolute)

    // End Collection
    0xC0,

    // **** Gamepad (Report ID 2) ****
    0x05, 0x01,               // Usage Page (Generic Desktop)
    0x09, 0x05,               // Usage (Gamepad)
    0xA1, 0x01,               // Collection (Application)
    0x85, static_cast<uint8_t>(REPORT_ID_GAMEPAD),  // Report ID

    // 16 buttons
    0x05, 0x09,               // Usage Page (Button)
    0x19, 0x01,               // Usage Minimum (Button 1)
    0x29, 0x10,               // Usage Maximum (Button 16)
    0x15, 0x00,               // Logical Minimum 0
    0x25, 0x01,               // Logical Maximum 1
    0x75, 0x01,               // Report Size 1 bit
    0x95, 0x10,               // Report Count 16
    0x81, 0x02,               // Input (Data, Variable, Absolute)

    // D-Pad (hat switch)
    0x05, 0x01,               // Usage Page (Generic Desktop)
    0x09, 0x39,               // Usage (Hat switch)
    0x15, 0x00,               // Logical Min 0
    0x25, 0x07,               // Logical Max 7 (8 directions)
    0x75, 0x04,               // Report Size 4 bits
    0x95, 0x01,               // Report Count 1
    0x81, 0x42,               // Input (Data, Variable, Absolute, Null state)
    0x75, 0x04,               // Report Size 4 bits (padding)
    0x95, 0x01,               // Report Count 1
    0x81, 0x03,               // Input (Constant, Variable, Absolute)

    // Left stick: X, Y
    0x05, 0x01,               // Usage Page (Generic Desktop)
    0x09, 0x30,               // Usage X
    0x09, 0x31,               // Usage Y
    0x16, 0x00, 0x80,         // Logical Min -32768
    0x26, 0xFF, 0x7F,         // Logical Max 32767
    0x75, 0x10,               // Report Size 16 bits
    0x95, 0x02,               // Report Count 2
    0x81, 0x02,               // Input (Data, Variable, Absolute)

    // Right stick: Rx, Ry
    0x05, 0x01,               // Usage Page (Generic Desktop)
    0x09, 0x33,               // Usage Rx
    0x09, 0x34,               // Usage Ry
    0x16, 0x00, 0x80,         // Logical Min -32768
    0x26, 0xFF, 0x7F,         // Logical Max 32767
    0x75, 0x10,               // Report Size 16 bits
    0x95, 0x02,               // Report Count 2
    0x81, 0x02,               // Input (Data, Variable, Absolute)

    // Triggers LT/RT: Z, Rz
    0x05, 0x01,               // Usage Page (Generic Desktop)
    0x09, 0x32,               // Usage Z (LT)
    0x09, 0x35,               // Usage Rz (RT)
    0x16, 0x00, 0x00,         // Logical Min 0
    0x26, 0xFF, 0x7F,         // Logical Max 32767
    0x75, 0x10,               // Report Size 16 bits
    0x95, 0x02,               // Report Count 2
    0x81, 0x02,               // Input (Data, Variable, Absolute)

    // End Collection
    0xC0
};
//@formatter:on


// Class implementation
BleHidController::BleHidController() = default;

BleHidController::~BleHidController() {
    if (m_hidDevice != nullptr) {
        delete m_hidDevice;
        m_hidDevice = nullptr;
    }
    // NimBLEServer is managed by NimBLEDevice and should not be deleted manually
    m_server = nullptr;
}


bool BleHidController::begin(const char* deviceName, const char* deviceManufacturer, const uint16_t vendorId, const uint16_t productId, const uint16_t version) {
    if (m_hidDevice != nullptr) {
        return false; // Already initialized
    }

    if (deviceName == nullptr || deviceManufacturer == nullptr) {
        return false;
    }

    NimBLEDevice::init(deviceName);
    NimBLEDevice::setPower(BLE_TX_POWER);
    NimBLEDevice::setSecurityAuth(true, false, false); // bonding, no MITM, no SC
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

    m_server = NimBLEDevice::createServer();
    if (m_server == nullptr) {
        return false;
    }

    m_server->setCallbacks(this);

    m_server->advertiseOnDisconnect(false);

    m_hidDevice = new NimBLEHIDDevice(m_server);
    if (m_hidDevice == nullptr) {
        return false;
    }
    m_hidDevice->setManufacturer(deviceManufacturer);
    m_hidDevice->setPnp(PNP_VENDOR_SRC_USB, vendorId, productId, version);
    m_hidDevice->setHidInfo(0x00, 0x01);
    m_hidDevice->setBatteryLevel(BATTERY_LEVEL);
    m_hidDevice->setReportMap(const_cast<uint8_t*>(hidReportMapData), sizeof(hidReportMapData));

    m_kbInputReport = m_hidDevice->getInputReport(REPORT_ID_KEYBOARD);
    m_gpInputReport = m_hidDevice->getInputReport(REPORT_ID_GAMEPAD);

    if (m_kbInputReport == nullptr || m_gpInputReport == nullptr) {
        return false;
    }

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    if (adv == nullptr) {
        return false;
    }
    adv->setName(deviceName);
    adv->addServiceUUID(m_hidDevice->getHidService()->getUUID());
    adv->addServiceUUID(m_hidDevice->getBatteryService()->getUUID());
    adv->setAppearance(GENERIC_HID); // !HERE Meta Quest 3 doesn't seem to accept HID_GAMEPAD
    adv->enableScanResponse(true);
    adv->start();

    return true;
}

void BleHidController::deleteAllBonds() {
    NimBLEDevice::deleteAllBonds();
}

void BleHidController::onConnect(NimBLEServer* pSrv, NimBLEConnInfo& connInfo) {
    m_deviceConnected = true;
    pSrv->updateConnParams(connInfo.getConnHandle(), 6, 7, 0, 600);
}

void BleHidController::onDisconnect(NimBLEServer* pSrv, NimBLEConnInfo& connInfo, int reason) {
    m_deviceConnected = false;
    NimBLEDevice::startAdvertising();
}

// ***************************************************************
// KEYBOARD API
// ***************************************************************

bool BleHidController::sendKeyboardState() {
    if (!m_deviceConnected || m_kbInputReport == nullptr) {
        return false;
    }
    m_kbInputReport->setValue(reinterpret_cast<uint8_t*>(&m_kbState), sizeof(m_kbState));
    return m_kbInputReport->notify();
}

/**
 * Add modifier bit and send report
 *
 * @param modifier
 */
bool BleHidController::keyModPress(uint8_t modifier) {
    if (modifier == 0 || !m_deviceConnected || m_kbInputReport == nullptr) {
        return false;
    }
    m_kbState.modifiers |= modifier;
    return sendKeyboardState();
}

/**
 * Remove modifier bit and send report
 *
 * @param modifier
 */
bool BleHidController::keyModRelease(uint8_t modifier) {
    if (modifier == 0 || !m_deviceConnected || m_kbInputReport == nullptr) {
        return false;
    }
    m_kbState.modifiers &= ~modifier;
    return sendKeyboardState();
}

/**
 * Add keycode to the first available slot in the keys array and send report
 *
 * @param keycode
 */
bool BleHidController::keyPress(uint8_t keycode) {
    if (keycode == 0 || keycode == static_cast<uint8_t>(KeyCode::NONE) || !m_deviceConnected || m_kbInputReport == nullptr) {
        return false;
    }

    // Verify if key already exists
    for (const unsigned char key : m_kbState.keys) {
        if (key == keycode) {
            return true; // Already pressed, no need to send again
        }
    }

    for (unsigned char& key : m_kbState.keys) {
        if (key == 0x00) {
            key = keycode;
            return sendKeyboardState();
        }
    }
    return false; // No available slot
}

/**
 * Remove keycode from the keys array and send report
 *
 * @param keycode
 */
bool BleHidController::keyRelease(uint8_t keycode) {
    if (keycode == 0 || !m_deviceConnected || m_kbInputReport == nullptr) {
        return false;
    }

    // Remove the keycode from the keys array
    for (unsigned char& key : m_kbState.keys) {
        if (key == keycode) {
            key = 0x00;
            return sendKeyboardState();
        }
    }
    return false; // Key not found
}

/**
 * Clear all active key presses and modifier states and send report
 */
bool BleHidController::keyReleaseAll() {
    if (!m_deviceConnected || m_kbInputReport == nullptr) {
        return false;
    }
    m_kbState = KeyReport{};
    return sendKeyboardState();
}


// ***************************************************************
//  GAMEPAD API
// ***************************************************************

bool BleHidController::sendGamepadState() {
    if (!m_deviceConnected || m_gpInputReport == nullptr) {
        return false;
    }
    m_gpInputReport->setValue(reinterpret_cast<uint8_t*>(&m_gpState), sizeof(m_gpState));
    return m_gpInputReport->notify();
}

bool BleHidController::buttonPress(uint16_t button) {
    if (button == 0 || !m_deviceConnected || m_gpInputReport == nullptr) {
        return false;
    }
    m_gpState.buttons |= button;
    return sendGamepadState();
}

bool BleHidController::buttonRelease(uint16_t button) {
    if (button == 0 || !m_deviceConnected || m_gpInputReport == nullptr) {
        return false;
    }
    m_gpState.buttons &= ~button;
    return sendGamepadState();
}

bool BleHidController::dpadPress(uint8_t dpad) {
    if (dpad > 0x0F || !m_deviceConnected || m_gpInputReport == nullptr) {
        return false;
    }
    m_gpState.dpad = dpad;
    return sendGamepadState();
}

bool BleHidController::dpadRelease() {
    if (!m_deviceConnected || m_gpInputReport == nullptr) {
        return false;
    }
    m_gpState.dpad = static_cast<uint8_t>(DpadDirection::CENTERED);
    return sendGamepadState();
}

bool BleHidController::setLeftStick(int16_t lx, int16_t ly, bool sendState) {
    m_gpState.leftX = lx;
    m_gpState.leftY = ly;
    if (sendState) {
        return sendGamepadState();
    }
    return true;
}

bool BleHidController::setRightStick(int16_t rx, int16_t ry, bool sendState) {
    m_gpState.rightX = rx;
    m_gpState.rightY = ry;
    if (sendState) {
        return sendGamepadState();
    }
    return true;
}

bool BleHidController::setLeftTrigger(int16_t lt, bool sendState) {
    if (lt < 0 || lt > 32767) {
        return false;
    }
    m_gpState.lt = lt;
    if (sendState) {
        return sendGamepadState();
    }
    return true;
}

bool BleHidController::setRightTrigger(int16_t rt, bool sendState) {
    if (rt < 0 || rt > 32767) {
        return false;
    }
    m_gpState.rt = rt;
    if (sendState) {
        return sendGamepadState();
    }
    return true;
}

bool BleHidController::sendGamepad(uint16_t buttons, uint8_t dpad, int16_t lx, int16_t ly, int16_t rx, int16_t ry, int16_t lt, int16_t rt) {
    if (lt < 0 || lt > 32767 || rt < 0 || rt > 32767) {
        return false;
    }
    m_gpState.buttons = buttons;
    m_gpState.dpad    = dpad & 0x0F;
    m_gpState.leftX   = lx;
    m_gpState.leftY   = ly;
    m_gpState.rightX  = rx;
    m_gpState.rightY  = ry;
    m_gpState.lt      = lt;
    m_gpState.rt      = rt;
    return sendGamepadState();
}
