#include <Arduino.h>
#include <Preferences.h>
#include "src/LedManager.h"
#include "src/BleHidController.h"
#include "src/AccelerometerManager.h"
#include "src/PlungerHandler.h"
#include "src/NudgeHandler.h"
#include "src/ButtonManager.h"
#include "src/ESP32_VR_Pinball_Controller.h"
#include "src/config.h"

constexpr uint32_t MODE_DEBOUNCE_MS = 700;

bool debugMode = false; // Press button Y on start to activate debug mode

BleHidController hidController;

// Mode configuration including flash storage
Preferences nvsConfig;
ControllerMode mode;
bool modeChanged        = false; // Flag to indicate if the controller mode has changed and needs to be saved to nvs
uint32_t lastModeChange = 0;     // Timestamp of the last controller mode change, used to throttle flash writes

AccelerometerManager accelManager;
PlungerHandler plungerHandler;
NudgeHandler nudgeHandler;
ButtonManager buttonManager;

// ISR handlers
volatile bool changeModeIRQ = false;
portMUX_TYPE changeModeMux = portMUX_INITIALIZER_UNLOCKED;
static void IRAM_ATTR onChangeModeISR() { 
    portENTER_CRITICAL_ISR(&changeModeMux);
    changeModeIRQ = true;
    portEXIT_CRITICAL_ISR(&changeModeMux);
}


void setup() {
    Serial.begin(115200);

    // LED initialization
    LedManager::setColor(LedColor::RED);

    // Initialize specific pins for debug mode and BLE bond deletion
    pinMode(BTN_Y_PIN, INPUT_PULLUP);
    pinMode(BTN_X_PIN, INPUT_PULLUP);
    delay(100);

    // Debug mode activation on Y button press
    if (digitalRead(BTN_Y_PIN) == LOW) {
        debugMode = true;
        LedManager::blink();
        Serial.println("[setup] Debug mode activated!");
    }

    // Change mode button initialization
    pinMode(CHANGE_MODE_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CHANGE_MODE_PIN), onChangeModeISR, RISING);

    // HID initialization and BLE bonds deletion on X button press
    if (!hidController.begin(DEVICE_NAME, DEVICE_MANUFACTURER)) {
        Serial.println("[setup] BLE HID initialization failed!");
        LedManager::blink(LedColor::RED, 10);
        ESP.restart();
    }

    if (digitalRead(BTN_X_PIN) == LOW) {
        BleHidController::deleteAllBonds();
        LedManager::blink();
        Serial.println("[setup] BLE bonds deleted!");
    }

    // Buttons initialization
    buttonManager.begin(hidController);

    // Plunger initialization
    plungerHandler.begin(hidController);

    // Accelerometer initialization
    // Ensure that the sensor remains stationary during calibration for accurate offset computation
    accelManager.begin();

    // Nudge initialization
    nudgeHandler.begin(hidController, accelManager);

    // NVS: open, read saved mode, then close to free the handle
    if (!nvsConfig.begin(NVS_NAMESPACE, true)) { // read-only
        Serial.println("[setup] NVS initialization failed! Using default mode.");
        setMode(ControllerMode::FX, true);
    } else {
        const auto nvsMode = static_cast<ControllerMode>(nvsConfig.getUChar("mode", static_cast<uint8_t>(ControllerMode::FX)));
        nvsConfig.end();
        Serial.printf("[setup] Loaded mode from flash: %d\n", nvsMode);
        setMode(nvsMode, true);
    }
}

void loop() {
    const auto currentMillis = millis();

    // If change mode button was pressed, cycle through modes
    portENTER_CRITICAL(&changeModeMux);
    bool modeChangeRequested = changeModeIRQ;
    changeModeIRQ = false;
    portEXIT_CRITICAL(&changeModeMux);

    if (modeChangeRequested) {
        setMode(static_cast<ControllerMode>((static_cast<uint8_t>(mode) + 1) % static_cast<uint8_t>(ControllerMode::count)));
    }

    // Save configuration if changed, regardless of BLE connection state
    // Open/write/close on each save to avoid holding the NVS handle open
    if (
        modeChanged &&
        currentMillis - lastModeChange > NVS_SAVE_INTERVAL_MS
    ) {
        Serial.printf("[loop] Saving mode %d to flash...\n", mode);
        if (nvsConfig.begin(NVS_NAMESPACE, false)) { // read-write
            if (nvsConfig.putUChar("mode", static_cast<uint8_t>(mode))) {
                Serial.println("[loop] Configuration saved!");
            } else {
                Serial.println("[loop] Failed to save configuration to NVS!");
            }
            nvsConfig.end();
        } else {
            Serial.println("[loop] Failed to open NVS for writing!");
        }
        modeChanged = false;
    }

    // Check BLE connection state before processing inputs
    static bool s_wasConnected = false;

    if (!hidController.isConnected()) {
        if (s_wasConnected) {
            s_wasConnected = false;
            LedManager::setColor(LedColor::RED);
        }
        yield();
        return;
    }

    if (!s_wasConnected) {
        // Restore LED color based on current mode when connection is established
        s_wasConnected = true;
        LedManager::setColor(LedManager::MODE_COLORS[static_cast<uint8_t>(mode)]);
    }

    // Handle nudge detection from accelerometer
    if (mode == ControllerMode::FX) {
        nudgeHandler.handleDigital(debugMode);
    }
    else {
        nudgeHandler.handleAnalog(debugMode);
    }

    // Handle button states
    buttonManager.handle(mode);

    // Handle plunger input (after buttons to avoid being overwritten)
    if (mode == ControllerMode::VPX) {
        plungerHandler.handle(debugMode);
    }

    // Send gamepad report
    sendGamepadReport();
}

/**
 * Set the controller's operating mode and perform associated configurations.
 *
 * This function updates the controller's mode to the specified value, performs any necessary
 * cleanup for the current mode, and applies the settings for the new mode. If `initialConfig`
 * is false, a debounce mechanism ensures that the mode cannot be changed more than once
 * within a MODE_DEBOUNCE_MS interval.
 *
 * When transitioning between modes (i.e., if `initialConfig` is false), all currently pressed
 * keys or buttons are released, and the d-pad is reset to its centered position. The LED color
 * is updated to reflect the new mode. Configuration change flags are marked if the mode change
 * is not part of the initial configuration.
 *
 * @param newMode The desired new mode for the controller. Must be a valid value from the
 * ControllerMode enumeration.
 * @param initialConfig A boolean indicating whether this mode change is part of the initial
 * configuration (true) or a runtime change initiated by user action (false).
 */
void setMode(const ControllerMode newMode, const bool initialConfig) {
    const uint32_t currentMillis     = millis();
    static uint32_t s_lastChangeTime = 0;
    if (
        !initialConfig &&
        currentMillis - s_lastChangeTime < MODE_DEBOUNCE_MS
    )
        return;
    s_lastChangeTime = currentMillis;

    // Release all keys/buttons and reset dpad to centered before switching mode
    if (!initialConfig) {
        hidController.keyReleaseAll();
        hidController.sendGamepad(static_cast<uint16_t>(GamepadButton::NONE), static_cast<uint8_t>(DpadDirection::CENTERED), 0, 0, 0, 0);
        Serial.println("[setMode] Releasing all keys and resetting dpad");
    }
    LedManager::setColor(LedManager::MODE_COLORS[static_cast<uint8_t>(newMode)]);

    mode = newMode;
    Serial.printf("[setMode] mode set to %d (initialConfig=%d)\n", newMode, initialConfig);
    if (!initialConfig) {
        modeChanged    = true;
        lastModeChange = currentMillis;
    }
}

/**
 * Sends the current gamepad HID report at a fixed rate.
 * Centralizes all gamepad state flushing to avoid redundant or conflicting sends.
 */
void sendGamepadReport() {
    const uint32_t now                 = micros();
    static uint32_t s_lastReportMicros = 0;
    if (now - s_lastReportMicros < GAMEPAD_REPORT_INTERVAL_US) return;
    s_lastReportMicros = now;

    hidController.sendGamepadState();
}
