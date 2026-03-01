#include <Arduino.h>
#include <Preferences.h>
#include <MPU6050.h>
#include "BleHidController.h"
#include "ESP32_VR_Pinball_Controller.h"
#include "config.h"


// ###########################################################################
// Array of button configurations
// ###########################################################################
constexpr uint8_t NUM_BUTTONS   = 15;
ButtonInfo buttons[NUM_BUTTONS] = {
    //@formatter:off
    //                                            Classic                      FX                      VPX
    {BTN_A_PIN,               ButtonType::BUTTON, ClassicBtn::A,               FxKey::A,               VpxKey::A,               HIGH, 0},
    {BTN_B_PIN,               ButtonType::BUTTON, ClassicBtn::B,               FxKey::B,               VpxKey::B,               HIGH, 0},
    {BTN_X_PIN,               ButtonType::BUTTON, ClassicBtn::X,               FxKey::X,               VpxKey::X,               HIGH, 0},
    {BTN_Y_PIN,               ButtonType::BUTTON, ClassicBtn::Y,               FxKey::Y,               VpxKey::Y,               HIGH, 0},
    {BTN_SELECT_PIN,          ButtonType::BUTTON, ClassicBtn::SELECT,          FxKey::SELECT,          VpxKey::SELECT,          HIGH, 0},
    {BTN_START_PIN,           ButtonType::BUTTON, ClassicBtn::START,           FxKey::START,           VpxKey::START,           HIGH, 0},
    {BTN_LAUNCH_PIN,          ButtonType::BUTTON, ClassicBtn::LAUNCH,          FxKey::LAUNCH,          VpxKey::LAUNCH,          HIGH, 0},
    {BTN_LEFT_FLIPPER_PIN,    ButtonType::BUTTON, ClassicBtn::LEFT_FLIPPER,    FxKey::LEFT_FLIPPER,    VpxKey::LEFT_FLIPPER,    HIGH, 0},
    {BTN_RIGHT_FLIPPER_PIN,   ButtonType::BUTTON, ClassicBtn::RIGHT_FLIPPER,   FxKey::RIGHT_FLIPPER,   VpxKey::RIGHT_FLIPPER,   HIGH, 0},
    {BTN_LEFT_MAGNASAVE_PIN,  ButtonType::BUTTON, ClassicBtn::LEFT_MAGNASAVE,  FxKey::LEFT_MAGNASAVE,  VpxKey::LEFT_MAGNASAVE,  HIGH, 0},
    {BTN_RIGHT_MAGNASAVE_PIN, ButtonType::BUTTON, ClassicBtn::RIGHT_MAGNASAVE, FxKey::RIGHT_MAGNASAVE, VpxKey::RIGHT_MAGNASAVE, HIGH, 0},
    {DPAD_UP_PIN,             ButtonType::DPAD,   ClassicBtn::UP,              FxKey::UP,              VpxKey::UP,              HIGH, 0},
    {DPAD_DOWN_PIN,           ButtonType::DPAD,   ClassicBtn::DOWN,            FxKey::DOWN,            VpxKey::DOWN,            HIGH, 0},
    {DPAD_LEFT_PIN,           ButtonType::DPAD,   ClassicBtn::LEFT,            FxKey::LEFT,            VpxKey::LEFT,            HIGH, 0},
    {DPAD_RIGHT_PIN,          ButtonType::DPAD,   ClassicBtn::RIGHT,           FxKey::RIGHT,           VpxKey::RIGHT,           HIGH, 0},
    //@formatter:on
};
static_assert(NUM_BUTTONS == std::size(buttons), "NUM_BUTTONS mismatch");
// ###########################################################################

MPU6050 mpu;
BleHidController hid;
Preferences config;
NudgeState nudgeState;
ControllerMode mode;

bool configChanged             = false; // Flag to indicate if the configuration has changed and needs to be saved
unsigned long lastConfigChange = 0;     // Timestamp of the last configuration change, used to throttle flash writes

// ISR handlers
volatile bool changeModeIRQ = false;
static void IRAM_ATTR onChangeModeISR() { changeModeIRQ = true; }

volatile bool motionIRQ = false;
static void IRAM_ATTR onMotionISR() { motionIRQ = true; }


// Calibration results (populated by calibrateSensor, used by handleNudgeDetection) //!HERE
int16_t accelOffsetX = 0; // Mean X at rest (raw units)
int16_t accelOffsetY = 0; // Mean Y at rest (raw units)
float accelSigmaX    = 0; // Standard deviation X at rest
float accelSigmaY    = 0; // Standard deviation Y at rest

void setup() {
    Serial.begin(115200);

    setLedColor(LedColor::RED);

    // Initialize change mode button
    pinMode(CHANGE_MODE_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CHANGE_MODE_PIN), onChangeModeISR, RISING);

    // Initialize buttons
    for (const auto& button : buttons) {
        pinMode(button.pin, INPUT_PULLUP);
    }

    // Initialize HID
    hid.begin(DEVICE_NAME, DEVICE_MANUFACTURER, 0x1234, 0x5678); //!HERE

    // Initialize accelerometer
    setupAccelerometer();

    // Load saved mode
    config.begin("ctrl_cfg", false);
    const auto savedMode = static_cast<ControllerMode>(config.getUChar("mode", static_cast<uint8_t>(ControllerMode::FX)));
    Serial.printf("[setup] Loaded mode from flash: %d\n", static_cast<int>(savedMode));
    setMode(savedMode, true);
}

void loop() {
    const auto currentMillis = millis();

    // Save configuration if changed, regardless of BLE connection state
    if (configChanged && (currentMillis - lastConfigChange > CONFIG_SAVE_INTERVAL_MS)) {
        Serial.printf("[loop] Saving mode %d to flash...\n", mode);
        config.putUChar("mode", static_cast<uint8_t>(mode));
        Serial.println("[loop] Configuration saved!");
        configChanged = false;
    }

    // Check BLE connection state before processing inputs
    static bool wasConnected = true;

    if (!hid.isConnected()) {
        if (wasConnected) {
            wasConnected = false;
            setLedColor(LedColor::RED);
        }
        delay(1000);
        return;
    }

    if (!wasConnected) {
        // Restore LED color based on current mode when connection is established
        wasConnected = true;
        setLedColor(MODE_COLORS[static_cast<uint8_t>(mode)]);
    }

    // If change mode button was pressed, cycle through modes
    if (changeModeIRQ == true) {
        changeModeIRQ = false;
        setMode(static_cast<ControllerMode>((static_cast<uint8_t>(mode) + 1) % static_cast<uint8_t>(ControllerMode::count)));
    }

    // Handle nudge detection from accelerometer
    handleNudgeDetection(currentMillis);

    // Handle button states
    for (auto& button : buttons) {
        handleButton(button, currentMillis);
    }
}

/**
* Handles the debouncing and state change of a single button
*
* @param button Reference to the ButtonInfo struct representing the button to handle
* @param currentMillis Current time in milliseconds, used for debouncing logic
*/
void handleButton(ButtonInfo& button, const unsigned long currentMillis) {
    if (currentMillis - button.lastDebounceTime < BTN_DEBOUNCE_MS) {
        return;
    }

    if (const int reading = digitalRead(button.pin); reading != button.state) {
        button.state            = reading;
        button.lastDebounceTime = currentMillis;
        if (button.state == LOW) {
            performButtonAction(getButtonAction(button, mode), true);
        }
        else {
            performButtonAction(getButtonAction(button, mode), false);
        }
    }
}

/**
 * Controller mode change handler
 *
 * @param newMode The new controller mode to switch to
 * @param initialConfig Indicates if this mode change is part of the initial configuration (true during setup) or a user-initiated change
 */
void setMode(const ControllerMode newMode, const bool initialConfig) {
    const unsigned long currentMillis   = millis();
    static unsigned long lastChangeTime = 0;
    if (!initialConfig && (currentMillis - lastChangeTime < 700)) return;
    lastChangeTime = currentMillis;

    // Release all keys/buttons and reset dpad to centered before switching mode
    if (!initialConfig) {
        hid.keyReleaseAll();
        hid.sendGamepad(BTN_NONE, DPAD_CENTERED, 0, 0, 0, 0);
        Serial.println("[setMode] Releasing all keys and resetting dpad");
    }
    setLedColor(MODE_COLORS[static_cast<uint8_t>(newMode)]);

    mode = newMode;
    Serial.printf("[setMode] mode set to %d (initialConfig=%d)\n", static_cast<int>(mode), static_cast<int>(initialConfig));
    if (!initialConfig) {
        configChanged    = true;
        lastConfigChange = currentMillis;
    }
}

void setLedColor(const LedColor color) {
#ifdef RGB_BUILTIN
    switch (color) {
        //@formatter:off
        case LedColor::OFF:             rgbLedWrite(RGB_BUILTIN, 0, 0, 0); break;
        case LedColor::RED:             rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0); break;
        case LedColor::CLASSIC_MODE:
        case LedColor::GREEN:           rgbLedWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0); break;
        case LedColor::FX_MODE:
        case LedColor::BLUE:            rgbLedWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS); break;
        case LedColor::YELLOW:          rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, RGB_BRIGHTNESS, 0); break;
        case LedColor::VPX_MODE:
        case LedColor::PURPLE:          rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, RGB_BRIGHTNESS); break;
        case LedColor::CYAN:            rgbLedWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, RGB_BRIGHTNESS); break;
        case LedColor::WHITE:           rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, RGB_BRIGHTNESS, RGB_BRIGHTNESS); break;
        default: break;
        //@formatter:on
    }
#endif
}

/**
 * Initializes the MPU6050 accelerometer and configures its settings
 */
void setupAccelerometer() {
    // Start I2C interface
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000); // Fast I²C

    // Initialize MPU6050
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        return;
    }
    mpu.setFullScaleAccelRange(ACCEL_RANGE);
    mpu.setDLPFMode(DLPF_MODE);
    mpu.setMotionDetectionThreshold(MOTION_DETECTION_THRESHOLD);
    mpu.setMotionDetectionDuration(1);
    mpu.setIntMotionEnabled(true);

    pinMode(MPU_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), onMotionISR, RISING);

    calibrateSensor();
}


/**
 * Calibrates the MPU6050 accelerometer
 *
 * @param samples The number of samples to collect for calibration
 */
void calibrateSensor(const uint16_t samples) {
    Serial.println("Auto calibration: don't touch the sensor...");

    float sx = 0, sy = 0, sx2 = 0, sy2 = 0;

    mpu.CalibrateAccel(20);
    mpu.PrintActiveOffsets();

    int16_t x, y;
    uint16_t validSamples = 0;
    for (uint16_t i = 0; i < samples; i++) {
        if (!readAccelG(x, y)) continue;

        sx += x;
        sy += y;

        sx2 += static_cast<float>(x) * x;
        sy2 += static_cast<float>(y) * y;

        validSamples++;

        delay(2);
    }

    if (!validSamples) {
        Serial.println("Calibration failed: no valid samples!");
        return;
    }

    const float meanX = sx / validSamples;
    const float meanY = sy / validSamples;

    const float varX = sx2 / validSamples - meanX * meanX;
    const float varY = sy2 / validSamples - meanY * meanY;

    const float sigmaX = sqrtf(fmaxf(varX, 0.0f));
    const float sigmaY = sqrtf(fmaxf(varY, 0.0f));

    // Store calibration results for use in handleNudgeDetection
    accelOffsetX = static_cast<int16_t>(meanX);
    accelOffsetY = static_cast<int16_t>(meanY);
    accelSigmaX  = sigmaX;
    accelSigmaY  = sigmaY;

    Serial.printf("Calibration done (%d samples)!\n", validSamples);
    Serial.printf("Mean X/Y:\t\t%f\t\t%f\n", meanX, meanY);
    Serial.printf("Sigma X/Y:\t\t%f\t\t%f\n", sigmaX, sigmaY);
}


/**
 * Reads raw accelerometer values (X and Y axes) from the MPU6050 sensor
 *
 * @param[out] x Reference to store the X-axis accelerometer value
 * @param[out] y Reference to store the Y-axis accelerometer value
 * @return true if the read was successful and 4 bytes were received, false otherwise
 */
bool readAccelG(int16_t& x, int16_t& y) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_XOUT_H);                                                       // Position the register pointer to the accelerometer X-axis high byte
    if (Wire.endTransmission(false) != 0) return false;                             // End transmission but keep the connection active for reading
    if (Wire.requestFrom(MPU6050_ADDR, static_cast<uint8_t>(4)) != 4) return false; // Request 4 bytes (X high, X low, Y high, Y low) and check that we received exactly 4 bytes

    const uint8_t xh = Wire.read();
    const uint8_t xl = Wire.read();
    const uint8_t yh = Wire.read();
    const uint8_t yl = Wire.read();

    x = static_cast<int16_t>((xh << 8) | xl);
    y = static_cast<int16_t>((yh << 8) | yl);
    return true;
}

/**
 * Reads multiple accelerometer samples and returns the peak X and Y values
 *
 * @return The peak X and Y values
 */
AccelPeak getAccelPeak() {
    int16_t x, y, maxX = 0, maxY    = 0;
    uint16_t maxAbsX   = 0, maxAbsY = 0;
    for (int i = 0; i < static_cast<int>(NUDGE_SAMPLES); i++) {
        if (!readAccelG(x, y)) continue;
        x -= accelOffsetX;
        y -= accelOffsetY;
        if (const uint16_t ax = abs(x); ax > maxAbsX) {
            maxAbsX = ax;
            maxX    = x;
        }
        if (const uint16_t ay = abs(y); ay > maxAbsY) {
            maxAbsY = ay;
            maxY    = y;
        }
    }
    return {maxX, maxY};
}

void handleNudgeDetection(const unsigned long currentMillis) {
    if (motionIRQ) {
        motionIRQ = false;

        // Read the status to clear the MPU hardware interrupt and check that it is indeed a motion detection
        if (!(mpu.getIntStatus() & 0x40)) return;

        if (currentMillis - nudgeState.lastNudgeMillis > COOLDOWN_MS) {
            nudgeState.lastNudgeMillis = currentMillis;
            nudgeState.isNudging       = true;
            AccelPeak peak             = getAccelPeak();
            Serial.printf("Max X/Y:\t%d\t\t%d\n", peak.x, peak.y);

            nudgeState.nudgeKey = 0;

#if 0
            // Ignore movements that are within sensor noise (3-sigma threshold)
            const float noiseThreshX = 3.0f * accelSigmaX;
            const float noiseThreshY = 3.0f * accelSigmaY;

            if (abs(maxX) < noiseThreshX && abs(maxY) < noiseThreshY) {
                Serial.println("Motion below noise threshold, ignoring");
                nudgeState.isNudging = false;
            }
            else
#endif

            if (abs(peak.x) > abs(peak.y)) {
                switch (mode) {
                    //@formatter:off
                    case ControllerMode::FX:      nudgeState.nudgeKey = static_cast<uint8_t>(FxNudgeKey::FORWARD); break;
                    case ControllerMode::VPX:     nudgeState.nudgeKey = static_cast<uint8_t>(VpxNudgeKey::FORWARD); break;
                    case ControllerMode::CLASSIC: hid.setLeftStick(0, INT16_MIN); break;
                    default: break;
                    //@formatter:on
                }
                Serial.println("Nudge up");
            }
            else {
                int16_t nudgeY = peak.y;
                if (nudgeY > 0) {
                    switch (mode) {
                        //@formatter:off
                        case ControllerMode::FX:      nudgeState.nudgeKey = static_cast<uint8_t>(FxNudgeKey::LEFT); break;
                        case ControllerMode::VPX:     nudgeState.nudgeKey = static_cast<uint8_t>(VpxNudgeKey::LEFT); break;
                        case ControllerMode::CLASSIC: hid.setLeftStick(INT16_MAX, 0); break;
                        default: break;
                        //@formatter:on
                    }
                    Serial.println("Nudge left");
                }
                else {
                    switch (mode) {
                        //@formatter:off
                        case ControllerMode::FX:      nudgeState.nudgeKey = static_cast<uint8_t>(FxNudgeKey::RIGHT); break;
                        case ControllerMode::VPX:     nudgeState.nudgeKey = static_cast<uint8_t>(VpxNudgeKey::RIGHT); break;
                        case ControllerMode::CLASSIC: hid.setLeftStick(INT16_MIN, 0); break;
                        default: break;
                        //@formatter:on
                    }
                    Serial.println("Nudge right");
                }
            }
            if (nudgeState.nudgeKey != 0) hid.keyPress(nudgeState.nudgeKey);
        }
    }

    else if (nudgeState.isNudging && (currentMillis - nudgeState.lastNudgeMillis > NUDGE_RESET_MS)) {
        nudgeState.isNudging = false;
        if (nudgeState.nudgeKey != 0) {
            hid.keyRelease(nudgeState.nudgeKey);
            nudgeState.nudgeKey = 0;
        }
        else hid.setLeftStick(0, 0);
    }
}


void performButtonAction(const ButtonAction& action, bool isPressed) {
    switch (action.type) {
        //@formatter:off
        case ActionType::KEYBOARD_KEY:
            if (isPressed)                  hid.keyPress(action.keyCode);
            else                            hid.keyRelease(action.keyCode);
            break;
        case ActionType::GAMEPAD_BUTTON:
            if (isPressed)                  hid.buttonPress(action.buttonCode);
            else                            hid.buttonRelease(action.buttonCode);
            break;
        case ActionType::GAMEPAD_DPAD:
            if (isPressed)                  hid.dpadPress(action.dpadValue);
            else                            hid.dpadRelease();
            break;

        case ActionType::NONE:
        default:
            break;
        //@formatter:on
    }
}

ButtonAction getButtonAction(const ButtonInfo& button, ControllerMode mode) {
    switch (mode) {
        //@formatter:off
        case ControllerMode::FX:                    return {.type = ActionType::KEYBOARD_KEY, .keyCode = static_cast<uint8_t>(button.fxKey)};
        case ControllerMode::VPX:                   return {.type = ActionType::KEYBOARD_KEY, .keyCode = static_cast<uint8_t>(button.vpxKey)};
        case ControllerMode::CLASSIC:
            if (button.type == ButtonType::DPAD)    return {.type = ActionType::GAMEPAD_DPAD, .dpadValue = static_cast<uint8_t>(button.classicCode)};
            else                                    return {.type = ActionType::GAMEPAD_BUTTON, .buttonCode = static_cast<uint16_t>(button.classicCode)};
        default:                                    return {.type = ActionType::NONE};
        //@formatter:on
    }
}
