#include <Arduino.h>
#include <Preferences.h>
#include <MPU6050.h>
#include "BleHidController.h"
#include "ESP32_VR_Pinball_Controller.h"
#include "config.h"

// #define DEBUG_ANALOG_NUDGE

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

MPU6050 mpu(MPU6050_ADDR);
BleHidController hid;
Preferences config;
NudgeState nudgeState;
ControllerMode mode;
NudgeProcess nudgeX, nudgeY; // Shared between analog and digital nudge handlers

bool configChanged        = false; // Flag to indicate if the configuration has changed and needs to be saved
uint32_t lastConfigChange = 0;     // Timestamp of the last configuration change, used to throttle flash writes

// ISR handlers
volatile bool changeModeIRQ = false;
static void IRAM_ATTR onChangeModeISR() { changeModeIRQ = true; }


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
    hid.begin(DEVICE_NAME, DEVICE_MANUFACTURER);

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

    // If change mode button was pressed, cycle through modes
    if (changeModeIRQ) {
        changeModeIRQ = false;
        setMode(static_cast<ControllerMode>((static_cast<uint8_t>(mode) + 1) % static_cast<uint8_t>(ControllerMode::count)));
    }

    // Save configuration if changed, regardless of BLE connection state
    if (
        configChanged &&
        currentMillis - lastConfigChange > CONFIG_SAVE_INTERVAL_MS
    ) {
        Serial.printf("[loop] Saving mode %d to flash...\n", mode);
        config.putUChar("mode", static_cast<uint8_t>(mode));
        Serial.println("[loop] Configuration saved!");
        configChanged = false;
    }

    // Check BLE connection state before processing inputs
    static bool wasConnected = true;

    if (!BleHidController::isConnected()) {
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

    // Handle nudge detection from accelerometer
    if (mode == ControllerMode::FX) {
        handleDigitalNudge();
    }
    else {
        handleAnalogNudge();
    }

    // Handle button states
    for (auto& button : buttons) {
        handleButton(button);
    }
}

void handleButton(ButtonInfo& button) {
    const auto currentMillis = millis();
    if (currentMillis - button.lastDebounceTime < BTN_DEBOUNCE_MS) {
        return;
    }

    if (const int reading = digitalRead(button.pin); reading != button.state) {
        button.state            = reading;
        button.lastDebounceTime = currentMillis;
        performButtonAction(getButtonAction(button), button.state == LOW);
    }
}

/**
 * Controller mode change handler
 *
 * @param newMode The new controller mode to switch to
 * @param initialConfig Indicates if this mode change is part of the initial configuration (true during setup) or a user-initiated change
 */
void setMode(const ControllerMode newMode, const bool initialConfig) {
    const uint32_t currentMillis   = millis();
    static uint32_t lastChangeTime = 0;
    if (
        !initialConfig &&
        currentMillis - lastChangeTime < 700
    )
        return;
    lastChangeTime = currentMillis;

    // Release all keys/buttons and reset dpad to centered before switching mode
    if (!initialConfig) {
        hid.keyReleaseAll();
        hid.sendGamepad(BTN_NONE, DPAD_CENTERED, 0, 0, 0, 0);
        Serial.println("[setMode] Releasing all keys and resetting dpad");
    }
    setLedColor(MODE_COLORS[static_cast<uint8_t>(newMode)]);

    mode = newMode;
    Serial.printf("[setMode] mode set to %d (initialConfig=%d)\n", static_cast<int>(newMode), static_cast<int>(initialConfig));
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
    if (!Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN)) {
        Serial.println("I2C init failed!");
        return;
    }

    Wire.setClock(400000); // Fast I²C

    // Initialize MPU6050
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        return;
    }

    Serial.println("Calibrating... Keep the MPU6050 sensor still.");
    mpu.CalibrateAccel();
}


/**
 * Reads raw accelerometer values (X and Y axes) from the MPU6050 sensor
 * and apply the configured rotation to align with the physical orientation
 * of the sensor.
 *
 * @param[out] x Reference to store the X-axis accelerometer value
 * @param[out] y Reference to store the Y-axis accelerometer value
 */
void readAccelRaw(int16_t& x, int16_t& y) {
    int16_t rawX, rawY, rawZ;
    mpu.getAcceleration(&rawX, &rawY, &rawZ);

    static_assert(SENSOR_ROTATION == 0 ||
                  SENSOR_ROTATION == 90 ||
                  SENSOR_ROTATION == 180 ||
                  SENSOR_ROTATION == 270,
                  "SENSOR_ROTATION must be 0, 90, 180 or 270");

    //@formatter:off
    if constexpr      (SENSOR_ROTATION ==  90) { x = rawY;                        y = static_cast<int16_t>(-rawX); }
    else if constexpr (SENSOR_ROTATION == 180) { x = static_cast<int16_t>(-rawX); y = static_cast<int16_t>(-rawY); }
    else if constexpr (SENSOR_ROTATION == 270) { x = static_cast<int16_t>(-rawY); y = rawX;  }
    else                                       { x = rawX;                        y = rawY;  }
    //@formatter:on
}

/**
 * Samples the accelerometer at the configured rate and processes both axes
 * through the shared NudgeProcess instances (jitter filter + DC blocker + velocity integration)
 *
 * @return true if a new sample was processed, false if the sampling interval has not elapsed yet
 */
bool sampleNudge() {
    const uint32_t now = micros();

    static uint32_t lastSampleMicros = 0;
    if (now - lastSampleMicros < NUDGE_SAMPLE_INTERVAL_US) return false;
    lastSampleMicros = now;

    int16_t rx, ry;
    readAccelRaw(rx, ry);
    nudgeX.process(rx, now);
    nudgeY.process(ry, now);
    return true;
}

#if 0
void handleAnalogNudgeBAK() {
    static NudgeProcess nudgeX, nudgeY;
    static uint32_t lastSampleMicros = 0;
    static uint32_t lastReportMicros = 0;

    const uint32_t now = micros();

    /**
     * Samples
     */
    if (now - lastSampleMicros >= NUDGE_SAMPLE_INTERVAL_US) {
        lastSampleMicros = now;

        int16_t rx, ry;
        readAccelRaw(rx, ry);

        nudgeX.process(rx, now);
        nudgeY.process(ry, now);
    }

    /**
     * Report
     */
    if (now - lastReportMicros >= ANALOG_NUDGE_REPORT_INTERVAL_US) {
        lastReportMicros = now;

        int16_t leftX  = 0, leftY  = 0,
                rightX = 0, rightY = 0;

        float accX = nudgeX.acceleration;
        float accY = nudgeY.acceleration;
        float velX = nudgeX.velocity;
        float velY = nudgeY.velocity;

        // Left stick: acceleration (Classic)
        leftX = static_cast<int16_t>(std::clamp(accX * ANALOG_NUDGE_ACCELERATION_SCALE, -32767.0f, 32767.0f));
        leftY = static_cast<int16_t>(std::clamp(accY * ANALOG_NUDGE_ACCELERATION_SCALE, -32767.0f, 32767.0f));

        // Right stick: velocity (VPX)
        rightX = static_cast<int16_t>(std::clamp(velX * ANALOG_NUDGE_VELOCITY_SCALE, -32767.0f, 32767.0f));
        rightY = static_cast<int16_t>(std::clamp(velY * ANALOG_NUDGE_VELOCITY_SCALE, -32767.0f, 32767.0f));

        // Send both axes together
        hid.setLeftStick(leftX, leftY, false);
        hid.setRightStick(rightX, rightY, false);
        hid.sendGamepadState();

#ifdef DEBUG_ANALOG_NUDGE
static uint32_t lastPrint = 0, lastReset = 0;

static float maxAccX     = 0.0f, maxAccY = 0.0f,
             maxVelX     = 0.0f, maxVelY = 0.0f;
static int16_t maxLeftX  = 0, maxLeftY   = 0,
               maxRightX = 0, maxRightY  = 0;

        if (fabsf(nudgeX.acceleration) > fabsf (maxAccX)) maxAccX= nudgeX.acceleration;
        if (fabsf(nudgeY.acceleration) > fabsf (maxAccY)) maxAccY= nudgeY.acceleration;

        if (fabsf(nudgeX.velocity) > fabsf (maxVelX)) maxVelX= nudgeX.velocity;
        if (fabsf(nudgeY.velocity) > fabsf (maxVelY)) maxVelY= nudgeY.velocity;

        if (abs(leftX)> abs (maxLeftX)) maxLeftX= leftX;
        if (abs(leftY)> abs (maxLeftY)) maxLeftY= leftY;

        if (abs(rightX)> abs (maxRightX)) maxRightX= rightX;
        if (abs(rightY)> abs (maxRightY)) maxRightY= rightY;

        if (now- lastPrint> 1000000) {
            Serial.printf(
                "maxAcc[%7.1f, %7.1f] / maxVel[%7.1f, %7.1f] "
                "*** maxLeft[%6d, %6d] / maxRight[%6d, %6d]\n",
                maxAccX, maxAccY, maxVelX, maxVelY,
                maxLeftX, maxLeftY, maxRightX, maxRightY);
            lastPrint = now;

            if (now - lastReset > 5000000) {
                Serial.println("\nResetting debug counters...");
                maxAccX   = 0.0f;
                maxAccY   = 0.0f;
                maxVelX   = 0.0f;
                maxVelY   = 0.0f;
                maxLeftX  = 0;
                maxLeftY  = 0;
                maxRightX = 0;
                maxRightY = 0;
                lastReset = now;
            }
        }
#endif
}
}
#endif

void handleAnalogNudge() {
    const uint32_t now = micros();

    sampleNudge();

    static uint32_t lastReportMicros = 0;

    if (now - lastReportMicros < ANALOG_NUDGE_REPORT_INTERVAL_US) return;
    lastReportMicros = now;

    const float accX = nudgeX.acceleration;
    const float accY = nudgeY.acceleration;
    const float velX = nudgeX.velocity;
    const float velY = nudgeY.velocity;

    // Left stick: acceleration (Classic)
    const int16_t leftX = static_cast<int16_t>(std::clamp(accX * ANALOG_NUDGE_ACCELERATION_SCALE, -32767.0f, 32767.0f));
    const int16_t leftY = static_cast<int16_t>(std::clamp(accY * ANALOG_NUDGE_ACCELERATION_SCALE, -32767.0f, 32767.0f));

    // Right stick: velocity (VPX)
    const int16_t rightX = static_cast<int16_t>(std::clamp(velX * ANALOG_NUDGE_VELOCITY_SCALE, -32767.0f, 32767.0f));
    const int16_t rightY = static_cast<int16_t>(std::clamp(velY * ANALOG_NUDGE_VELOCITY_SCALE, -32767.0f, 32767.0f));

    // Send both axes together
    hid.setLeftStick(leftX, leftY, false);
    hid.setRightStick(rightX, rightY, false);
    hid.sendGamepadState();

#ifdef DEBUG_ANALOG_NUDGE
    static uint32_t lastPrint = 0, lastReset = 0;

    static float maxAccX     = 0.0f, maxAccY = 0.0f,
                 maxVelX     = 0.0f, maxVelY = 0.0f;
    static int16_t maxLeftX  = 0, maxLeftY   = 0,
                   maxRightX = 0, maxRightY  = 0;

    if (fabsf(nudgeX.acceleration) > fabsf(maxAccX)) maxAccX = nudgeX.acceleration;
    if (fabsf(nudgeY.acceleration) > fabsf(maxAccY)) maxAccY = nudgeY.acceleration;

    if (fabsf(nudgeX.velocity) > fabsf(maxVelX)) maxVelX = nudgeX.velocity;
    if (fabsf(nudgeY.velocity) > fabsf(maxVelY)) maxVelY = nudgeY.velocity;

    if (abs(leftX) > abs(maxLeftX)) maxLeftX = leftX;
    if (abs(leftY) > abs(maxLeftY)) maxLeftY = leftY;

    if (abs(rightX) > abs(maxRightX)) maxRightX = rightX;
    if (abs(rightY) > abs(maxRightY)) maxRightY = rightY;

    if (now - lastPrint > 1000000) {
        Serial.printf(
            "maxAcc[%7.1f, %7.1f] / maxVel[%7.1f, %7.1f] "
            "*** maxLeft[%6d, %6d] / maxRight[%6d, %6d]\n",
            maxAccX, maxAccY, maxVelX, maxVelY,
            maxLeftX, maxLeftY, maxRightX, maxRightY);
        lastPrint = now;

        if (now - lastReset > 5000000) {
            Serial.println("\nResetting debug counters...");
            maxAccX   = 0.0f;
            maxAccY   = 0.0f;
            maxVelX   = 0.0f;
            maxVelY   = 0.0f;
            maxLeftX  = 0;
            maxLeftY  = 0;
            maxRightX = 0;
            maxRightY = 0;
            lastReset = now;
        }
    }
#endif
}


/**
 * Handles digital nudge input for FX by detecting when filtered accelerometer
 * values exceed a threshold and sending corresponding key presses.
 *
 * Sampling and state evaluation run at different rates:
 * - Sampling (200 Hz): reads the accelerometer, applies jitter filter and DC blocker,
 *   and accumulates the peak acceleration over the current evaluation window
 * - Evaluation (50 Hz): checks the accumulated peak against thresholds to trigger
 *   or release a nudge key, then resets the peak accumulators
 */
#if 0
void handleDigitalNudgeBAK() {
    constexpr float DC_ADAPT_TIME_s = 0.3f; // DC removal adaptation time — 200-500 ms recommended

    const uint32_t now = micros();

    static JitterFilter jitterX, jitterY;

    static bool dcInitialized = false;

    static float dcX   = 0.0f, dcY   = 0.0f;
    static float peakX = 0.0f, peakY = 0.0f;

    static uint32_t lastSampleMicros = 0;

    /**
     * Samples
     */
    if (now - lastSampleMicros >= NUDGE_SAMPLE_INTERVAL_US) {
        const float dt   = static_cast<float>(now - lastSampleMicros) * 1e-6f;
        lastSampleMicros = now;

        int16_t rawX, rawY;
        readAccelRaw(rawX, rawY);

        const float stableX = static_cast<float>(jitterX.process(rawX));
        const float stableY = static_cast<float>(jitterY.process(rawY));

        // DC Blocker: removes gravity and static tilt offset
        if (!dcInitialized) {
            dcX           = stableX;
            dcY           = stableY;
            dcInitialized = true;
        }
        else {
            const float alpha = std::clamp(dt / DC_ADAPT_TIME_s, 0.0f, 1.0f);
            dcX               += alpha * (stableX - dcX);
            dcY               += alpha * (stableY - dcY);
        }

        const float filtX = stableX - dcX;
        const float filtY = stableY - dcY;

        // Accumulate peak for direction detection over the evaluation window
        if (fabsf(filtX) > fabsf(peakX)) peakX = filtX;
        if (fabsf(filtY) > fabsf(peakY)) peakY = filtY;
    }

    /**
     * State evaluation
     */
    static uint32_t lastEvalMicros = 0;
    if (now - lastEvalMicros < DIGITAL_NUDGE_EVAL_INTERVAL_US) return;
    lastEvalMicros = now;

    const float absPeakX      = fabsf(peakX);
    const float absPeakY      = fabsf(peakY);
    const bool aboveThreshold = (absPeakX > DIGITAL_NUDGE_THRESHOLD || absPeakY > DIGITAL_NUDGE_THRESHOLD);
    const uint32_t nowMs      = millis();

    // Nudge trigger
    if (
        aboveThreshold && !nudgeState.isNudging &&
        (nowMs - nudgeState.lastNudgeMillis > DIGITAL_NUDGE_COOLDOWN_MS)
    ) {
        nudgeState.lastNudgeMillis = nowMs;
        nudgeState.isNudging       = true;
        nudgeState.nudgeKey        = 0;

        // Determine nudge direction from the dominant peak axis
        if (absPeakY >= absPeakX) {
            if (peakY > 0) nudgeState.nudgeKey = static_cast<uint8_t>(FxNudgeKey::FORWARD);
        }
        else {
            nudgeState.nudgeKey = static_cast<uint8_t>(peakX < 0 ? FxNudgeKey::LEFT : FxNudgeKey::RIGHT);
        }
        if (nudgeState.nudgeKey != 0) hid.keyPress(nudgeState.nudgeKey);
    }
    // Nudge release (hysteresis)
    else if (
        nudgeState.isNudging &&
        absPeakX < DIGITAL_NUDGE_RELEASE_THRESHOLD &&
        absPeakY < DIGITAL_NUDGE_RELEASE_THRESHOLD &&
        (nowMs - nudgeState.lastNudgeMillis > DIGITAL_NUDGE_RESET_MS)
    ) {
        nudgeState.isNudging = false;
        if (nudgeState.nudgeKey != 0) {
            hid.keyRelease(nudgeState.nudgeKey);
            nudgeState.nudgeKey = 0;
        }
    }

    // Reset peak accumulators for the next evaluation window
    peakX = 0.0f;
    peakY = 0.0f;
}
#endif

void handleDigitalNudge() {
    const uint32_t now = micros();

    static float peakX = 0.0f, peakY = 0.0f;

    // Accumulate peak acceleration for direction detection over the evaluation window
    if (sampleNudge()) {
        if (fabsf(nudgeX.acceleration) > fabsf(peakX)) peakX = nudgeX.acceleration;
        if (fabsf(nudgeY.acceleration) > fabsf(peakY)) peakY = nudgeY.acceleration;
    }

    /**
     * State evaluation
     */
    static uint32_t lastEvalMicros = 0;
    if (now - lastEvalMicros < DIGITAL_NUDGE_EVAL_INTERVAL_US) return;
    lastEvalMicros = now;

    const float absPeakX      = fabsf(peakX);
    const float absPeakY      = fabsf(peakY);
    const bool aboveThreshold = absPeakX > static_cast<float>(DIGITAL_NUDGE_THRESHOLD) || absPeakY > static_cast<float>(DIGITAL_NUDGE_THRESHOLD);
    const uint32_t nowMs      = millis();

    // Nudge trigger
    if (
        aboveThreshold && !nudgeState.isNudging &&
        nowMs - nudgeState.lastNudgeMillis > DIGITAL_NUDGE_COOLDOWN_MS
    ) {
        nudgeState.lastNudgeMillis = nowMs;
        nudgeState.isNudging       = true;
        nudgeState.nudgeKey        = 0;

        // Determine nudge direction from the dominant peak axis
        if (absPeakY >= absPeakX) {
            if (peakY > 0) nudgeState.nudgeKey = static_cast<uint8_t>(FxNudgeKey::FORWARD);
        }
        else {
            nudgeState.nudgeKey = static_cast<uint8_t>(peakX < 0 ? FxNudgeKey::LEFT : FxNudgeKey::RIGHT);
        }
        if (nudgeState.nudgeKey != 0) hid.keyPress(nudgeState.nudgeKey);
    }
    // Nudge release (hysteresis)
    else if (
        nudgeState.isNudging &&
        absPeakX < static_cast<float>(DIGITAL_NUDGE_RELEASE_THRESHOLD) &&
        absPeakY < static_cast<float>(DIGITAL_NUDGE_RELEASE_THRESHOLD) &&
        nowMs - nudgeState.lastNudgeMillis > DIGITAL_NUDGE_RESET_MS
    ) {
        nudgeState.isNudging = false;
        if (nudgeState.nudgeKey != 0) {
            hid.keyRelease(nudgeState.nudgeKey);
            nudgeState.nudgeKey = 0;
        }
    }

    // Reset peak accumulators for the next evaluation window
    peakX = 0.0f;
    peakY = 0.0f;
}

void performButtonAction(const ButtonAction& action, const bool isPressed) {
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
        case ActionType::GAMEPAD_LT:
            if (isPressed)                  hid.setLeftTrigger(1023);
            else                            hid.setLeftTrigger(0);
            break;
        case ActionType::GAMEPAD_RT:
            if (isPressed)                  hid.setRightTrigger(1023);
            else                            hid.setRightTrigger(0);
            break;
        case ActionType::NONE:
        default:
            break;
        //@formatter:on
    }
}

ButtonAction getButtonAction(const ButtonInfo& button) {
    switch (mode) {
        //@formatter:off
        case ControllerMode::FX:
            return {.type = ActionType::KEYBOARD_KEY, .keyCode = static_cast<uint8_t>(button.fxKey)};
        case ControllerMode::VPX:
            return {.type = ActionType::KEYBOARD_KEY, .keyCode = static_cast<uint8_t>(button.vpxKey)};
        case ControllerMode::CLASSIC:
            if (button.type == ButtonType::DPAD)
                return {.type = ActionType::GAMEPAD_DPAD, .dpadValue = static_cast<uint8_t>(button.classicCode)};
            if (static_cast<uint16_t>(button.classicCode) == TRIGGER_LEFT)
                return {.type = ActionType::GAMEPAD_LT};
            if (static_cast<uint16_t>(button.classicCode) == TRIGGER_RIGHT)
                return {.type = ActionType::GAMEPAD_RT};
            return {.type = ActionType::GAMEPAD_BUTTON, .buttonCode = static_cast<uint16_t>(button.classicCode)};
        default:
            return {.type = ActionType::NONE};
        //@formatter:on
    }
}
