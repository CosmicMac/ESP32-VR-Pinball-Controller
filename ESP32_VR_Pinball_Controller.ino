#include <Arduino.h>
#include <Preferences.h>
#include "BleHidController.h"
#include "ESP32_VR_Pinball_Controller.h"
#include "config.h"

#if ACCEL_SENSOR_TYPE == ACCEL_SENSOR_MPU6050
#include <MPU6050.h>
MPU6050 mpu(ACCEL_SENSOR_ADDR);
#elif ACCEL_SENSOR_TYPE == ACCEL_SENSOR_LIS3DH
#include <7Semi_LIS3DH.h>
LIS3DH_7Semi Adx;
#endif


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

BleHidController hid;
Preferences config;
NudgeState nudgeState;
ControllerMode mode;
NudgeProcess nudgeX, nudgeY; // Shared between analog and digital nudge handlers

bool configChanged        = false; // Flag to indicate if the configuration has changed and needs to be saved
uint32_t lastConfigChange = 0;     // Timestamp of the last configuration change, used to throttle flash writes

int16_t offsetX = 0;
int16_t offsetY = 0;
int16_t offsetZ = 0;

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
        Serial.printf("[loop] Saving mode %d to flash...\n", static_cast<int>(mode));
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

/**
 * Handle the state and debouncing of a button.
 *
 * This function processes the input from a specified button by checking its current
 * state, applying a debounce mechanism, and invoking the appropriate action based
 * on its state change. If the button's state differs from its previously recorded
 * state and the debounce threshold has been satisfied, the button state is updated,
 * and the corresponding action is executed.
 *
 * @param button A reference to a ButtonInfo object containing the button's information,
 * including its pin, current state, and last debounce time.
 */
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
 * Set the controller's operating mode and perform associated configurations.
 *
 * This function updates the controller's mode to the specified value, performs any necessary
 * cleanup for the current mode, and applies the settings for the new mode. If `initialConfig`
 * is false, a debounce mechanism ensures that the mode cannot be changed more than once
 * within a 700ms interval.
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

/**
 * Set the color of the built-in RGB LED based on the specified color.
 *
 * This function updates the RGB LED's color using predefined brightness levels,
 * allowing for different visual indications depending on the current color setting.
 * The function relies on the RGB_BUILTIN macro to reference the built-in LED.
 *
 * Supported colors include:
 * - OFF: Turns the LED off.
 * - RED, GREEN, BLUE: Primary color settings.
 * - YELLOW, PURPLE, CYAN, WHITE: Secondary and mixed color settings.
 * - CLASSIC_MODE, FX_MODE, VPX_MODE: Modes associated with specific colors.
 *
 * @param color The desired LED color and/or mode. Must be one of the values defined in the LedColor enumeration.
 */
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
 * Setup and initialize the accelerometer based on the configured sensor type
 *
 * This function handles the initialization of the I2C interface, sets up
 * the accelerometer hardware (either MPU6050 or LIS3DH, depending on the
 * configured sensor type), and performs calibration if necessary.
 *
 * MPU6050-specific setup:
 * - Configures the Digital Low Pass Filter (DLPF) and sampling rate for the MPU6050 sensor.
 * - Verifies the sensor connection and runs an automatic calibration routine.
 *
 * LIS3DH-specific setup:
 * - Initializes the LIS3DH accelerometer with specific parameters, such as range, data rate,
 *   and resolution.
 * - Performs a multi-step calibration that computes accelerometer offsets based on sample averages
 *   before and after calibration.
 *
 * Calibration:
 * - For the LIS3DH sensor, a pre-calibration sample average is computed, offsets are calculated from
 *   a defined number of calibration samples, and a post-calibration average is used for verification.
 *
 * @note Ensure that the sensor remains stationary during calibration for accurate offset computation.
 * @note The function uses delays during calibration to gather consistent sensor readings.
 */
void setupAccelerometer() {
    // Start I2C interface
    if (!Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN)) {
        Serial.println("I2C init failed!");
        return;
    }

    Wire.setClock(400000); // Fast I²C

#if ACCEL_SENSOR_TYPE == ACCEL_SENSOR_MPU6050
    // Initialize MPU6050
    mpu.initialize();
    mpu.setDLPFMode(MPU6050_DLPF_BW_188); // DLPF bandwidth: 188Hz (allows up to 500Hz sample rate)
    mpu.setRate(1);                       // Sample rate divider: 1kHz internal sample rate / (Sample rate divider + 1) = 500Hz

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (true) delay(1000);
    }

    Serial.println("Calibrating... Keep the MPU6050 sensor still.");
    mpu.CalibrateAccel();
#elif ACCEL_SENSOR_TYPE == ACCEL_SENSOR_LIS3DH
    // Initialize LIS3DH
    if (!Adx.begin(Wire)) {
        Serial.println("LIS3DH init failed!");
        while (true) delay(1000);
    }

    Adx.setScale(RANGE_2G);
    Adx.setDataRate(ODR_400HZ);
    Adx.enableTemperature(false);
    Adx.setHighResolution(true);

    /**
     * Calibration
     */

    constexpr uint8_t CALIB_SAMPLES   = 64; // Number of samples used for calibration
    constexpr uint16_t CALIB_DELAY    = 10; // Delay (ms) between calibration samples
    constexpr uint8_t PREVIEW_SAMPLES = 10;

    int32_t preX = 0, preY = 0, preZ = 0;

    uint8_t preCount = 0;

    int16_t xRaw, yRaw, zRaw;

    // Average of samples before calibration
    for (uint8_t i = 0; i < PREVIEW_SAMPLES; i++) {
        if (Adx.readAccel(xRaw, yRaw, zRaw)) {
            preX += xRaw;
            preY += yRaw;
            preZ += zRaw;
            preCount++;
        }
        delay(CALIB_DELAY);
    }
    if (preCount > 0) {
        Serial.printf("Before calibration (avg %d samples) -> X: %d  Y: %d  Z: %d\n", preCount, preX / preCount, preY / preCount, preZ / preCount);
    }

    // Calibration: compute offsets over CALIB_SAMPLES readings
    int32_t sumX  = 0, sumY = 0, sumZ = 0;
    uint8_t count = 0;

    // First pass: compute mean (offsets)
    int16_t samples[CALIB_SAMPLES][3];
    for (uint8_t i = 0; i < CALIB_SAMPLES; i++) {
        if (Adx.readAccel(xRaw, yRaw, zRaw)) {
            samples[count][0] = xRaw;
            samples[count][1] = yRaw;
            samples[count][2] = zRaw;
            sumX              += xRaw;
            sumY              += yRaw;
            sumZ              += zRaw;
            count++;
        }
        delay(CALIB_DELAY);
    }

    if (count > 0) {
        offsetX = static_cast<int16_t>(sumX / count);
        offsetY = static_cast<int16_t>(sumY / count);
        offsetZ = static_cast<int16_t>(sumZ / count);
    }

    Serial.printf("Calibration done. Offsets -> X: %d  Y: %d  Z: %d\n", offsetX, offsetY, offsetZ);

    // Average of samples after calibration
    int32_t postX     = 0, postY = 0, postZ = 0;
    uint8_t postCount = 0;

    for (uint8_t i = 0; i < PREVIEW_SAMPLES; i++) {
        if (Adx.readAccel(xRaw, yRaw, zRaw)) {
            postX += xRaw - offsetX;
            postY += yRaw - offsetY;
            postZ += zRaw - offsetZ;
            postCount++;
        }
        delay(CALIB_DELAY);
    }
    if (postCount > 0) {
        Serial.printf("After calibration (avg %d samples) -> X: %d  Y: %d  Z: %d\n", postCount, postX / postCount, postY / postCount, postZ / postCount);
    }

#endif
}


/**
 * Reads raw acceleration values from the sensor and applies calibration, rotation, and flip adjustments.
 *
 * This function retrieves raw X, Y, and Z axis acceleration values from the selected sensor
 * and adjusts them based on predefined offsets, rotation, and axis flipping configurations.
 *
 * @param xr Reference to store the calibrated and adjusted X-axis acceleration
 * @param yr Reference to store the calibrated and adjusted Y-axis acceleration
 * @return True if the acceleration values are successfully read and adjusted; false otherwise
 */
bool readAccelRaw(int16_t& xr, int16_t& yr) {
    int16_t x, y, z;

#if ACCEL_SENSOR_TYPE == ACCEL_SENSOR_MPU6050
    mpu.getAcceleration(&x, &y, &z);
#elif ACCEL_SENSOR_TYPE == ACCEL_SENSOR_LIS3DH
    if (!Adx.readAccel(x, y, z)) {
        return false;
    }
#endif

    // Apply offsets
    x -= offsetX;
    y -= offsetY;
    //z -= offsetZ;

    // Rotation around Z axis (CCW, viewed from +Z)
    // zr = z;
    switch (ACCEL_SENSOR_ROTATION) {
        //@formatter:off
        case 0:     xr = x;     yr = y;     break;
        case 90:    xr = -y;    yr = x;     break; // 90° CCW: X -> -Y, Y -> X
        case 180:   xr = -x;    yr = -y;    break; // 180°: X -> -X, Y -> -Y
        case 270:   xr = y;     yr = -x;    break; // 270° CCW (90° CW): X -> Y, Y -> -X
        default:    xr = x;     yr = y;     break; // Unsupported angle → no rotation
        //@formatter:on
    }

    // Apply flips (after rotation)
    if (ACCEL_SENSOR_UPSIDEDOWN_X) {
        // Flip around X: Y and Z invert
        yr = -yr;
        // zr = -zr;
    }
    if (ACCEL_SENSOR_UPSIDEDOWN_Y) {
        // Flip around Y: X and Z invert
        xr = -xr;
        // zr = -zr;
    }
    return true;
}


/**
 * Processes input from the accelerometer at a defined sampling interval
 * and updates the nudge subsystem with the latest raw acceleration values.
 *
 * @return true if a new sample was read and processed, false if the sampling interval has not yet elapsed
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

/**
 * Handles processing of analog nudge inputs and updates gamepad stick positions.
 *
 * This method samples acceleration and velocity data for both X and Y axes from
 * the nudge sensors, processes them into scaled stick values, and sends the updated
 * state to the BLE HID controller. It incorporates timing constraints to regulate
 * the frequency of stick updates.
 *
 * If debugging is enabled, this method logs the maximum acceleration, velocity,
 * and stick values observed over specified intervals, with the ability to periodically
 * reset these counters.
 */
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
            "*** maxLeftStick[%6d, %6d] / maxRightStick[%6d, %6d]\n",
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
 * Processes digital nudge inputs for directional detection and state management.
 *
 * This method evaluates acceleration data sampled over a defined window to
 * detect peak values in both X and Y axes and determines if a nudge input exceeds
 * the configured threshold. Based on the dominant axis, it triggers a corresponding
 * directional key press and manages nudge release state with hysteresis.
 *
 * Key features include:
 * - Directional detection using peak acceleration values.
 * - Cooldown and reset intervals for stable nudge state transitions.
 * - Integration with HID controller for key press/release events.
 *
 * State and calculation flow:
 * 1. Accumulate peak values during sampling intervals.
 * 2. Evaluate state after a defined evaluation interval.
 * 3. Trigger nudge key events when conditions are met.
 * 4. Handle release hysteresis to reset the nudge state when thresholds are below the release limit.
 */
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

    // !HERE
    Serial.printf("[DEBUG] peakX=%.1f, peakY=%.1f, threshold=%d, isNudging=%d\n", absPeakX, absPeakY, DIGITAL_NUDGE_THRESHOLD, nudgeState.isNudging);

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

/**
 * Executes the specified button action based on the given input state.
 *
 * @param action A reference to the ButtonAction object, specifying the type of action and associated data such as key code, button code, or dpad value.
 * @param isPressed A boolean indicating whether the button is currently pressed (true) or released (false).
 */
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


/**
 * Determines the appropriate button action based on the current controller mode and button information.
 *
 * @param button Reference to the ButtonInfo object containing button-specific data, such as button type and associated codes for each controller mode.
 * @return A ButtonAction object representing the action to be performed, including the action type and any specific parameters (e.g., keyCode, dpadValue, buttonCode) required for the action.
 */
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
