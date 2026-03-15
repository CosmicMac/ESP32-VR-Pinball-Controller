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

bool configChanged             = false; // Flag to indicate if the configuration has changed and needs to be saved
unsigned long lastConfigChange = 0;     // Timestamp of the last configuration change, used to throttle flash writes

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
        performButtonAction(getButtonAction(button, mode), button.state == LOW);
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
    mpu.setFullScaleAccelRange(ACCEL_RANGE);
    mpu.setDLPFMode(DLPF_MODE);

    Serial.println("Calibrating... Keep the MPU6050 sensor still.");
    mpu.CalibrateAccel();
}


/**
 * Reads raw accelerometer values (X and Y axes) from the MPU6050 sensor
 *
 * @param[out] x Reference to store the X-axis accelerometer value
 * @param[out] y Reference to store the Y-axis accelerometer value
 * @return true if the read was successful and 4 bytes were received, false otherwise
 */
bool readAccelRaw(int16_t& x, int16_t& y) {
    /*
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        return false;
    }
    */

    int16_t rawX, rawY, rawZ;
    mpu.getAcceleration(&rawX, &rawY, &rawZ);

    static_assert(SENSOR_ROTATION == 0 ||
                  SENSOR_ROTATION == 90 ||
                  SENSOR_ROTATION == 180 ||
                  SENSOR_ROTATION == 270,
                  "SENSOR_ROTATION must be 0, 90, 180 or 270");

    //@formatter:off
    if constexpr      (SENSOR_ROTATION ==  90) { x = rawY;  y = -rawX; }
    else if constexpr (SENSOR_ROTATION == 180) { x = -rawX; y = -rawY; }
    else if constexpr (SENSOR_ROTATION == 270) { x = -rawY; y = rawX;  }
    else                                       { x = rawX;  y = rawY;  }
    //@formatter:on

    return true;
}

#if 0
void handleAnalogNudge() {
    // --- Configuration ---
    constexpr float SAMPLE_RATE_HZ             = 400.0f;
    constexpr auto SAMPLE_INTERVAL_US          = static_cast<unsigned long>(1000000.0f / SAMPLE_RATE_HZ);
    constexpr unsigned long REPORT_INTERVAL_MS = 50UL;
    constexpr float NOISE_ALPHA                = 0.4f; //!HERE

    // Velocity pipeline
    constexpr float FRICTION_HALF_LIFE_S = 2.0f,
                    VELOCITY_DEAD_ZONE   = 5.0f,
                    MAX_VELOCITY         = 200.0f; // !HERE

    // Acceleration pipeline
    constexpr float ACCELERATION_DEAD_ZONE = 200.0f;
    constexpr int16_t MAX_ACCELERATION     = 20000; // !HERE

    // Static state
    static unsigned long lastSampleMicros = 0,
                         lastReportMillis = 0;

    static float dcX   = 0.0f, dcY   = 0.0f,
                 filtX = 0.0f, filtY = 0.0f,
                 velX  = 0.0f, velY  = 0.0f;

    int16_t rawX = 0, rawY = 0;

    // --- SAMPLE AT SAMPLE_RATE_HZ ---
    const unsigned long nowMicros = micros();
    if (nowMicros - lastSampleMicros >= SAMPLE_INTERVAL_US) {
        // Ignore first sample to avoid spike from uninitialized timer
        if (lastSampleMicros == 0) {
            lastSampleMicros = nowMicros;
            return;
        }

        const float realDt = static_cast<float>(nowMicros - lastSampleMicros) * 1e-6f;
        lastSampleMicros   = nowMicros;

        if (!readAccelRaw(rawX, rawY)) {
            dcX   = 0.0f;
            dcY   = 0.0f;
            filtX = 0.0f;
            filtY = 0.0f;
            velX  = 0.0f;
            velY  = 0.0f;
            hid.setLeftStick(0, 0);
            hid.setRightStick(0, 0);
            return;
        }

        const float ax = static_cast<float>(rawX),
                    ay = static_cast<float>(rawY);

        // 1. DC removal: sliding exponential average of the bias
        const float dcAlpha = std::clamp(realDt / 0.300f, 0.0f, 1.0f);

        dcX += dcAlpha * (ax - dcX);
        dcY += dcAlpha * (ay - dcY);

        const float acX = ax - dcX,
                    acY = ay - dcY;

        // 2. Noise low-pass filter
        filtX = NOISE_ALPHA * acX + (1.0f - NOISE_ALPHA) * filtX;
        filtY = NOISE_ALPHA * acY + (1.0f - NOISE_ALPHA) * filtY;

        // 3. Integrate acceleration → velocity  (v += a * realDt)
        velX += filtX * realDt;
        velY += filtY * realDt;

        // 4. Friction filter: attenuate to prevent drift accumulation
        const float friction = powf(0.5f, realDt / FRICTION_HALF_LIFE_S);

        velX *= friction;
        velY *= friction;
    }

    // --- REPORT AT REPORT_INTERVAL_MS ---
    const auto currentMillis = millis();
    if (currentMillis - lastReportMillis < REPORT_INTERVAL_MS) return;
    lastReportMillis = currentMillis;

    // Left stick: acceleration
    const float outAccX = (fabsf(filtX) > ACCELERATION_DEAD_ZONE) ? filtX : 0.0f,
                outAccY = (fabsf(filtY) > ACCELERATION_DEAD_ZONE) ? filtY : 0.0f;

    const float scaledAccX = outAccX * 32767.0f / static_cast<float>(MAX_ACCELERATION),
                scaledAccY = outAccY * 32767.0f / static_cast<float>(MAX_ACCELERATION);

    const int16_t leftX = static_cast<int16_t>(std::clamp(scaledAccX, -32767.0f, 32767.0f)),
                  leftY = static_cast<int16_t>(std::clamp(scaledAccY, -32767.0f, 32767.0f));

    hid.setLeftStick(leftX, leftY, false);

    // Right stick: velocity
    const float outVelX = (fabsf(velX) < VELOCITY_DEAD_ZONE) ? 0.0f : velX,
                outVelY = (fabsf(velY) < VELOCITY_DEAD_ZONE) ? 0.0f : velY;

    const float scaledVelX = outVelX * 32767.0f / MAX_VELOCITY,
                scaledVelY = outVelY * 32767.0f / MAX_VELOCITY;

    const int16_t rightX = static_cast<int16_t>(std::clamp(scaledVelX, -32767.0f, 32767.0f)),
                  rightY = static_cast<int16_t>(std::clamp(scaledVelY, -32767.0f, 32767.0f));

    hid.setRightStick(rightX, rightY, false);

    hid.sendGamepadState();

#ifdef DEBUG_ANALOG_NUDGE
static unsigned long lastPrint = 0,
                     lastReset = 0;

static float maxFiltX = 0.0f, maxFiltY = 0.0f,
             maxVelX  = 0.0f, maxVelY  = 0.0f;

static int16_t maxRawX   = 0, maxRawY   = 0,
               maxLeftX  = 0, maxLeftY  = 0,
               maxRightX = 0, maxRightY = 0;

    if (abs(rawX)> abs (maxRawX)) maxRawX= rawX;
    if (abs(rawY)> abs (maxRawY)) maxRawY= rawY;

    if (fabsf(filtX)> fabsf (maxFiltX)) maxFiltX= filtX;
    if (fabsf(filtY)> fabsf (maxFiltY)) maxFiltY= filtY;

    if (abs(leftX)> abs (maxLeftX)) maxLeftX= leftX;
    if (abs(leftY)> abs (maxLeftY)) maxLeftY= leftY;

    if (fabsf(velX)> fabsf (maxVelX)) maxVelX= velX;
    if (fabsf(velY)> fabsf (maxVelY)) maxVelY= velY;

    if (abs(rightX)> abs (maxRightX)) maxRightX= rightX;
    if (abs(rightY)> abs (maxRightY)) maxRightY= rightY;

    if (currentMillis- lastPrint> 1000) {
        Serial.printf(
            "maxRaw[%6d, %6d] *** maxAcc[%7.1f, %7.1f] / maxVel[%7.1f, %7.1f] *** maxLeft[%6d, %6d] / maxRight[%6d, %6d]\n",
            maxRawX, maxRawY, maxFiltX, maxFiltY, maxVelX, maxVelY, maxLeftX, maxLeftY, maxRightX, maxRightY
        );

        lastPrint = currentMillis;

        if (currentMillis - lastReset > 5000) {
            Serial.println("\nResetting values...");
            //@formatter:off
            maxRawX   = 0;      maxRawY   = 0;
            maxFiltX  = 0.0f;   maxFiltY  = 0.0f;
            maxLeftX  = 0;      maxLeftY  = 0;
            maxVelX   = 0.0f;   maxVelY   = 0.0f;
            maxRightX = 0;      maxRightY = 0;
            //@formatter:on
            lastReset = currentMillis;
        }
    }

#endif
}
#endif


/**
 * Handles analog nudge input for Classic and VPX by tracking peak accelerometer values and sending them as left stick input
 *
 * Configurable dead zone and maximum acceleration for scaling.
 * Optional debug output for monitoring peak values when DEBUG_ANALOG_NUDGE is defined.
 */
void handleAnalogNudge() {
    static int16_t peakX = 0, peakY = 0;

    // Sample
    int16_t rawX = 0, rawY = 0;

    if (!readAccelRaw(rawX, rawY)) return;

    if (std::abs(static_cast<int32_t>(rawX)) > std::abs(static_cast<int32_t>(peakX))) peakX = rawX;
    if (std::abs(static_cast<int32_t>(rawY)) > std::abs(static_cast<int32_t>(peakY))) peakY = rawY;

    // Report
    static unsigned long lastReportMillis = 0;
    const unsigned long currentMillis     = millis();
    if (currentMillis - lastReportMillis >= ANALOG_NUDGE_REPORT_INTERVAL_MS) {
        lastReportMillis = currentMillis;

        int16_t leftX = static_cast<int16_t>(std::clamp(static_cast<int64_t>(peakX) * 32767 / MAX_ACCELERATION, -32767LL, 32767LL)),
                leftY = static_cast<int16_t>(std::clamp(static_cast<int64_t>(peakY) * 32767 / MAX_ACCELERATION, -32767LL, 32767LL));

        // Dead zone
        if (std::abs(leftX) < ANALOG_NUDGE_DEAD_ZONE) leftX = 0;
        if (std::abs(leftY) < ANALOG_NUDGE_DEAD_ZONE) leftY = 0;

        hid.setLeftStick(leftX, leftY);

#ifdef DEBUG_ANALOG_NUDGE
        static unsigned long lastPrint = 0,
                             lastReset = 0;

        static int16_t maxPeakX = 0, maxPeakY = 0,
                       maxLeftX = 0, maxLeftY = 0;

        if (std::abs(static_cast<int32_t>(peakX)) > std::abs(static_cast<int32_t>(maxPeakX))) maxPeakX = peakX;
        if (std::abs(static_cast<int32_t>(peakY)) > std::abs(static_cast<int32_t>(maxPeakY))) maxPeakY = peakY;

        if (std::abs(static_cast<int32_t>(leftX)) > std::abs(static_cast<int32_t>(maxLeftX))) maxLeftX = leftX;
        if (std::abs(static_cast<int32_t>(leftY)) > std::abs(static_cast<int32_t>(maxLeftY))) maxLeftY = leftY;

        if (currentMillis - lastPrint > 1000) {
            Serial.printf(
                "maxPeak[%6d, %6d] *** maxLeft[%6d, %6d]\n",
                maxPeakX, maxPeakY, maxLeftX, maxLeftY
            );

            lastPrint = currentMillis;

            if (currentMillis - lastReset > 5000) {
                Serial.println("\nResetting values...");
                maxPeakX  = 0;
                maxPeakY  = 0;
                maxLeftX  = 0;
                maxLeftY  = 0;
                lastReset = currentMillis;
            }
        }

#endif

        peakX = 0;
        peakY = 0;
    }
}


/**
 * Handles digital nudge input for FX by detecting when filtered accelerometer
 * values exceed a threshold and sending corresponding key presses
 */
void handleDigitalNudge() {

    const auto currentMillis = millis();

    static unsigned long lastSampleMillis = 0;

    static float filtX = 0.0f, filtY = 0.0f;

    // Throttle sampling rate
    if (currentMillis - lastSampleMillis < DIGITAL_NUDGE_SAMPLE_INTERVAL_MS) return;
    lastSampleMillis = currentMillis;

    int16_t rawX, rawY;
    if (!readAccelRaw(rawX, rawY)) return;

    // Low-pass filter
    filtX = DIGITAL_NUDGE_ALPHA * static_cast<float>(rawX) + (1.0f - DIGITAL_NUDGE_ALPHA) * filtX;
    filtY = DIGITAL_NUDGE_ALPHA * static_cast<float>(rawY) + (1.0f - DIGITAL_NUDGE_ALPHA) * filtY;

    const float absX          = fabsf(filtX);
    const float absY          = fabsf(filtY);
    const bool aboveThreshold = (absX > DIGITAL_NUDGE_THRESHOLD || absY > DIGITAL_NUDGE_THRESHOLD);

    // Cooldown guard
    if (
        aboveThreshold && !nudgeState.isNudging &&
        (currentMillis - nudgeState.lastNudgeMillis > COOLDOWN_MS)
    ) {
        nudgeState.lastNudgeMillis = currentMillis;
        nudgeState.isNudging       = true;
        nudgeState.nudgeKey        = 0;

        // Determine nudge direction (Y axis = forward, X axis = left/right)
        if (filtY > 0 && filtY > absX) {
            nudgeState.nudgeKey = static_cast<uint8_t>(FxNudgeKey::FORWARD);
        }
        else {
            nudgeState.nudgeKey = static_cast<uint8_t>(filtX < 0 ? FxNudgeKey::LEFT : FxNudgeKey::RIGHT);
        }

        if (nudgeState.nudgeKey != 0) hid.keyPress(nudgeState.nudgeKey);
    }

    // Release nudge when signal drops below release threshold (hysteresis)
    else if (
        nudgeState.isNudging &&
        !aboveThreshold &&
        absX < DIGITAL_NUDGE_RELEASE_THRESHOLD && absY < DIGITAL_NUDGE_RELEASE_THRESHOLD &&
        (currentMillis - nudgeState.lastNudgeMillis > NUDGE_RESET_MS)
    ) {
        nudgeState.isNudging = false;
        if (nudgeState.nudgeKey != 0) {
            hid.keyRelease(nudgeState.nudgeKey);
            nudgeState.nudgeKey = 0;
        }
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
