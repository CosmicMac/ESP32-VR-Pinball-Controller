#include <Arduino.h>
#include <Preferences.h>
#include <MPU6050.h>
#include "BleHidController.h"
#include "ESP32_VR_Pinball_Controller.h"
#include "config.h"

// #define DEBUG_ANALOG_NUDGE_RAW
// #define DEBUG_ANALOG_NUDGE_VELOCITY

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
* Handles the debouncing and state change of a single button
*
* @param button Reference to the ButtonInfo struct representing the button to handle
* @param currentMillis Current time in milliseconds, used for debouncing logic
*/
void handleButton(ButtonInfo& button) {
    const auto currentMillis = millis();
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
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        return false;
    }

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

/**
 * Reads multiple accelerometer samples and returns the peak X and Y values
 *
 * @return The peak X and Y values
 */
AccelPeak getAccelPeak() {
    int16_t x, y, maxX = 0, maxY    = 0;
    uint16_t maxAbsX   = 0, maxAbsY = 0;
    for (int i = 0; i < static_cast<int>(NUDGE_SAMPLES); i++) {
        if (!readAccelRaw(x, y)) continue;
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

void handleAnalogNudgeVelocity() {
    // --- Configuration ---
    constexpr float SAMPLE_RATE_HZ = 400.0f; // Accelerometer polling rate (Hz)
    constexpr float DT             = 1.0f / SAMPLE_RATE_HZ;

    // Noise low-pass filter weight (0 = no update, 1 = no filter)
    constexpr float NOISE_ALPHA = 0.3f;

    // Friction filter: half-life of ~2 seconds
    // factor = pow(0.5, 1 / (SAMPLE_RATE_HZ * halfLifeSeconds))
    constexpr float FRICTION_HALF_LIFE_S = 2.0f;

    // Dead zone to avoid sending residual noise to the host
    constexpr float VELOCITY_DEAD_ZONE = 10.0f;

    // Max velocity value for axis scaling
    constexpr float MAX_VELOCITY = 200.0f;

    // Interval between USB HID reports (10ms = 100 Hz report rate)
    constexpr unsigned long REPORT_INTERVAL_MS = 10UL;

    // --- Static state ---
    static unsigned long lastSampleMicros = 0;
    static unsigned long lastReportMillis = 0;

    static float dcX   = 0.0f, dcY   = 0.0f; // DC bias estimate
    static float filtX = 0.0f, filtY = 0.0f; // Noise-filtered acceleration
    static float velX  = 0.0f, velY  = 0.0f; // Integrated velocity

    // --- Sample the accelerometer at SAMPLE_RATE_HZ ---
    const unsigned long nowMicros        = micros();
    const unsigned long sampleIntervalUs = static_cast<unsigned long>(DT * 1e6f);

    if (nowMicros - lastSampleMicros >= sampleIntervalUs) {
        // Fix: ignore first sample to avoid spike from uninitialized timer
        if (lastSampleMicros == 0) {
            lastSampleMicros = nowMicros;
            return;
        }

        const float realDt = static_cast<float>(nowMicros - lastSampleMicros) * 1e-6f;
        lastSampleMicros   = nowMicros;

        int16_t rawX, rawY;
        if (!readAccelRaw(rawX, rawY)) return;

        const float ax = static_cast<float>(rawX);
        const float ay = static_cast<float>(rawY);

        // 1. DC removal: sliding exponential average of the bias
        dcX             += (realDt / 0.300f) * (ax - dcX);
        dcY             += (realDt / 0.300f) * (ay - dcY);
        const float acX = ax - dcX;
        const float acY = ay - dcY;

        // 2. Noise low-pass filter
        filtX = NOISE_ALPHA * acX + (1.0f - NOISE_ALPHA) * filtX;
        filtY = NOISE_ALPHA * acY + (1.0f - NOISE_ALPHA) * filtY;

        // 3. Integrate acceleration → velocity  (v += a * realDt)
        velX += filtX * realDt;
        velY += filtY * realDt;

        // 4. Friction filter: attenuate to prevent drift accumulation
        const float friction = powf(0.5f, realDt / FRICTION_HALF_LIFE_S);
        velX                 *= friction;
        velY                 *= friction;

#ifdef DEBUG_ANALOG_NUDGE_VELOCITY
        static unsigned long lastPrint = 0;

        static uint16_t maxRawX = 0, maxRawY = 0;
        if (abs(rawX) > maxRawX) maxRawX = abs(rawX);
        if (abs(rawY) > maxRawY) maxRawY = abs(rawY);

        static float maxVelX = 0.0f, maxVelY = 0.0f;
        if (abs(velX) > abs(maxVelX)) maxVelX = velX;
        if (abs(velY) > abs(maxVelY)) maxVelY = velY;

        const auto currentMillis = millis();
        if (currentMillis - lastPrint > 1000) {
            Serial.printf("maxRawX: %4d\tmaxRawY: %4d\tmaxVelX: %7.1f\tmaxVelY: %7.1f\n", maxRawX, maxRawY, maxVelX, maxVelY);
            lastPrint = currentMillis;
        }
#endif
    }

    // --- Send HID report at report rate ---
    const auto currentMillis = millis();
    if (currentMillis - lastReportMillis < REPORT_INTERVAL_MS) return;
    lastReportMillis = currentMillis;

    // Apply velocity dead zone
    const float outX = (fabsf(velX) < VELOCITY_DEAD_ZONE) ? 0.0f : velX;
    const float outY = (fabsf(velY) < VELOCITY_DEAD_ZONE) ? 0.0f : velY;

    // Scale to int16 joystick range [-32767, 32767]
    const auto stickX = static_cast<int16_t>(std::clamp(
        map(static_cast<long>(outX), static_cast<long>(-MAX_VELOCITY), static_cast<long>(MAX_VELOCITY), -32767L, 32767L),
        -32767L, 32767L));
    const auto stickY = static_cast<int16_t>(std::clamp(
        map(static_cast<long>(outY), static_cast<long>(-MAX_VELOCITY), static_cast<long>(MAX_VELOCITY), -32767L, 32767L),
        -32767L, 32767L));

    // Report on RX/RY axes (velocity), X/Y axes remain free for raw acceleration
    // hid.setLeftStick(stickX, stickY);
    hid.setRightStick(-stickX, stickY);
}

void handleAnalogNudgeRaw() {
    constexpr float SAMPLE_RATE_HZ             = 400.0f;
    constexpr unsigned long SAMPLE_INTERVAL_US = static_cast<unsigned long>(1e6f / SAMPLE_RATE_HZ); // 2500 µs
    constexpr unsigned long REPORT_INTERVAL_MS = 10UL;                                              // 100 Hz report rate
    constexpr float ACCEL_ALPHA                = 0.4f;
    constexpr float ACCEL_DEAD_ZONE            = 200.0f;
    constexpr int16_t ACCEL_MAX_VALUE          = 30000;

    static unsigned long lastSampleMicros = 0;
    static unsigned long lastReportMillis = 0;
    static float xFiltered                = 0.0f, yFiltered = 0.0f;

    // --- Sample at SAMPLE_RATE_HZ ---
    const unsigned long nowMicros = micros();
    if (nowMicros - lastSampleMicros >= SAMPLE_INTERVAL_US) {
        if (lastSampleMicros == 0) {
            lastSampleMicros = nowMicros;
            return;
        }
        lastSampleMicros = nowMicros;

        int16_t x = 0, y = 0;
        if (!readAccelRaw(x, y)) return;

#ifdef DEBUG_ANALOG_NUDGE_RAW
        static unsigned long lastPrint = 0;
        static unsigned long lastReset = 0;
        static int16_t maxX            = 0, maxY = 0;

        if (abs(x) > abs(maxX)) maxX = x;
        if (abs(y) > abs(maxY)) maxY = y;
        const auto currentMillis = millis();
        if (currentMillis - lastPrint > 1000) {
            Serial.printf("RAW maxX: %d\tmaxY: %d\n", maxX, maxY);
            lastPrint = currentMillis;
        }
        if (currentMillis - lastReset > 10000) {
            maxX      = 0;
            maxY      = 0;
            lastReset = currentMillis;
        }
#endif

        // Low-pass filter applied at high sample rate for better noise rejection
        xFiltered = ACCEL_ALPHA * static_cast<float>(x) + (1.0f - ACCEL_ALPHA) * xFiltered;
        yFiltered = ACCEL_ALPHA * static_cast<float>(y) + (1.0f - ACCEL_ALPHA) * yFiltered;
    }

    // --- Report at REPORT_INTERVAL_MS ---
    const auto currentMillis = millis();
    if (currentMillis - lastReportMillis < REPORT_INTERVAL_MS) return;
    lastReportMillis = currentMillis;

    int16_t stickX = 0, stickY = 0;
    if (fabsf(xFiltered) > ACCEL_DEAD_ZONE || fabsf(yFiltered) > ACCEL_DEAD_ZONE) {
        stickX = static_cast<int16_t>(std::clamp<long>(
            map(static_cast<long>(xFiltered), -ACCEL_MAX_VALUE, ACCEL_MAX_VALUE, -32767L, 32767L),
            -32767L, 32767L));
        stickY = static_cast<int16_t>(std::clamp<long>(
            map(static_cast<long>(yFiltered), -ACCEL_MAX_VALUE, ACCEL_MAX_VALUE, -32767L, 32767L),
            -32767L, 32767L));
    }

    hid.setLeftStick(stickX, stickY);
}
#endif


/**
 * Handles processing of analog nudge input from an accelerometer, including filtering,
 * bias correction, and velocity computation, to drive virtual joystick controls.
 *
 * The method samples accelerometer input at a fixed rate, applies several signal
 * processing stages (DC removal, low-pass filtering, integration, and friction-based
 * velocity attenuation), and updates virtual joystick values at a specified reporting
 * interval. The results are sent as left and right stick values, corresponding to
 * acceleration and velocity respectively.
 *
 * - DC removal offsets biases using an exponential sliding average.
 * - Noise is reduced with a low-pass filter.
 * - Velocity is computed by integrating acceleration.
 * - Friction prevents velocity drift accumulation.
 *
 * Analog dead zones are applied to both acceleration and velocity to avoid noise
 * contributing to stick outputs.
 */
void handleAnalogNudge() {
    // --- Configuration ---
    constexpr float SAMPLE_RATE_HZ             = 400.0f;
    constexpr auto SAMPLE_INTERVAL_US          = static_cast<unsigned long>(1000000.0f / SAMPLE_RATE_HZ);
    constexpr unsigned long REPORT_INTERVAL_MS = 10UL;
    constexpr float NOISE_ALPHA                = 0.3f; //!HERE

    // Velocity pipeline
    constexpr float FRICTION_HALF_LIFE_S = 2.0f;
    constexpr float VELOCITY_DEAD_ZONE   = 10.0f;
    constexpr float MAX_VELOCITY         = 200.0f;

    // Acceleration pipeline
    constexpr float ACCELERATION_DEAD_ZONE = 200.0f;
    constexpr int16_t MAX_ACCELERATION     = 30000;

    // Static state
    static unsigned long lastSampleMicros = 0;
    static unsigned long lastReportMillis = 0;
    static float dcX                      = 0.0f, dcY   = 0.0f;
    static float filtX                    = 0.0f, filtY = 0.0f;
    static float velX                     = 0.0f, velY  = 0.0f;

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

        int16_t rawX, rawY;
        if (!readAccelRaw(rawX, rawY)) {
            filtX = 0.0f;
            filtY = 0.0f;
            velX  = 0.0f;
            velY  = 0.0f;
            hid.setLeftStick(0, 0);
            hid.setRightStick(0, 0);
            return;
        }

        const float ax = static_cast<float>(rawX);
        const float ay = static_cast<float>(rawY);

        // 1. DC removal: sliding exponential average of the bias
        const float dcAlpha = std::clamp(realDt / 0.300f, 0.0f, 1.0f);
        dcX                 += dcAlpha * (ax - dcX);
        dcY                 += dcAlpha * (ay - dcY);
        const float acX     = ax - dcX;
        const float acY     = ay - dcY;

        // 2. Noise low-pass filter
        filtX = NOISE_ALPHA * acX + (1.0f - NOISE_ALPHA) * filtX;
        filtY = NOISE_ALPHA * acY + (1.0f - NOISE_ALPHA) * filtY;

        // 3. Integrate acceleration → velocity  (v += a * realDt)
        velX += filtX * realDt;
        velY += filtY * realDt;

        // 4. Friction filter: attenuate to prevent drift accumulation
        const float friction = powf(0.5f, realDt / FRICTION_HALF_LIFE_S);
        velX                 *= friction;
        velY                 *= friction;
    }

    // TODO Debug output

    // --- REPORT AT REPORT_INTERVAL_MS ---
    const auto currentMillis = millis();
    if (currentMillis - lastReportMillis < REPORT_INTERVAL_MS) return;
    lastReportMillis = currentMillis;

    // Left stick: acceleration
    const float outRawX = (fabsf(filtX) > ACCELERATION_DEAD_ZONE) ? filtX : 0.0f;
    const float outRawY = (fabsf(filtY) > ACCELERATION_DEAD_ZONE) ? filtY : 0.0f;

    const float scaledLeftX = outRawX * 32767.0f / static_cast<float>(MAX_ACCELERATION);
    const float scaledLeftY = outRawY * 32767.0f / static_cast<float>(MAX_ACCELERATION);

    const int16_t leftX = static_cast<int16_t>(std::clamp(scaledLeftX, -32767.0f, 32767.0f));
    const int16_t leftY = static_cast<int16_t>(std::clamp(scaledLeftY, -32767.0f, 32767.0f));

    hid.setLeftStick(leftX, leftY);

    // Right stick: velocity
    const float outX = (fabsf(velX) < VELOCITY_DEAD_ZONE) ? 0.0f : velX;
    const float outY = (fabsf(velY) < VELOCITY_DEAD_ZONE) ? 0.0f : velY;

    const float scaledRightX = outX * 32767.0f / MAX_VELOCITY;
    const float scaledRightY = outY * 32767.0f / MAX_VELOCITY;

    const int16_t rightX = static_cast<int16_t>(std::clamp(scaledRightX, -32767.0f, 32767.0f));
    const int16_t rightY = static_cast<int16_t>(std::clamp(scaledRightY, -32767.0f, 32767.0f));

    hid.setRightStick(rightX, rightY);
}


/**
 * Handles digital nudge input for FX mode by detecting peaks in accelerometer data and sending corresponding key presses
 */
void handleDigitalNudge() {
    constexpr uint16_t SAMPLE_INTERVAL_MS = 10;      // Sampling interval for accelerometer readings (ms)
    constexpr float ALPHA                 = 0.4f;    // Low-pass filter weight
    constexpr float NUDGE_THRESHOLD       = 3000.0f; // Trigger threshold (tune to taste)
    constexpr float RELEASE_THRESHOLD     = 1500.0f; // Release threshold (hysteresis)

    const auto currentMillis = millis();

    static unsigned long lastSampleMillis = 0;

    static float filtX = 0.0f, filtY = 0.0f;

    // Throttle sampling rate
    if (currentMillis - lastSampleMillis < SAMPLE_INTERVAL_MS) return;
    lastSampleMillis = currentMillis;

    int16_t rawX, rawY;
    if (!readAccelRaw(rawX, rawY))  return;


    // Low-pass filter
    filtX = ALPHA * static_cast<float>(rawX) + (1.0f - ALPHA) * filtX;
    filtY = ALPHA * static_cast<float>(rawY) + (1.0f - ALPHA) * filtY;

    const float absX          = fabsf(filtX);
    const float absY          = fabsf(filtY);
    const bool aboveThreshold = (absX > NUDGE_THRESHOLD || absY > NUDGE_THRESHOLD);

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
        absX < RELEASE_THRESHOLD && absY < RELEASE_THRESHOLD &&
        (currentMillis - nudgeState.lastNudgeMillis > NUDGE_RESET_MS)
    ) {
        nudgeState.isNudging = false;
        if (nudgeState.nudgeKey != 0) {
            hid.keyRelease(nudgeState.nudgeKey);
            nudgeState.nudgeKey = 0;
        }
        else {
            hid.setLeftStick(0, 0);
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
