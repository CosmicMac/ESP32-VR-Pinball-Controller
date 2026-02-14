#include <Arduino.h>
#include <SQUIDHID.h>
#include <MPU6050.h>

// ###########################################################################
//  CONFIGURATION
// ###########################################################################

constexpr auto DEVICE_NAME           = "VR Pinball controller"; // BLE device name
constexpr auto DEVICE_MANUFACTURER   = "CosmicMac";             // BLE device manufacturer
constexpr uint8_t BTN_DEBOUNCE_MS    = 10;                      // Debounce delay for buttons in ms
constexpr uint16_t MODE_HOLD_TIME_MS = 2000;                    // Time in ms to hold the Select + Start buttons to switch mode between FX and Classic

// MPU (nudge detection)
constexpr uint8_t ACCEL_RANGE                = MPU6050_ACCEL_FS_2;  // Accelerometer range (±2g)
constexpr uint8_t DLPF_MODE                  = MPU6050_DLPF_BW_188; // Digital low-pass filter configuration (from 5 to 256 Hz, the fastest the noisiest)
constexpr uint8_t MOTION_DETECTION_THRESHOLD = 4;                   // Motion detection threshold in MPU6050 units
constexpr uint32_t COOLDOWN_MS               = 200;                 // Delay between two motion detections
constexpr uint32_t NUDGE_RESET_MS            = 50;                  // Time to reset nudge to center
constexpr uint8_t NUDGE_SAMPLES              = 10;                  // Number of samples to average for nudge detection
constexpr uint8_t MPU6050_ADDR               = 0x68;                // MPU6050 I2C address
constexpr uint8_t ACCEL_XOUT_H               = 0x3B;                // MPU6050 register address for accelerometer X-axis high byte

// GPIO pins
constexpr uint8_t BTN_SELECT_PIN     = 1;  // Button select
constexpr uint8_t BTN_START_PIN      = 2;  // Button start
constexpr uint8_t BTN_LAUNCH_PIN     = 4;  // Button Launch ball
constexpr uint8_t BTN_A_PIN          = 5;  // Button A
constexpr uint8_t BTN_B_PIN          = 6;  // Button B
constexpr uint8_t BTN_X_PIN          = 7;  // Button X
constexpr uint8_t BTN_Y_PIN          = 15; // Button Y
constexpr uint8_t BTN_L1_PIN         = 42; // Button left flipper
constexpr uint8_t BTN_L2_PIN         = 41; // Button left Magna Save
constexpr uint8_t BTN_R1_PIN         = 16; // Button right flipper
constexpr uint8_t BTN_R2_PIN         = 17; // Button right Magna Save
constexpr uint8_t JOYSTICK_DOWN_PIN  = 40; // Joystick down
constexpr uint8_t JOYSTICK_UP_PIN    = 39; // Joystick up
constexpr uint8_t JOYSTICK_RIGHT_PIN = 38; // Joystick right
constexpr uint8_t JOYSTICK_LEFT_PIN  = 37; // Joystick left
constexpr uint8_t I2C_SDA_PIN        = 8;  // MPU6050 I2C SDA
constexpr uint8_t I2C_SCL_PIN        = 9;  // MPU6050 I2C SCL
constexpr uint8_t MPU_INT_PIN        = 10; // MPU6050 interruption
constexpr uint8_t DEFAULT_MODE_PIN   = 21; // High = Pinball FX VR as default mode (keyboard), Low = Pinball VR Classic as default mode (gamepad)

// Mapping between user actions and keyboard keys (Pinball FX VR)
constexpr NKROKey KEY_NONE{0}; // Dummy key
constexpr NKROKey KEY_LAUNCH_BALL   = KC_8;
constexpr NKROKey KEY_LEFT_FLIPPER  = KC_U;
constexpr NKROKey KEY_RIGHT_FLIPPER = KC_6;
constexpr NKROKey KEY_PAUSE         = KC_I;
constexpr NKROKey KEY_NUDGE_UP      = KC_A;
constexpr NKROKey KEY_NUDGE_DOWN    = KC_S;
constexpr NKROKey KEY_NUDGE_RIGHT   = KC_D;
constexpr NKROKey KEY_NUDGE_LEFT    = KC_F;
constexpr NKROKey KEY_MAGNA_SAVE    = KC_8; // idem KEY_LAUNCH_BALL
constexpr NKROKey KEY_POWER_UP      = KC_5;

// Mapping between user actions and gamepad buttons (Pinball VR Classic)
constexpr GamepadButton GAMEPAD_NONE{0};               // Dummy button
constexpr GamepadButton GAMEPAD_BTN_SELECT    = GB_BA; // Back
constexpr GamepadButton GAMEPAD_BTN_START     = GB_ST; // Start
constexpr GamepadButton GAMEPAD_LAUNCH_BALL   = GB_WE; // X
constexpr GamepadButton GAMEPAD_LEFT_FLIPPER  = GB_L1; // Trigger 1 left
constexpr GamepadButton GAMEPAD_RIGHT_FLIPPER = GB_R1; // Trigger 1 right
constexpr GamepadButton GAMEPAD_BTN_A         = GB_SO; // A
constexpr GamepadButton GAMEPAD_BTN_B         = GB_EA; // B
constexpr GamepadButton GAMEPAD_BTN_X         = GB_WE; // X
constexpr GamepadButton GAMEPAD_BTN_Y         = GB_NO; // Y
constexpr GamepadButton GAMEPAD_BTN_UP        = GB_UP; // Up
constexpr GamepadButton GAMEPAD_BTN_DOWN      = GB_DO; // Down
constexpr GamepadButton GAMEPAD_BTN_LEFT      = GB_LE; // Left
constexpr GamepadButton GAMEPAD_BTN_RIGHT     = GB_RI; // Right

// Array of button configurations
constexpr uint8_t NUM_BUTTONS = 15;

struct ButtonInfo
{
    uint8_t pin;                    // GPIO pin number for the button
    GamepadButton button;           // Associated gamepad button mapping
    NKROKey key;                    // Associated keyboard key mapping
    int state;                      // Current debounced state (HIGH or LOW)
    unsigned long lastDebounceTime; // Timestamp of the last debounce event (in ms)
};

//@formatter:off
ButtonInfo buttons[NUM_BUTTONS] = {
//                          Classic                 FX
//   Button pin             Gamepad button          Keyboard key        State   Debounce time
    {BTN_LAUNCH_PIN,        GAMEPAD_LAUNCH_BALL,    KEY_LAUNCH_BALL,    HIGH,   0},
    {BTN_A_PIN,             GAMEPAD_BTN_A,          KEY_LAUNCH_BALL,    HIGH,   0},
    {BTN_B_PIN,             GAMEPAD_BTN_B,          KEY_POWER_UP,       HIGH,   0},
    {BTN_X_PIN,             GAMEPAD_BTN_X,          KEY_PAUSE,          HIGH,   0},
    {BTN_Y_PIN,             GAMEPAD_BTN_Y,          KEY_PAUSE,          HIGH,   0},
    {BTN_SELECT_PIN,        GAMEPAD_BTN_SELECT,     KEY_PAUSE,          HIGH,   0},
    {BTN_START_PIN,         GAMEPAD_BTN_START,      KEY_PAUSE,          HIGH,   0},
    {BTN_L1_PIN,            GAMEPAD_LEFT_FLIPPER,   KEY_LEFT_FLIPPER,   HIGH,   0},
    {BTN_R1_PIN,            GAMEPAD_RIGHT_FLIPPER,  KEY_RIGHT_FLIPPER,  HIGH,   0},
    {BTN_L2_PIN,            GAMEPAD_NONE,           KEY_MAGNA_SAVE,     HIGH,   0},
    {BTN_R2_PIN,            GAMEPAD_NONE,           KEY_MAGNA_SAVE,     HIGH,   0},
    {JOYSTICK_LEFT_PIN,     GAMEPAD_BTN_LEFT,       KEY_NUDGE_LEFT,     HIGH,   0},
    {JOYSTICK_RIGHT_PIN,    GAMEPAD_BTN_RIGHT,      KEY_NUDGE_RIGHT,    HIGH,   0},
    {JOYSTICK_UP_PIN,       GAMEPAD_BTN_UP,         KEY_NUDGE_UP,       HIGH,   0},
    {JOYSTICK_DOWN_PIN,     GAMEPAD_BTN_DOWN,       KEY_NUDGE_DOWN,     HIGH,   0},
};
//@formatter:on

// ###########################################################################


// ENUMS

// Controller mode: Pinball VR Classic (gamepad controller) or Pinball FX VR (keyboard controller)
enum class ControllerMode : uint8_t { UNDEFINED, CLASSIC, FX };

// LED states for RGB indication
enum class LedState : uint8_t { OFF, INITIALIZATION, FX_MODE_ACTIVE, CLASSIC_MODE_ACTIVE };


// GLOBAL VARIABLES

MPU6050 mpu;
SQUIDHID hidDevice(DEVICE_NAME, DEVICE_MANUFACTURER);
volatile bool motionIRQ = false;
auto controllerMode     = ControllerMode::UNDEFINED;


// FUNCTION DECLARATIONS

void IRAM_ATTR onMotionInterrupt();
void setRgbLedState(LedState state);
void calibrateSensor(uint16_t samples = 500);
void setupAccelerometer();
bool readAccelG(int16_t& x, int16_t& y);
void setupHIDDevice();
void handleButton(ButtonInfo& button);

void setup() {
    Serial.begin(115200);

    setRgbLedState(LedState::INITIALIZATION);

    // Initialize button pins
    for (const auto& button : buttons) {
        pinMode(button.pin, INPUT_PULLUP);
    }

    // Initialize accelerometer
    setupAccelerometer();

    // Initialize HID device
    setupHIDDevice();

    // Set default controller mode
    pinMode(DEFAULT_MODE_PIN, INPUT_PULLUP);
    controllerMode = digitalRead(DEFAULT_MODE_PIN) == LOW ? ControllerMode::CLASSIC : ControllerMode::FX;
    if (controllerMode == ControllerMode::CLASSIC) {
        setRgbLedState(LedState::CLASSIC_MODE_ACTIVE);
        Serial.println("Switched to Pinball VR Classic mode (gamepad)");
    }
    else {
        setRgbLedState(LedState::FX_MODE_ACTIVE);
        Serial.println("Switched to Pinball FX VR mode (keyboard)");
    }
}

void loop() {
    const unsigned long currentMillis = millis();

    // HID DEVICE HANDLER

    hidDevice.update();
    if (!hidDevice.isConnected()) {
        Serial.println("Waiting connection...");
        delay(1000);
        return;
    }

    // MPU HANDLER (NUDGE DETECTION)

    static unsigned long lastNudgeMillis = 0;
    static bool isNudging                = false;
    static NKROKey nudgeKey(0);

    if (motionIRQ) {
        motionIRQ = false;

        // Read the status to clear the MPU hardware interrupt and check that it is indeed a motion detection
        if (!(mpu.getIntStatus() & 0x40)) return;

        if (currentMillis - lastNudgeMillis > COOLDOWN_MS) {
            lastNudgeMillis = currentMillis;
            isNudging       = true;

            uint16_t absX    = 0, absY    = 0;
            uint16_t maxAbsX = 0, maxAbsY = 0;
            int16_t maxX     = 0, maxY    = 0;

            // Loop to read accelerometer values 10 times and track the maximum absolute X and Y values
            int16_t x, y;
            for (int i = 0; i < static_cast<int>(NUDGE_SAMPLES); i++) {
                if (!readAccelG(x, y)) continue;

                Serial.printf("X, Y:\t\t%d\t\t%d\n", x, y);

                absX = abs(x);
                if (absX > maxAbsX) {
                    maxAbsX = absX;
                    maxX    = x;
                }

                absY = abs(y);
                if (absY > maxAbsY) {
                    maxAbsY = absY;
                    maxY    = y;
                }
            }

            Serial.printf("Max X/Y:\t%d\t\t%d\n", maxX, maxY);

            if (abs(maxX) > abs(maxY)) {
                if (controllerMode == ControllerMode::FX) nudgeKey = KEY_NUDGE_UP;
                else hidDevice.setLeftStick(0, -32768);
                Serial.println("Nudge up");
            }
            else {
                int16_t nudgeY = maxY;
                if (nudgeY > 0) {
                    if (controllerMode == ControllerMode::FX) nudgeKey = KEY_NUDGE_LEFT;
                    else hidDevice.setLeftStick(-32768, 0);
                    Serial.println("Nudge left");
                }
                else {
                    if (controllerMode == ControllerMode::FX) nudgeKey = KEY_NUDGE_RIGHT;
                    else hidDevice.setLeftStick(32767, 0);
                    Serial.println("Nudge right");
                }
            }
            if (controllerMode == ControllerMode::FX) hidDevice.press(nudgeKey);
        }
    }

    if (isNudging && (currentMillis - lastNudgeMillis > NUDGE_RESET_MS)) {
        isNudging = false;
        if (controllerMode == ControllerMode::FX) hidDevice.release(nudgeKey);
        else hidDevice.setLeftStick(0, 0);
    }

    // CONTROLLER MODE SWITCH HANDLER

    static bool modeSwitchPending = false;

    static unsigned long modeHoldStartMillis  = 0;
    static unsigned long lastModeSwitchMillis = 0;

    if (
        currentMillis - lastModeSwitchMillis > MODE_HOLD_TIME_MS * 3 // Force a delay between 2 mode switches
        && digitalRead(BTN_SELECT_PIN) == LOW
        && digitalRead(BTN_START_PIN) == LOW
    ) {
        if (!modeSwitchPending) {
            modeHoldStartMillis = currentMillis;
            modeSwitchPending   = true;
        }
        else if (currentMillis - modeHoldStartMillis > MODE_HOLD_TIME_MS) {
            // Change mode after long press
            lastModeSwitchMillis = currentMillis;
            if (controllerMode == ControllerMode::FX) {
                controllerMode = ControllerMode::CLASSIC;
                setRgbLedState(LedState::CLASSIC_MODE_ACTIVE);
                Serial.println("Switched to Pinball VR Classic mode (gamepad)");
            }
            else {
                controllerMode = ControllerMode::FX;
                setRgbLedState(LedState::FX_MODE_ACTIVE);
                Serial.println("Switched to Pinball FX VR mode (keyboard)");
            }
            modeSwitchPending = false; // Prevents re-switching while buttons are held down
            delay(100);                // Anti-rebond pour éviter plusieurs changements rapides
        }
        return;
    }
    else {
        modeSwitchPending = false;
    }

    // BUTTONS HANDLER

    for (auto& button : buttons) {
        handleButton(button);
    }
}


/**
 * Interrupt Service Routine triggered by motion detection
 */
void IRAM_ATTR onMotionInterrupt() {
    motionIRQ = true;
}


/**
 * Sets the RGB LED to a specific state based on the provided LedState
 * If RGB_BUILTIN is not defined, the function returns immediately
 *
 * @param state The LedState to set the RGB LED to
 */
void setRgbLedState(const LedState state) {
#ifndef RGB_BUILTIN
    return;
#endif

    switch (state) {
    case LedState::OFF:
        rgbLedWrite(RGB_BUILTIN, 0, 0, 0);
        break;
    case LedState::INITIALIZATION:
        rgbLedWrite(RGB_BUILTIN, 128, 0, 0); // Red
        break;
    case LedState::FX_MODE_ACTIVE:
        rgbLedWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS); // Blue, like in keyBoard
        break;
    case LedState::CLASSIC_MODE_ACTIVE:
        rgbLedWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0); // Green, like in Gamepad
        break;
    }
}


/**
 * Handles the debouncing and state change of a single button
 *
 * @param button Reference to the ButtonInfo struct representing the button to handle
 */
void handleButton(ButtonInfo& button) {
    if (!hidDevice.isConnected()) {
        return;
    }

    if (millis() - button.lastDebounceTime < BTN_DEBOUNCE_MS) {
        return;
    }

    if (const int reading = digitalRead(button.pin); reading != button.state) {
        button.state            = reading;
        button.lastDebounceTime = millis();
        if (button.state == LOW) {
            if (controllerMode == ControllerMode::FX) {
                hidDevice.press(button.key);
            }
            else {
                hidDevice.press(button.button);
            }
            Serial.printf("Button on pin %d pressed\n", button.pin);
        }
        else {
            if (controllerMode == ControllerMode::FX) {
                if (button.key != KEY_NONE) hidDevice.release(button.key);
            }
            else {
                if (button.button != GAMEPAD_NONE) hidDevice.release(button.button);
            }
            Serial.printf("Button on pin %d released\n", button.pin);
        }
    }
}


/**
 * Sets up the BLE gamepad with the desired configuration
 */
void setupHIDDevice() {
    Serial.println("Initializing HID device...");
    hidDevice.setAppearance(GAMEPAD);
    hidDevice.begin();
    while (!hidDevice.isConnected()) {
        Serial.println("Waiting connection...");
        delay(1000);
    }
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
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), onMotionInterrupt, RISING);

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

        sx2 += static_cast<float>(x) * x; // Cast to float to avoid int16 overflow
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

    const float sigmaX = sqrtf(sx2 / validSamples - meanX * meanX);
    const float sigmaY = sqrtf(sy2 / validSamples - meanY * meanY);

    Serial.printf("Calibration done (%d samples)!", validSamples);
    Serial.printf("Mean X/Y:\t\t%f\t\t%f", meanX, meanY);
    Serial.printf("Sigma X/Y:\t\t%f\t\t%f", sigmaX, sigmaY);
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
