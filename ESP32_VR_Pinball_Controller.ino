#include <Arduino.h>
#include <Preferences.h>
#include "BleHidController.h"
#include "AccelerometerManager.h"
#include "PlungerHandler.h"
#include "ESP32_VR_Pinball_Controller.h"
#include "config.h"

bool DEBUG_MODE = false; // Press button Y on start to activate debug mode

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

AccelerometerManager accelManager;
PlungerHandler plungerHandler;

// ISR handlers
volatile bool changeModeIRQ = false;
static void IRAM_ATTR onChangeModeISR() { changeModeIRQ = true; }


void setup() {
    Serial.begin(115200);

    setLedColor(LedColor::RED);

    // Initialize buttons
    for (const auto& button : buttons) {
        pinMode(button.pin, INPUT_PULLUP);
    }

    if (digitalRead(BTN_Y_PIN) == LOW) {
        DEBUG_MODE = true;
        blink();
        Serial.println("[setup] Debug mode activated!");
    }

    // Initialize change mode button
    pinMode(CHANGE_MODE_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CHANGE_MODE_PIN), onChangeModeISR, RISING);

    // Initialize HID
    hid.begin(DEVICE_NAME, DEVICE_MANUFACTURER);
    if (digitalRead(BTN_X_PIN) == LOW) {
        BleHidController::deleteAllBonds();
        blink();
        Serial.println("[setup] All BLE bonds deleted!");
    }

    // Initialize accelerometer
    setupAccelerometer();

    // Initialize plunger
    plungerHandler.setup();

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

    plungerHandler.handle(mode, hid, DEBUG_MODE);

    sendGamepadReport();

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
        hid.sendGamepad(static_cast<uint16_t>(GamepadButton::NONE), static_cast<uint8_t>(DpadDirection::CENTERED), 0, 0, 0, 0);
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
 * Blink the built-in RGB LED a specified number of times with the given color
 *
 * If RGB support is not available (RGB_BUILTIN not defined) the function is a no-op
 * to allow safe calls from code that may run on hardware without an RGB LED
 *
 * @param color The LedColor value to show during the blink on-phase
 * @param times Number of on/off cycles to perform
 */
void blink(const LedColor color, const uint8_t times) {
#ifdef RGB_BUILTIN
    for (uint8_t i = 0; i < times; ++i) {
        setLedColor(color);
        delay(200);
        setLedColor(LedColor::OFF);
        delay(200);
    }
#else
    (void)color;
    (void)times;
#endif
}

/**
 * Setup and initialize the accelerometer using the AccelerometerManager
 *
 * @note Ensure that the sensor remains stationary during calibration for accurate offset computation.
 */
void setupAccelerometer() {
    accelManager.begin();
}


/**
 * Reads raw acceleration values from the sensor using AccelerometerManager
 *
 * @param xr Reference to store the calibrated and adjusted X-axis acceleration
 * @param yr Reference to store the calibrated and adjusted Y-axis acceleration
 * @return True if the acceleration values are successfully read and adjusted; false otherwise
 */
bool readAccelRaw(int16_t& xr, int16_t& yr) {
    return accelManager.readRaw(xr, yr);
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

    if (readAccelRaw(rx, ry)) {
        nudgeX.process(rx, now);
        nudgeY.process(ry, now);
    }

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
    // hid.sendGamepadState();

    if (DEBUG_MODE) {
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
    }
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

    if (DEBUG_MODE) {
        Serial.printf("[DEBUG] peakX=%.1f, peakY=%.1f, threshold=%d, isNudging=%d\n", absPeakX, absPeakY, DIGITAL_NUDGE_THRESHOLD, nudgeState.isNudging);
    }

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

/**
 * Sends the current gamepad HID report at a fixed rate.
 * Centralizes all gamepad state flushing to avoid redundant or conflicting sends.
 */
void sendGamepadReport() {
    const uint32_t now               = micros();
    static uint32_t lastReportMicros = 0;
    if (now - lastReportMicros < GAMEPAD_REPORT_INTERVAL_US) return;
    lastReportMicros = now;

    hid.sendGamepadState();
}


