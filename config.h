#pragma once
#include "BleHidController.h"

// #########################################################
// GPIO pins for buttons
// Make sure to connect one leg of each button to GND
// and the other leg to the GPIO pin defined here
// #########################################################

// Buttons: connect one leg of each button to GND and the other leg to the GPIO pin
constexpr uint8_t BTN_SELECT_PIN          = 1;  // "Select" button
constexpr uint8_t BTN_START_PIN           = 2;  // "Start" button
constexpr uint8_t BTN_LAUNCH_PIN          = 4;  // Launch button (plunger)
constexpr uint8_t BTN_A_PIN               = 5;  // "Action A" button (bottom button in the diamond layout)
constexpr uint8_t BTN_B_PIN               = 6;  // "Action B" button (right button in the diamond layout)
constexpr uint8_t BTN_X_PIN               = 7;  // "Action X" button (left button in the diamond layout)
constexpr uint8_t BTN_Y_PIN               = 15; // "Action Y" button (top button in the diamond layout)
constexpr uint8_t BTN_LEFT_FLIPPER_PIN    = 42; // Left flipper button
constexpr uint8_t BTN_LEFT_MAGNASAVE_PIN  = 41; // Left magnasave button
constexpr uint8_t BTN_RIGHT_FLIPPER_PIN   = 16; // Right flipper button
constexpr uint8_t BTN_RIGHT_MAGNASAVE_PIN = 17; // Right magnasave button
constexpr uint8_t DPAD_DOWN_PIN           = 40; // D-Pad down button
constexpr uint8_t DPAD_UP_PIN             = 39; // D-Pad up button
constexpr uint8_t DPAD_RIGHT_PIN          = 38; // D-Pad right button
constexpr uint8_t DPAD_LEFT_PIN           = 37; // D-Pad left button
constexpr uint8_t CHANGE_MODE_PIN         = 21; // Momentary button to cycle through controller modes (FX, Classic, VPX)

// Accelerometer
constexpr uint8_t I2C_SDA_PIN = 8; // I2C SDA pin
constexpr uint8_t I2C_SCL_PIN = 9; // I2C SCL pin


// #########################################################
// Accelerometer parameters
// #########################################################
constexpr uint8_t ACCEL_SENSOR_ADDR      = 0x68;  // I2C address
constexpr uint16_t ACCEL_SENSOR_ROTATION = 90;    // Rotation of the sensor on the horizontal plane (0, 90, 180, 270 degrees counter clockwise)
constexpr bool ACCEL_SENSOR_UPSIDEDOWN_X = false; // Sensor flipped around X axis, after rotation (Y and Z invert)
constexpr bool ACCEL_SENSOR_UPSIDEDOWN_Y = true;  // Sensor flipped around Y axis, after rotation (X and Z invert)

#define ACCEL_SENSOR_MPU6050    1
#define ACCEL_SENSOR_LIS3DH     2
#define ACCEL_SENSOR_TYPE       ACCEL_SENSOR_LIS3DH    // Change this single line to switch accelerometer

#if ACCEL_SENSOR_TYPE == ACCEL_SENSOR_LIS3DH
static constexpr float ACCEL_FULL_SCALE = 2048.0f; // 12-bit right-justified
#else
static constexpr float ACCEL_FULL_SCALE = 32768.0f; // 16-bit
#endif


// #########################################################
// Buttons mappings
// #########################################################

// Classic gamepad buttons
enum class ClassicBtn : uint16_t
{
    A               = BTN_A,
    B               = BTN_B,
    X               = BTN_X,
    Y               = BTN_Y,
    SELECT          = BTN_SELECT,
    START           = BTN_START,
    LAUNCH          = BTN_X,
    LEFT_FLIPPER    = BTN_LB,
    RIGHT_FLIPPER   = BTN_RB,
    LEFT_MAGNASAVE  = TRIGGER_LEFT,
    RIGHT_MAGNASAVE = TRIGGER_RIGHT,
    UP              = DPAD_UP,
    DOWN            = DPAD_DOWN,
    LEFT            = DPAD_LEFT,
    RIGHT           = DPAD_RIGHT,
};

// FX keys
enum class FxKey : uint8_t
{
    A               = KEY_I,
    B               = KEY_I,
    X               = KEY_I,
    Y               = KEY_I,
    SELECT          = KEY_I,
    START           = KEY_I,
    LAUNCH          = KEY_8,
    LEFT_FLIPPER    = KEY_U,
    RIGHT_FLIPPER   = KEY_6,
    LEFT_MAGNASAVE  = KEY_8,
    RIGHT_MAGNASAVE = KEY_8,
    UP              = KEY_NONE,
    DOWN            = KEY_NONE,
    LEFT            = KEY_NONE,
    RIGHT           = KEY_NONE,
};

// VPX keys
enum class VpxKey : uint8_t
{
    A               = KEY_ESC, // Pause game and display VR launcher
    B               = KEY_KP8, // Numpad 8: recenter view in VR
    X               = KEY_KP2, // Numpad 2: lower table in VR
    Y               = KEY_KP5, // Numpad 5: upper table in VR
    SELECT          = KEY_5,
    START           = KEY_1,
    LAUNCH          = KEY_ENTER,
    LEFT_FLIPPER    = KEY_LEFT_SHIFT,
    RIGHT_FLIPPER   = KEY_RIGHT_SHIFT,
    LEFT_MAGNASAVE  = KEY_LEFT_CTRL,
    RIGHT_MAGNASAVE = KEY_RIGHT_CTRL,
    UP              = KEY_LEFT_SHIFT,  // Navigate in VR launcher menu
    DOWN            = KEY_RIGHT_SHIFT, // ...
    LEFT            = KEY_LEFT_CTRL,   // ...
    RIGHT           = KEY_RIGHT_CTRL,  // ...
};


// #########################################################
// Nudge direction mappings
// #########################################################

// FX keys
enum class FxNudgeKey : uint8_t
{
    FORWARD = KEY_A,
    RIGHT   = KEY_D,
    LEFT    = KEY_F,
};


// #########################################################
// BLE
// #########################################################
constexpr auto DEVICE_NAME                 = "VR Pinball controller"; // BLE device name
constexpr auto DEVICE_MANUFACTURER         = "CosmicMac";             // BLE device manufacturer
constexpr uint8_t BTN_DEBOUNCE_MS          = 10;                      // Debounce delay for buttons in ms
constexpr uint16_t CONFIG_SAVE_INTERVAL_MS = 5000;                    // Interval to save configuration to flash in ms


// #########################################################
// Nudge parameters
// #########################################################
#define DEBUG_ANALOG_NUDGE  true

constexpr uint16_t NUDGE_SAMPLE_RATE_HZ        = 400; // Accelerometer sampling rate in Hz
constexpr uint16_t ANALOG_NUDGE_REPORT_RATE_HZ = 120; // HID report rate in Hz

#if ACCEL_SENSOR_TYPE == ACCEL_SENSOR_MPU6050
constexpr int NUDGE_JITTER_WINDOW                  = 800;  // Dead zone around zero (raw units) to filter out sensor noise
constexpr uint16_t ANALOG_NUDGE_MAX_ACCELERATION   = 7000; // Maximum expected acceleration for left stick HID scaling (raw units) - Decrease if you want more sensitivity
constexpr uint16_t ANALOG_NUDGE_MAX_VELOCITY       = 150;  // Maximum expected velocity for right stick HID scaling (mm/s) - Decrease if you want more sensitivity
constexpr uint16_t DIGITAL_NUDGE_EVAL_RATE_HZ      = 50;   // Frequency to evaluate nudge state and trigger key events in Hz
constexpr uint16_t DIGITAL_NUDGE_THRESHOLD         = 3000; // Trigger threshold
constexpr uint16_t DIGITAL_NUDGE_RELEASE_THRESHOLD = 1500; // Release threshold (hysteresis)
constexpr uint32_t DIGITAL_NUDGE_COOLDOWN_MS       = 200;  // Delay between two motion detections
constexpr uint32_t DIGITAL_NUDGE_RESET_MS          = 50;   // Time to reset nudge to center
#elif ACCEL_SENSOR_TYPE == ACCEL_SENSOR_LIS3DH
constexpr int NUDGE_JITTER_WINDOW                  = 200;  // Dead zone around zero (raw units) to filter out sensor noise
constexpr uint16_t ANALOG_NUDGE_MAX_ACCELERATION   = 700;  // Maximum expected acceleration for left stick HID scaling (raw units) - Decrease if you want more sensitivity !HERE
constexpr uint16_t ANALOG_NUDGE_MAX_VELOCITY       = 10;   // Maximum expected velocity for right stick HID scaling (mm/s) - Decrease if you want more sensitivity !HERE
constexpr uint16_t DIGITAL_NUDGE_EVAL_RATE_HZ      = 50;   // Frequency to evaluate nudge state and trigger key events in Hz
constexpr uint16_t DIGITAL_NUDGE_THRESHOLD         = 300;  // Trigger threshold
constexpr uint16_t DIGITAL_NUDGE_RELEASE_THRESHOLD = 150;  // Release threshold (hysteresis)
constexpr uint32_t DIGITAL_NUDGE_COOLDOWN_MS       = 200;  // Delay between two motion detections
constexpr uint32_t DIGITAL_NUDGE_RESET_MS          = 50;   // Time to reset nudge to center
#endif

// #########################################################
// Calculated values from constants above (don't change)
// #########################################################
constexpr uint32_t NUDGE_SAMPLE_INTERVAL_US        = static_cast<uint32_t>(1000000.0f / static_cast<float>(NUDGE_SAMPLE_RATE_HZ));        // Accelerometer sampling interval in microseconds
constexpr uint32_t ANALOG_NUDGE_REPORT_INTERVAL_US = static_cast<uint32_t>(1000000.0f / static_cast<float>(ANALOG_NUDGE_REPORT_RATE_HZ)); // HID report interval in microseconds
constexpr uint32_t DIGITAL_NUDGE_EVAL_INTERVAL_US  = static_cast<uint32_t>(1000000.0f / static_cast<float>(DIGITAL_NUDGE_EVAL_RATE_HZ));  // Nudge state evaluation interval in microseconds
constexpr float ANALOG_NUDGE_ACCELERATION_SCALE    = 32767.0f / static_cast<float>(ANALOG_NUDGE_MAX_ACCELERATION);
constexpr float ANALOG_NUDGE_VELOCITY_SCALE        = 32767.0f / static_cast<float>(ANALOG_NUDGE_MAX_VELOCITY);
