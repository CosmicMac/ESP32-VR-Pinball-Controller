#pragma once
#include <MPU6050.h>
#include "BleHidController.h"

// #########################################################
// GPIO pins for buttons
// Make sure to connect one leg of each button to GND
// and the other leg to the GPIO pin defined here
// #########################################################
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


// #########################################################
// GPIO pins for MPU6050
// #########################################################
constexpr uint8_t I2C_SDA_PIN = 8;  // I2C SDA pin
constexpr uint8_t I2C_SCL_PIN = 9;  // I2C SCL pin
constexpr uint8_t MPU_INT_PIN = 10; // MPU6050 interrupt pin


// #########################################################
// MPU6050 parameters
// #########################################################
constexpr uint8_t MPU6050_ADDR               = 0x68;                // MPU6050 I2C address
constexpr uint8_t ACCEL_RANGE                = MPU6050_ACCEL_FS_2;  // Accelerometer range (±2g)
constexpr uint8_t DLPF_MODE                  = MPU6050_DLPF_BW_188; // Digital low-pass filter configuration (from 5 to 256 Hz, the fastest the noisiest), default : MPU6050_DLPF_BW_188
constexpr uint8_t MOTION_DETECTION_THRESHOLD = 4;                   // Motion detection threshold in MPU6050 units, default: 4
constexpr uint32_t COOLDOWN_MS               = 200;                 // Delay between two motion detections
constexpr uint32_t NUDGE_RESET_MS            = 50;                  // Time to reset nudge to center
constexpr uint8_t NUDGE_SAMPLES              = 5;                   // Number of samples to average for nudge detection
constexpr uint8_t ACCEL_XOUT_H               = 0x3B;                // MPU6050 register address for accelerometer X-axis high byte


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
    LEFT_MAGNASAVE  = BTN_NONE,
    RIGHT_MAGNASAVE = BTN_NONE,
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
    A               = KEY_ESC,              // Pause game and display VR launcher
    B               = KEY_KP8,              // Numpad 8: recenter view in VR
    X               = KEY_KP2,              // Numpad 2: lower table in VR
    Y               = KEY_KP5,              // Numpad 5: upper table in VR
    SELECT          = KEY_5,
    START           = KEY_1,
    LAUNCH          = KEY_ENTER,
    LEFT_FLIPPER    = KEY_LEFT_SHIFT,
    RIGHT_FLIPPER   = KEY_RIGHT_SHIFT,
    LEFT_MAGNASAVE  = KEY_LEFT_CTRL,
    RIGHT_MAGNASAVE = KEY_RIGHT_CTRL,
    UP              = KEY_LEFT_SHIFT,       // Navigate in VR launcher menu
    DOWN            = KEY_RIGHT_SHIFT,      // ...
    LEFT            = KEY_LEFT_CTRL,        // ...
    RIGHT           = KEY_RIGHT_CTRL,       // ...
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

// VPX keys
enum class VpxNudgeKey : uint8_t
{
    FORWARD = KEY_SPACE,
    LEFT    = KEY_Z,
    RIGHT   = KEY_SLASH,
};


// #########################################################
// BLE
// #########################################################
constexpr auto DEVICE_NAME                 = "VR Pinball controller"; // BLE device name
constexpr auto DEVICE_MANUFACTURER         = "CosmicMac";             // BLE device manufacturer
constexpr uint8_t BTN_DEBOUNCE_MS          = 10;                      // Debounce delay for buttons in ms
constexpr uint16_t CONFIG_SAVE_INTERVAL_MS = 5000;                    // Interval to save configuration to flash in ms
