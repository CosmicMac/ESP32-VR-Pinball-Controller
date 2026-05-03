#pragma once
#include <Arduino.h>

// Report IDs
constexpr uint8_t REPORT_ID_KEYBOARD = 1;
constexpr uint8_t REPORT_ID_GAMEPAD  = 2;

// Key codes
enum class KeyCode : uint8_t {
    NONE            = 0x00,
    ERROR_ROLLOVER  = 0x01,
    POST_FAIL       = 0x02,
    ERROR_UNDEFINED = 0x03,
    A               = 0x04,
    B               = 0x05,
    C               = 0x06,
    D               = 0x07,
    E               = 0x08,
    F               = 0x09,
    G               = 0x0A,
    H               = 0x0B,
    I               = 0x0C,
    J               = 0x0D,
    K               = 0x0E,
    L               = 0x0F,
    M               = 0x10,
    N               = 0x11,
    O               = 0x12,
    P               = 0x13,
    Q               = 0x14,
    R               = 0x15,
    S               = 0x16,
    T               = 0x17,
    U               = 0x18,
    V               = 0x19,
    W               = 0x1A,
    X               = 0x1B,
    Y               = 0x1C,
    Z               = 0x1D,
    NUM_1           = 0x1E,
    NUM_2           = 0x1F,
    NUM_3           = 0x20,
    NUM_4           = 0x21,
    NUM_5           = 0x22,
    NUM_6           = 0x23,
    NUM_7           = 0x24,
    NUM_8           = 0x25,
    NUM_9           = 0x26,
    NUM_0           = 0x27,
    ENTER           = 0x28,
    ESC             = 0x29,
    BACKSPACE       = 0x2A,
    TAB             = 0x2B,
    SPACE           = 0x2C,
    MINUS           = 0x2D,
    EQUAL           = 0x2E,
    LEFTBRACE       = 0x2F,
    RIGHTBRACE      = 0x30,
    BACKSLASH       = 0x31,
    HASHTILDE       = 0x32,
    SEMICOLON       = 0x33,
    APOSTROPHE      = 0x34,
    GRAVE           = 0x35,
    COMMA           = 0x36,
    DOT             = 0x37,
    SLASH           = 0x38,
    CAPSLOCK        = 0x39,
    F1              = 0x3A,
    F2              = 0x3B,
    F3              = 0x3C,
    F4              = 0x3D,
    F5              = 0x3E,
    F6              = 0x3F,
    F7              = 0x40,
    F8              = 0x41,
    F9              = 0x42,
    F10             = 0x43,
    F11             = 0x44,
    F12             = 0x45,
    SYSRQ           = 0x46,
    SCROLLLOCK      = 0x47,
    PAUSE           = 0x48,
    INSERT          = 0x49,
    HOME            = 0x4A,
    PAGEUP          = 0x4B,
    DELETE          = 0x4C,
    END             = 0x4D,
    PAGEDOWN        = 0x4E,
    RIGHT           = 0x4F,
    LEFT            = 0x50,
    DOWN            = 0x51,
    UP              = 0x52,
    NUMLOCK         = 0x53,
    KPSLASH         = 0x54,
    KPASTERISK      = 0x55,
    KPMINUS         = 0x56,
    KPPLUS          = 0x57,
    KPENTER         = 0x58,
    KP1             = 0x59,
    KP2             = 0x5A,
    KP3             = 0x5B,
    KP4             = 0x5C,
    KP5             = 0x5D,
    KP6             = 0x5E,
    KP7             = 0x5F,
    KP8             = 0x60,
    KP9             = 0x61,
    KP0             = 0x62,
    KPDOT           = 0x63,
    KP102ND         = 0x64,
    COMPOSE         = 0x65,
    POWER           = 0x66,
    KPEQUAL         = 0x67,
    SECTIONSIGN     = 0x68,
    // Modifier key codes (support not guaranteed)
    LEFT_CTRL       = 0xE0,
    LEFT_SHIFT      = 0xE1,
    LEFT_ALT        = 0xE2,
    LEFT_META       = 0xE3, // Windows/Command/Meta
    RIGHT_CTRL      = 0xE4,
    RIGHT_SHIFT     = 0xE5,
    RIGHT_ALT       = 0xE6,
    RIGHT_META      = 0xE7
};

// Modifier codes
enum class KeyModifier : uint8_t {
    NONE         = 0x00,
    LEFT_CTRL    = 0x01,
    LEFT_SHIFT   = 0x02,
    LEFT_ALT     = 0x04,
    LEFT_META    = 0x08, // Windows/Command/Meta
    RIGHT_CTRL   = 0x10,
    RIGHT_SHIFT  = 0x20,
    RIGHT_ALT    = 0x40,
    RIGHT_META   = 0x80
};

// Gamepad buttons
enum class GamepadButton : uint16_t {
    NONE    = 0x00,
    A       = 0x01,
    B       = 0x02,
    X       = 0x08,
    Y       = 0x10,
    LB      = 0x40,
    RB      = 0x80,
    SELECT  = 0x400,
    START   = 0x800,
    HOME    = 0x1000
};

// D-Pad
enum class DpadDirection : uint8_t {
    CENTERED   = 0x0F,
    UP         = 0x00,
    UP_RIGHT   = 0x01,
    RIGHT      = 0x02,
    DOWN_RIGHT = 0x03,
    DOWN       = 0x04,
    DOWN_LEFT  = 0x05,
    LEFT       = 0x06,
    UP_LEFT    = 0x07
};

// Triggers (dummy codes for API, actual values are 0-1023 in the report)
constexpr uint16_t TRIGGER_LEFT  = 0x2000;
constexpr uint16_t TRIGGER_RIGHT = 0x4000;
