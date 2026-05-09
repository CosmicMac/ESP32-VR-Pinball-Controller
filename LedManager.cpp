#include "LedManager.h"

void LedManager::setColor(LedColor color) {
#ifdef RGB_BUILTIN
    switch (color) {
        //@formatter:off
        case LedColor::OFF:             rgbLedWrite(RGB_BUILTIN, 0, 0, 0); break;
        case LedColor::RED:             rgbLedWrite(RGB_BUILTIN, LED_BRIGHTNESS, 0, 0); break;
        case LedColor::CLASSIC_MODE:
        case LedColor::GREEN:           rgbLedWrite(RGB_BUILTIN, 0, LED_BRIGHTNESS, 0); break;
        case LedColor::FX_MODE:
        case LedColor::BLUE:            rgbLedWrite(RGB_BUILTIN, 0, 0, LED_BRIGHTNESS); break;
        case LedColor::YELLOW:          rgbLedWrite(RGB_BUILTIN, LED_BRIGHTNESS, LED_BRIGHTNESS, 0); break;
        case LedColor::VPX_MODE:
        case LedColor::PURPLE:          rgbLedWrite(RGB_BUILTIN, LED_BRIGHTNESS, 0, LED_BRIGHTNESS); break;
        case LedColor::CYAN:            rgbLedWrite(RGB_BUILTIN, 0, LED_BRIGHTNESS, LED_BRIGHTNESS); break;
        case LedColor::WHITE:           rgbLedWrite(RGB_BUILTIN, LED_BRIGHTNESS, LED_BRIGHTNESS, LED_BRIGHTNESS); break;
        default: break;
        //@formatter:on
    }
#endif
}

void LedManager::blink(LedColor color, uint8_t times) {
#ifdef RGB_BUILTIN
    // Execute blink cycles: on for BLINK_DELAY_MS, off for BLINK_DELAY_MS
    for (uint8_t i = 0; i < times; ++i) {
        setColor(color);
        delay(BLINK_DELAY_MS);
        setColor(LedColor::OFF);
        delay(BLINK_DELAY_MS);
    }
#else
    // Suppress unused parameter warnings when RGB_BUILTIN is not defined
    (void)color;
    (void)times;
#endif
}
