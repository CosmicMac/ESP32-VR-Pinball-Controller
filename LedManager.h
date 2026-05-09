#pragma once
#include <Arduino.h>
#include "config.h"

/**
 * RGB LED colors and mode indicators.
 * Mode colors map to specific operational modes (FX_MODE, CLASSIC_MODE, VPX_MODE).
 */
enum class LedColor : uint8_t { OFF, RED, GREEN, BLUE, YELLOW, PURPLE, CYAN, WHITE, FX_MODE, CLASSIC_MODE, VPX_MODE };

/**
 * Manages RGB LED for status indication.
 * Provides static methods to set LED colors and blink patterns.
 */
class LedManager
{
public:
    static constexpr uint16_t BLINK_DELAY_MS = 200;                                                             // Delay between LED on/off states in milliseconds
    static constexpr LedColor MODE_COLORS[]  = {LedColor::FX_MODE, LedColor::CLASSIC_MODE, LedColor::VPX_MODE}; // Array mapping mode indices to LED colors

    /**
     * Set LED to specified color.
     * 
     * @param color LED color to display.
     */
    static void setColor(LedColor color);

    /**
     * Blink LED specified times.
     * 
     * @param color LED color to blink (default: WHITE).
     * @param times Number of blink cycles (default: 3).
     */
    static void blink(LedColor color = LedColor::WHITE, uint8_t times = 3);
};
