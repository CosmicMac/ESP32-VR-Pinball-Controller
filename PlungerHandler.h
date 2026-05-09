/**
 * Plunger sensor input handling for VR pinball.
 * 
 * Manages analog sensor calibration, exponential filtering, dead zone handling,
 * and mapping to HID left trigger axis for VR pinball plunger control.
 */
#pragma once
#include "config.h"
#include "ESP32_VR_Pinball_Controller.h"

// Forward declaration
class BleHidController;

class PlungerHandler
{
public:
    PlungerHandler();

    ~PlungerHandler() = default;

    /**
     * Calibrates sensor by sampling rest position.
     * 
     * Samples analog input to determine minimum value and usable range.
     * Keep plunger at rest during calibration.
     * 
     * @param hidController BLE HID controller reference.
     */
    void begin(BleHidController& hidController);

    /**
     * Processes input: reads, filters, normalizes, and maps to HID trigger.
     * 
     * Applies exponential low-pass filter, normalizes to [0,1], handles dead zone,
     * and maps to HID left trigger axis [0, 32767].
     * 
     * @param debugMode Enable debug output.
     */
    void handle(bool debugMode);

private:
    bool m_isCalibrated               = false;           // Calibration state
    float m_filteredValue             = 0.0f;            // Current filtered sensor value (exponential moving average)
    uint16_t m_minValue               = PLUNGER_MAX_VAL; // Minimum sensor value detected during calibration (rest position)
    uint16_t m_valueRange             = PLUNGER_MAX_VAL; // Usable sensor range (max - min) for normalization
    uint32_t m_lastDebugPrint         = 0;               // Timestamp of last debug print for rate limiting
    BleHidController* m_hidController = nullptr;         // Pointer to BLE HID controller for sending reports
};
