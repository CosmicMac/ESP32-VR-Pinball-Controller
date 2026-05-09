#include "PlungerHandler.h"
#include "BleHidController.h"
#include <Arduino.h>

/**
 * Plunger sensor implementation.
 * 
 * Handles analog sensor calibration, exponential filtering, dead zone processing,
 * and HID trigger mapping for VR pinball plunger control.
 */

PlungerHandler::PlungerHandler() = default;

void PlungerHandler::begin(BleHidController& hidController) {
    if constexpr (!PLUNGER_ENABLED) return;

    m_hidController = &hidController;

    Serial.println("[PlungerHandler] Calibrating... Keep the plunger in resting position.");
    const uint32_t start = millis();
    uint16_t sampleCount = 0;
    uint32_t sampleSum = 0;

    // Sample the sensor for the configured calibration duration
    while (millis() - start < PLUNGER_CALIBRATION_MS) {
        const uint16_t raw = analogRead(PLUNGER_PIN);

        // Track the minimum value (rest position)
        if (raw < m_minValue) {
            m_minValue = raw;
        }

        sampleSum += raw;
        sampleCount++;
        delay(PLUNGER_CALIBRATION_DELAY_MS);
    }

    // Validate calibration succeeded
    if (sampleCount == 0) {
        Serial.println("[PlungerHandler] Calibration failed - no samples collected");
        return;
    }

    // Calculate initial filtered value and sensor range
    m_filteredValue = static_cast<float>(sampleSum) / static_cast<float>(sampleCount);
    m_valueRange = PLUNGER_MAX_VAL - m_minValue;

    m_isCalibrated = true;
    Serial.printf("[PlungerHandler] Calibration done. minVal=%d, range=%d, samples=%d\n", m_minValue, m_valueRange, sampleCount);
}

void PlungerHandler::handle(bool debugMode) {
    if constexpr (!PLUNGER_ENABLED) return;
    if (!m_hidController) return;
    if (!m_isCalibrated) return;
    if (m_valueRange == 0) return;

    // Read raw analog value from plunger sensor
    const uint16_t raw = analogRead(PLUNGER_PIN);

    // Apply exponential low-pass filter for smooth output
    // filtered = alpha * new + (1 - alpha) * old
    m_filteredValue = PLUNGER_FILTER_ALPHA * static_cast<float>(raw) + (1.0f - PLUNGER_FILTER_ALPHA) * m_filteredValue;

    // Normalize to [0, 1] range:
    // - 0.0 = rest position (m_minValue)
    // - 1.0 = fully extended (PLUNGER_MAX_VAL)
    float normalizedValue = (m_filteredValue - static_cast<float>(m_minValue)) / static_cast<float>(m_valueRange);
    normalizedValue = std::clamp(normalizedValue, 0.0f, 1.0f);

    // Apply dead zone around rest position to eliminate noise
    // Values below the dead zone threshold are treated as rest position
    if (normalizedValue < PLUNGER_DEAD_ZONE) {
        normalizedValue = 0.0f;
    }
    else {
        // Remap the remaining range to [0, 1] for full utilization
        normalizedValue = (normalizedValue - PLUNGER_DEAD_ZONE) / (1.0f - PLUNGER_DEAD_ZONE);
    }

    // Map to HID left trigger axis range [0, 32767]
    // - 0 = rest position (plunger forward)
    // - 32767 = fully pulled back
    const auto zAxis = static_cast<int16_t>(normalizedValue * 32767.0f);

    // Optional debug output with rate limiting
    if (debugMode) {
        uint32_t now = millis();
        if (now - m_lastDebugPrint > PLUNGER_DEBUG_PRINT_INTERVAL_MS) {
            m_lastDebugPrint = now;
            Serial.printf("[PlungerHandler] raw=%d, filtered=%f, normalized=%.2f, zAxis=%d\n", raw, m_filteredValue, normalizedValue, zAxis);
        }
    }

    // Send the trigger value via BLE HID
    m_hidController->setLeftTrigger(zAxis, true);
}
