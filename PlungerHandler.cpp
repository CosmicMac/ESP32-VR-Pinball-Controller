#include "PlungerHandler.h"
#include <Arduino.h>

extern BleHidController hid;

PlungerHandler::PlungerHandler() 
    : m_filteredValue(0.0f)
    , m_minValue(PLUNGER_MAX_VAL) {
}

void PlungerHandler::setup() {
    if constexpr (!PLUNGER_ENABLED) return;

    Serial.println("[PlungerHandler] Calibrating... Keep the plunger in resting position.");
    const uint32_t start = millis();

    while (millis() - start < 2000) {
        if (const int raw = analogRead(PLUNGER_PIN); raw < m_minValue)
            m_minValue = raw;
        delay(5);
    }

    m_filteredValue = static_cast<float>(analogRead(PLUNGER_PIN));
    Serial.printf("[PlungerHandler] Calibration done. minVal=%d\n", m_minValue);
}

void PlungerHandler::handle(ControllerMode mode, bool debugMode) {
    if constexpr (!PLUNGER_ENABLED) return;
    if (mode != ControllerMode::VPX) return;

    const int raw = analogRead(PLUNGER_PIN);

    // Exponential low-pass filter
    m_filteredValue = PLUNGER_FILTER_ALPHA * static_cast<float>(raw) + (1.0f - PLUNGER_FILTER_ALPHA) * m_filteredValue;

    // Normalize to [0, 1]
    float norm = (m_filteredValue - static_cast<float>(m_minValue)) / static_cast<float>(PLUNGER_MAX_VAL - m_minValue);
    norm = std::clamp(norm, 0.0f, 1.0f);

    // Center to [-1, +1]
    float centered = (norm * 2.0f) - 1.0f;

    // Dead zone
    if (fabsf(centered) < PLUNGER_DEAD_ZONE) {
        centered = 0.0f;
    }
    else {
        centered = centered > 0.0f
                       ? (centered - PLUNGER_DEAD_ZONE) / (1.0f - PLUNGER_DEAD_ZONE)
                       : (centered + PLUNGER_DEAD_ZONE) / (1.0f - PLUNGER_DEAD_ZONE);
    }

    // Map to LT range [0, 32767] — fully forward = 0, fully pulled = 32767
    const auto zAxis = static_cast<uint16_t>(((centered + 1.0f) / 2.0f) * 32767.0f);

    if (debugMode) {
        uint32_t now = millis();
        static uint32_t lastPrint = 0;
        if (now - lastPrint > 1000) {
            lastPrint = now;
            Serial.printf("[PlungerHandler] raw=%d, filtered=%f, centered=%.2f, zAxis=%d\n", 
                          raw, m_filteredValue, centered, zAxis);
        }
    }

    hid.setLeftTrigger(zAxis, false);
}
