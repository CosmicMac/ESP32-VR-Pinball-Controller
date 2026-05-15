#include "NudgeHandler.h"
#include "AccelerometerManager.h"
#include "BleHidController.h"
#include <Arduino.h>

NudgeHandler::NudgeHandler() 
    : m_hidController(nullptr)
    , m_accelManager(nullptr)
    , m_lastSampleMicros(0)
    , m_lastReportMicros(0)
    , m_lastEvalMicros(0)
    , m_lastPrint(0)
    , m_lastReset(0)
    , m_maxAccX(0.0f)
    , m_maxAccY(0.0f)
    , m_maxVelX(0.0f)
    , m_maxVelY(0.0f)
    , m_maxLeftX(0)
    , m_maxLeftY(0)
    , m_maxRightX(0)
    , m_maxRightY(0)
    , m_peakX(0.0f)
    , m_peakY(0.0f) {
}

void NudgeHandler::begin(BleHidController& hidController, AccelerometerManager& accelManager) {
    m_hidController = &hidController;
    m_accelManager = &accelManager;
}

/**
 * Reads raw acceleration values from the sensor using AccelerometerManager
 *
 * @param xr Reference to store the calibrated and adjusted X-axis acceleration
 * @param yr Reference to store the calibrated and adjusted Y-axis acceleration
 * @return True if the acceleration values are successfully read and adjusted; false otherwise
 */
bool NudgeHandler::readAccelRaw(int16_t& xr, int16_t& yr) {
    if (!m_accelManager) return false;
    return m_accelManager->readRaw(xr, yr);
}

/**
 * Processes input from the accelerometer at a defined sampling interval
 * and updates the nudge subsystem with the latest raw acceleration values.
 *
 * @return true if a new sample was read and processed, false if the sampling interval has not yet elapsed
 */
bool NudgeHandler::sampleNudge() {
    const uint32_t now = micros();

    if (now - m_lastSampleMicros < NUDGE_SAMPLE_INTERVAL_US) return false;
    m_lastSampleMicros = now;

    int16_t rx, ry;

    if (readAccelRaw(rx, ry)) {
        m_nudgeX.process(rx, now);
        m_nudgeY.process(ry, now);
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
void NudgeHandler::handleAnalog(bool debugMode) {
    const uint32_t now = micros();

    sampleNudge();

    if (now - m_lastReportMicros < ANALOG_NUDGE_REPORT_INTERVAL_US) return;
    m_lastReportMicros = now;

    const float accX = m_nudgeX.acceleration;
    const float accY = m_nudgeY.acceleration;
    const float velX = m_nudgeX.velocity;
    const float velY = m_nudgeY.velocity;

    // Left stick: acceleration (Classic)
    const int16_t leftX = static_cast<int16_t>(std::clamp(accX * ANALOG_NUDGE_ACCELERATION_SCALE, -32767.0f, 32767.0f));
    const int16_t leftY = static_cast<int16_t>(std::clamp(accY * ANALOG_NUDGE_ACCELERATION_SCALE, -32767.0f, 32767.0f));

    // Right stick: velocity (VPX)
    const int16_t rightX = static_cast<int16_t>(std::clamp(velX * ANALOG_NUDGE_VELOCITY_SCALE, -32767.0f, 32767.0f));
    const int16_t rightY = static_cast<int16_t>(std::clamp(velY * ANALOG_NUDGE_VELOCITY_SCALE, -32767.0f, 32767.0f));

    // Send both axes together
    if (m_hidController) {
        m_hidController->setLeftStick(leftX, leftY, false);
        m_hidController->setRightStick(rightX, rightY, false);
    }
    // hid.sendGamepadState();

    if (debugMode) {
        if (fabsf(m_nudgeX.acceleration) > fabsf(m_maxAccX)) m_maxAccX = m_nudgeX.acceleration;
        if (fabsf(m_nudgeY.acceleration) > fabsf(m_maxAccY)) m_maxAccY = m_nudgeY.acceleration;

        if (fabsf(m_nudgeX.velocity) > fabsf(m_maxVelX)) m_maxVelX = m_nudgeX.velocity;
        if (fabsf(m_nudgeY.velocity) > fabsf(m_maxVelY)) m_maxVelY = m_nudgeY.velocity;

        if (abs(leftX) > abs(m_maxLeftX)) m_maxLeftX = leftX;
        if (abs(leftY) > abs(m_maxLeftY)) m_maxLeftY = leftY;

        if (abs(rightX) > abs(m_maxRightX)) m_maxRightX = rightX;
        if (abs(rightY) > abs(m_maxRightY)) m_maxRightY = rightY;

        if (now - m_lastPrint > 1000000) {
            Serial.printf(
                "maxAcc[%7.1f, %7.1f] / maxVel[%7.1f, %7.1f] "
                "*** maxLeftStick[%6d, %6d] / maxRightStick[%6d, %6d]\n",
                m_maxAccX, m_maxAccY, m_maxVelX, m_maxVelY,
                m_maxLeftX, m_maxLeftY, m_maxRightX, m_maxRightY);
            m_lastPrint = now;

            if (now - m_lastReset > 5000000) {
                Serial.println("\nResetting debug counters...");
                m_maxAccX   = 0.0f;
                m_maxAccY   = 0.0f;
                m_maxVelX   = 0.0f;
                m_maxVelY   = 0.0f;
                m_maxLeftX  = 0;
                m_maxLeftY  = 0;
                m_maxRightX = 0;
                m_maxRightY = 0;
                m_lastReset = now;
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
void NudgeHandler::handleDigital(bool debugMode) {
    const uint32_t now = micros();

    // Accumulate peak acceleration for direction detection over the evaluation window
    if (sampleNudge()) {
        if (fabsf(m_nudgeX.acceleration) > fabsf(m_peakX)) m_peakX = m_nudgeX.acceleration;
        if (fabsf(m_nudgeY.acceleration) > fabsf(m_peakY)) m_peakY = m_nudgeY.acceleration;
    }

    /**
     * State evaluation
     */
    if (now - m_lastEvalMicros < DIGITAL_NUDGE_EVAL_INTERVAL_US) return;
    m_lastEvalMicros = now;

    const float absPeakX      = fabsf(m_peakX);
    const float absPeakY      = fabsf(m_peakY);
    const bool aboveThreshold = absPeakX > static_cast<float>(DIGITAL_NUDGE_THRESHOLD) || absPeakY > static_cast<float>(DIGITAL_NUDGE_THRESHOLD);
    const uint32_t nowMs      = millis();

    if (debugMode) {
        Serial.printf("[DEBUG] peakX=%.1f, peakY=%.1f, threshold=%d, isNudging=%d\n", absPeakX, absPeakY, DIGITAL_NUDGE_THRESHOLD, m_nudgeState.isNudging);
    }

    // Nudge trigger
    if (
        aboveThreshold && !m_nudgeState.isNudging &&
        nowMs - m_nudgeState.lastNudgeMillis > DIGITAL_NUDGE_COOLDOWN_MS
    ) {
        m_nudgeState.lastNudgeMillis = nowMs;
        m_nudgeState.isNudging       = true;
        m_nudgeState.nudgeKey        = 0;

        // Determine nudge direction from the dominant peak axis
        if (absPeakY >= absPeakX) {
            if (m_peakY > 0) m_nudgeState.nudgeKey = static_cast<uint8_t>(FxNudgeKey::FORWARD);
        }
        else {
            m_nudgeState.nudgeKey = static_cast<uint8_t>(m_peakX < 0 ? FxNudgeKey::LEFT : FxNudgeKey::RIGHT);
        }
        if (m_nudgeState.nudgeKey != 0 && m_hidController) {
            m_hidController->keyPress(m_nudgeState.nudgeKey);
        }
    }
    // Nudge release (hysteresis)
    else if (
        m_nudgeState.isNudging &&
        absPeakX < static_cast<float>(DIGITAL_NUDGE_RELEASE_THRESHOLD) &&
        absPeakY < static_cast<float>(DIGITAL_NUDGE_RELEASE_THRESHOLD) &&
        nowMs - m_nudgeState.lastNudgeMillis > DIGITAL_NUDGE_RESET_MS
    ) {
        m_nudgeState.isNudging = false;
        if (m_nudgeState.nudgeKey != 0 && m_hidController) {
            m_hidController->keyRelease(m_nudgeState.nudgeKey);
            m_nudgeState.nudgeKey = 0;
        }
    }

    // Reset peak accumulators for the next evaluation window
    m_peakX = 0.0f;
    m_peakY = 0.0f;
}
