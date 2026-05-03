#include "NudgeHandler.h"
#include "AccelerometerManager.h"
#include "BleHidController.h"
#include <Arduino.h>

extern AccelerometerManager accelManager;
extern BleHidController hid;

NudgeHandler::NudgeHandler() {
}

void NudgeHandler::setup() {
    // Initialization is handled in process() when first called
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
bool NudgeHandler::sampleNudge() {
    const uint32_t now = micros();

    static uint32_t lastSampleMicros = 0;
    if (now - lastSampleMicros < NUDGE_SAMPLE_INTERVAL_US) return false;
    lastSampleMicros = now;

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

    static uint32_t lastReportMicros = 0;

    if (now - lastReportMicros < ANALOG_NUDGE_REPORT_INTERVAL_US) return;
    lastReportMicros = now;

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
    hid.setLeftStick(leftX, leftY, false);
    hid.setRightStick(rightX, rightY, false);
    // hid.sendGamepadState();

    if (debugMode) {
        static uint32_t lastPrint = 0, lastReset = 0;

        static float maxAccX     = 0.0f, maxAccY = 0.0f,
                     maxVelX     = 0.0f, maxVelY = 0.0f;
        static int16_t maxLeftX  = 0, maxLeftY   = 0,
                       maxRightX = 0, maxRightY  = 0;

        if (fabsf(m_nudgeX.acceleration) > fabsf(maxAccX)) maxAccX = m_nudgeX.acceleration;
        if (fabsf(m_nudgeY.acceleration) > fabsf(maxAccY)) maxAccY = m_nudgeY.acceleration;

        if (fabsf(m_nudgeX.velocity) > fabsf(maxVelX)) maxVelX = m_nudgeX.velocity;
        if (fabsf(m_nudgeY.velocity) > fabsf(maxVelY)) maxVelY = m_nudgeY.velocity;

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
void NudgeHandler::handleDigital(bool debugMode) {
    const uint32_t now = micros();

    static float peakX = 0.0f, peakY = 0.0f;

    // Accumulate peak acceleration for direction detection over the evaluation window
    if (sampleNudge()) {
        if (fabsf(m_nudgeX.acceleration) > fabsf(peakX)) peakX = m_nudgeX.acceleration;
        if (fabsf(m_nudgeY.acceleration) > fabsf(peakY)) peakY = m_nudgeY.acceleration;
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
            if (peakY > 0) m_nudgeState.nudgeKey = static_cast<uint8_t>(FxNudgeKey::FORWARD);
        }
        else {
            m_nudgeState.nudgeKey = static_cast<uint8_t>(peakX < 0 ? FxNudgeKey::LEFT : FxNudgeKey::RIGHT);
        }
        if (m_nudgeState.nudgeKey != 0) hid.keyPress(m_nudgeState.nudgeKey);
    }
    // Nudge release (hysteresis)
    else if (
        m_nudgeState.isNudging &&
        absPeakX < static_cast<float>(DIGITAL_NUDGE_RELEASE_THRESHOLD) &&
        absPeakY < static_cast<float>(DIGITAL_NUDGE_RELEASE_THRESHOLD) &&
        nowMs - m_nudgeState.lastNudgeMillis > DIGITAL_NUDGE_RESET_MS
    ) {
        m_nudgeState.isNudging = false;
        if (m_nudgeState.nudgeKey != 0) {
            hid.keyRelease(m_nudgeState.nudgeKey);
            m_nudgeState.nudgeKey = 0;
        }
    }

    // Reset peak accumulators for the next evaluation window
    peakX = 0.0f;
    peakY = 0.0f;
}
