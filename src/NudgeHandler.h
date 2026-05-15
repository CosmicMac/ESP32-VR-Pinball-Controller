#pragma once
#include "config.h"

// Forward declarations
class BleHidController;
class AccelerometerManager;

/**
 * Jitter Filter (hysteresis window)
 * Stabilizes a signal by absorbing small fluctuations within a sliding window
 */
struct JitterFilter
{
    int windowMin    = 0;
    int windowMax    = 0;
    bool initialized = false;

    int process(int raw) {
        if (!initialized) {
            initialized = true;
            windowMin   = raw;
            windowMax   = raw;
            return raw;
        }
        if (raw < windowMin) {
            windowMin = raw;
            windowMax = raw + NUDGE_JITTER_WINDOW;
        }
        else if (raw > windowMax) {
            windowMax = raw;
            windowMin = raw - NUDGE_JITTER_WINDOW;
        }
        return (windowMin + windowMax) / 2;
    }

    void reset(int raw) {
        initialized = true;
        windowMin   = raw;
        windowMax   = raw;
    }
};

struct NudgeProcess
{
    // Filter parameters
    static constexpr float ACCEL_G_RANGE        = 2.0f;
    static constexpr float DC_ADAPT_TIME_s      = 0.3f; // DC removal adaptation time — 200-500 ms recommended
    static constexpr float FRICTION_HALF_LIFE_s = 2.0f; // Velocity half-life in seconds — 2 s recommended

    // Optimization: Pre-calculate inverses to use multiplication instead of division (faster on microcontrollers without FPU)
    static constexpr float INV_DC_ADAPT_TIME      = 1.0f / DC_ADAPT_TIME_s;
    static constexpr float INV_FRICTION_HALF_LIFE = 1.0f / FRICTION_HALF_LIFE_s;

    // Conversion factor for converting raw units to mm/s²
    static constexpr float ACCEL_CONV_FACTOR = (ACCEL_G_RANGE / ACCEL_FULL_SCALE) * 9806.65f;

    // Internal state
    bool initialized    = false;
    float dcValue       = 0.0f; // DC Blocker (moving average)
    float acceleration  = 0.0f; // Filtered acceleration in raw sensor units
    float velocity      = 0.0f; // Integrated velocity in mm/s
    uint32_t lastMicros = 0;    // Timestamp of the last processed sample in microseconds
    JitterFilter jitter;

    void process(int raw, uint32_t nowMicros) {
        if (!initialized) {
            initialized = true;
            jitter.reset(raw);
            dcValue      = static_cast<float>(raw);
            acceleration = 0.0f;
            velocity     = 0.0f;
            lastMicros   = nowMicros;
            return;
        }

        // Calculation of Delta Time (dt) in seconds
        float dt = static_cast<float>(nowMicros - lastMicros) * 1e-6f;

        // Safety: Cap dt to prevent physics explosions if the loop lags significantly (e.g. > 100ms)
        if (dt > 0.1f) dt = 0.1f;

        lastMicros = nowMicros;

        // Jitter filter (hysteresis)
        const int stable = jitter.process(raw);

        // Dynamic DC Blocker (Gravity/Tilt Cancellation)
        // Use dt for consistent scaling regardless of the sample rate
        // Optimized: Used multiplication by inverse
        const float dcAlpha = std::clamp(dt * INV_DC_ADAPT_TIME, 0.0f, 1.0f);
        dcValue             += dcAlpha * (static_cast<float>(stable) - dcValue);
        acceleration        = static_cast<float>(stable) - dcValue;

        // Conversion to mm/s²
        const float accel_mm_s2 = acceleration * ACCEL_CONV_FACTOR;

        // Friction Filter (Velocity Attenuation)
        // Calculates the damping factor based on the actual elapsed time
        // Optimized: Used multiplication by inverse
        float friction = powf(0.5f, dt * INV_FRICTION_HALF_LIFE);
        velocity       *= friction;

        // Integration: v = v + (a * dt)
        // The acceleration is added to the velocity for this time step
        velocity += accel_mm_s2 * dt;
    }
};

struct NudgeState
{
    uint32_t lastNudgeMillis = 0;
    bool isNudging           = false;
    uint8_t nudgeKey         = 0;
};

class NudgeHandler
{
public:
    NudgeHandler();
    ~NudgeHandler() = default;

    void begin(BleHidController& hidController, AccelerometerManager& accelManager);
    void handleAnalog(bool debugMode);
    void handleDigital(bool debugMode);

private:
    bool sampleNudge();
    bool readAccelRaw(int16_t& xr, int16_t& yr);

    NudgeState m_nudgeState;
    NudgeProcess m_nudgeX;
    NudgeProcess m_nudgeY;
    BleHidController* m_hidController;
    AccelerometerManager* m_accelManager;

    // Timing variables (moved from static in functions)
    uint32_t m_lastSampleMicros = 0;
    uint32_t m_lastReportMicros = 0;
    uint32_t m_lastEvalMicros   = 0;

    // Analog nudge debug variables
    uint32_t m_lastPrint = 0;
    uint32_t m_lastReset = 0;
    float m_maxAccX      = 0.0f;
    float m_maxAccY      = 0.0f;
    float m_maxVelX      = 0.0f;
    float m_maxVelY      = 0.0f;
    int16_t m_maxLeftX   = 0;
    int16_t m_maxLeftY   = 0;
    int16_t m_maxRightX  = 0;
    int16_t m_maxRightY  = 0;

    // Digital nudge peak accumulators
    float m_peakX = 0.0f;
    float m_peakY = 0.0f;
};
