#include "LIS3DH_Sensor.h"
#include <Wire.h>
#include <Arduino.h>

LIS3DH_Sensor::LIS3DH_Sensor() : sensor() {}

bool LIS3DH_Sensor::initialize() {
    if (!sensor.begin(Wire)) {
        Serial.println("LIS3DH init failed!");
        return false;
    }
    
    sensor.setScale(RANGE_2G);
    sensor.setDataRate(ODR_400HZ);
    sensor.enableTemperature(false);
    sensor.setHighResolution(true);
    
    return true;
}

void LIS3DH_Sensor::calibrate() {
    constexpr uint8_t CALIB_SAMPLES = 64;
    constexpr uint16_t CALIB_DELAY = 10;
    constexpr uint8_t PREVIEW_SAMPLES = 10;
    
    int32_t preX = 0, preY = 0, preZ = 0;
    uint8_t preCount = 0;
    int16_t xRaw, yRaw, zRaw;
    
    // Average of samples before calibration
    for (uint8_t i = 0; i < PREVIEW_SAMPLES; i++) {
        if (sensor.readAccel(xRaw, yRaw, zRaw)) {
            preX += xRaw;
            preY += yRaw;
            preZ += zRaw;
            preCount++;
        }
        delay(CALIB_DELAY);
    }
    if (preCount > 0) {
        Serial.printf("Before calibration (avg %d samples) -> X: %d  Y: %d  Z: %d\n", preCount, preX / preCount, preY / preCount, preZ / preCount);
    }
    
    // Calibration: compute offsets over CALIB_SAMPLES readings
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    uint8_t count = 0;
    
    for (uint8_t i = 0; i < CALIB_SAMPLES; i++) {
        if (sensor.readAccel(xRaw, yRaw, zRaw)) {
            sumX += xRaw;
            sumY += yRaw;
            sumZ += zRaw;
            count++;
        }
        delay(CALIB_DELAY);
    }
    
    if (count > 0) {
        offsetX = static_cast<int16_t>(sumX / count);
        offsetY = static_cast<int16_t>(sumY / count);
        offsetZ = static_cast<int16_t>(sumZ / count);
    }
    
    Serial.printf("Calibration done. Offsets -> X: %d  Y: %d  Z: %d\n", offsetX, offsetY, offsetZ);
    
    // Average of samples after calibration
    int32_t postX = 0, postY = 0, postZ = 0;
    uint8_t postCount = 0;
    
    for (uint8_t i = 0; i < PREVIEW_SAMPLES; i++) {
        if (sensor.readAccel(xRaw, yRaw, zRaw)) {
            postX += xRaw - offsetX;
            postY += yRaw - offsetY;
            postZ += zRaw - offsetZ;
            postCount++;
        }
        delay(CALIB_DELAY);
    }
    if (postCount > 0) {
        Serial.printf("After calibration (avg %d samples) -> X: %d  Y: %d  Z: %d\n", postCount, postX / postCount, postY / postCount, postZ / postCount);
    }
}

bool LIS3DH_Sensor::read(int16_t& x, int16_t& y, int16_t& z) {
    if (!sensor.readAccel(x, y, z)) {
        return false;
    }
    
    x -= offsetX;
    y -= offsetY;
    z -= offsetZ;
    
    return true;
}
