#include "AccelerometerManager.h"
#include "MPU6050_Sensor.h"
#include "LIS3DH_Sensor.h"
#include <Wire.h>
#include <Arduino.h>

AccelerometerManager::AccelerometerManager() : sensor(nullptr) {}

void AccelerometerManager::begin() {
    // Initialize I2C
    if (!Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN)) {
        Serial.println("I2C init failed!");
        return;
    }
    Wire.setClock(400000);

    // Instantiate concrete sensor based on configuration
#if ACCEL_SENSOR_TYPE == ACCEL_SENSOR_MPU6050
    sensor = std::make_unique<MPU6050_Sensor>(ACCEL_SENSOR_ADDR);
#elif ACCEL_SENSOR_TYPE == ACCEL_SENSOR_LIS3DH
    sensor = std::make_unique<LIS3DH_Sensor>();
#endif

    if (sensor) {
        if (!sensor->initialize()) {
            Serial.println("Accelerometer initialization failed!");
            return;
        }
        sensor->calibrate();
    }
}

bool AccelerometerManager::readRaw(int16_t& x, int16_t& y, int16_t& z) {
    if (!sensor) return false;
    
    if (!sensor->read(x, y, z)) {
        return false;
    }
    
    applyRotationAndFlip(x, y);
    return true;
}

bool AccelerometerManager::readRaw(int16_t& x, int16_t& y) {
    int16_t z;
    return readRaw(x, y, z);
}

void AccelerometerManager::applyRotationAndFlip(int16_t& x, int16_t& y) {
    int16_t xr = x, yr = y;
    
    // Rotation around Z axis (CCW, viewed from +Z)
    switch (ACCEL_SENSOR_ROTATION) {
        case 0:     xr = x;     yr = y;     break;
        case 90:    xr = -y;    yr = x;     break;
        case 180:   xr = -x;    yr = -y;    break;
        case 270:   xr = y;     yr = -x;    break;
        default:    xr = x;     yr = y;     break;
    }
    
    // Apply flips (after rotation)
    if (ACCEL_SENSOR_UPSIDEDOWN_X) {
        yr = -yr;
    }
    if (ACCEL_SENSOR_UPSIDEDOWN_Y) {
        xr = -xr;
    }
    
    x = xr;
    y = yr;
}
