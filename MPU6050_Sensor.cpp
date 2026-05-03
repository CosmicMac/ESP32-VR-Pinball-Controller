#include "MPU6050_Sensor.h"
#include <Wire.h>
#include <Arduino.h>

MPU6050_Sensor::MPU6050_Sensor(uint8_t addr) : mpu(addr), address(addr) {}

bool MPU6050_Sensor::initialize() {
    mpu.initialize();
    mpu.setDLPFMode(MPU6050_DLPF_BW_188);
    mpu.setRate(1);
    
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        return false;
    }
    return true;
}

void MPU6050_Sensor::calibrate() {
    Serial.println("Calibrating... Keep the MPU6050 sensor still.");
    mpu.CalibrateAccel();
}

bool MPU6050_Sensor::read(int16_t& x, int16_t& y, int16_t& z) {
    mpu.getAcceleration(&x, &y, &z);
    return true;
}
