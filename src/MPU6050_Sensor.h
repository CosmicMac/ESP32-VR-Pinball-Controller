#pragma once
#include "IAccelerometer.h"
#include <MPU6050.h>

class MPU6050_Sensor : public IAccelerometer {
public:
    explicit MPU6050_Sensor(uint8_t address = 0x68);
    ~MPU6050_Sensor() override = default;

    bool initialize() override;
    void calibrate() override;
    bool read(int16_t& x, int16_t& y, int16_t& z) override;

private:
    MPU6050 mpu;
    uint8_t address;
};
