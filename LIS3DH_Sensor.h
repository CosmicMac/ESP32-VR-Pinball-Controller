#pragma once
#include "IAccelerometer.h"
#include <7Semi_LIS3DH.h>

class LIS3DH_Sensor : public IAccelerometer {
public:
    LIS3DH_Sensor();
    ~LIS3DH_Sensor() override = default;

    bool initialize() override;
    void calibrate() override;
    bool read(int16_t& x, int16_t& y, int16_t& z) override;

    int16_t getOffsetX() const { return offsetX; }
    int16_t getOffsetY() const { return offsetY; }
    int16_t getOffsetZ() const { return offsetZ; }

private:
    LIS3DH_7Semi sensor;
    int16_t offsetX = 0;
    int16_t offsetY = 0;
    int16_t offsetZ = 0;
};
