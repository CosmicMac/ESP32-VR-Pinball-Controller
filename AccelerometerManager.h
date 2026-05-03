#pragma once
#include "IAccelerometer.h"
#include <memory>
#include "config.h"

class AccelerometerManager {
public:
    AccelerometerManager();
    ~AccelerometerManager() = default;

    void begin();
    bool readRaw(int16_t& x, int16_t& y);
    bool readRaw(int16_t& x, int16_t& y, int16_t& z);

private:
    std::unique_ptr<IAccelerometer> sensor;
    
    void applyRotationAndFlip(int16_t& x, int16_t& y);
};
