#pragma once
#include <cstdint>

class IAccelerometer {
public:
    virtual ~IAccelerometer() = default;
    virtual bool initialize() = 0;
    virtual void calibrate() = 0;
    virtual bool read(int16_t& x, int16_t& y, int16_t& z) = 0;
};
