#pragma once
#include "config.h"
#include "ESP32_VR_Pinball_Controller.h"
#include "BleHidController.h"

class PlungerHandler {
public:
    PlungerHandler();
    ~PlungerHandler() = default;

    void setup();
    void handle(ControllerMode mode, bool debugMode);

private:
    float m_filteredValue;
    int m_minValue;
};
