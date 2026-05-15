#pragma once
#include <Arduino.h>
#include "config.h"

// Enums
//@formatter: off
enum class ControllerMode : uint8_t { FX, CLASSIC, VPX, count }; // Meta Quest Pinball FX VR (keyboard), Meta Quest Pinball VR Classic (gamepad), Virtual Pinball X (keyboard)
//@formatter: on

// Function declarations
void setMode(ControllerMode newMode, bool initialConfig = false);
void sendGamepadReport();
