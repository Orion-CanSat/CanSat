#pragma once

#ifdef ARDUINO
    #include <Arduino.h>
#endif

#include <Orion/Orion.hpp>
#include <Orion/Sensors/Sensor.hpp>
#include <Orion/Sensors/TeensyChipTemperature.hpp>

// Sets up the Internal Temperature Controller.
bool SetUpInternalTempController(Orion::Sensors::Sensor* internalTempModule);
// Allows access to the Internal Processing Unit Temperature.
float GetInternalTemperature();
// Will adjust the speed of the Processing Unit to maintain a temperature acceptable (Read DOCS)
int32_t RunTemperatureCheck();