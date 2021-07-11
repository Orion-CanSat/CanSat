#include "Initialization.hpp"

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

void InitializePins() {
    pinMode(LED_BUILTIN, OUTPUT);
    SPI.begin();
    Wire.begin();
}