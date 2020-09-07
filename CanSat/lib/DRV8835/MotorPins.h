#ifndef MOTORPINS_h
#define MOTORPINS_h

#include "Arduino.h"

class MotorPins {
private:
    char _phasePin = 0;
    char _modePin = 0;
public:
    MotorPins();
    MotorPins(char, char);
    void SetPins(char, char);
    char GetPhasePin();
    char GetModePin();
};

#endif