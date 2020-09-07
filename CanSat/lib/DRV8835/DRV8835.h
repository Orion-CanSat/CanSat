#ifndef DRV8835_h
#define DRV8835_h

#include "Arduino.h"
#include "MotorPins.h"

class DRV8835{
private:
    MotorPins _a;
    MotorPins _b;
    short int _aPower = 0, _bPower = 0;
    unsigned char _max = 100;
public:
    DRV8835(MotorPins, MotorPins);
    void SetMax(unsigned char);
    void SetPower(short int, short int);
    short int GetAPower();
    short int GetBPower();
};

#endif