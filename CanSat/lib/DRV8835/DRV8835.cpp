#include "DRV8835.h"

DRV8835::DRV8835(MotorPins a, MotorPins b) {
    this->_a = a;
    this->_b = b;
    pinMode(this->_a.GetModePin(), OUTPUT);
    pinMode(this->_b.GetModePin(), OUTPUT);
    pinMode(this->_a.GetPhasePin(), OUTPUT);
    pinMode(this->_b.GetPhasePin(), OUTPUT);
}

void DRV8835::SetMax(unsigned char max) {
    if (max > 100)
        max = 100;
    this->_max = max;
}

void DRV8835::SetPower(short int aPower, short int bPower) {
    if (aPower > this->_max)
        aPower = this->_max;
    else if (aPower < -this->_max)
        aPower = -this->_max;
    this->_aPower = aPower;

    if (bPower > this->_max)
        bPower = this->_max;
    else if (bPower < -this->_max)
        bPower = -this->_max;
    this->_bPower = bPower;

    if (this->_aPower < 0)
        digitalWrite(this->_a.GetModePin(), HIGH);
    else 
        digitalWrite(this->_a.GetModePin(), LOW);
    analogWrite(this->_a.GetPhasePin(), abs(this->_aPower) * 2);

    if (this->_bPower < 0)
        digitalWrite(this->_b.GetModePin(), HIGH);
    else 
        digitalWrite(this->_b.GetModePin(), LOW);
    analogWrite(this->_b.GetPhasePin(), abs(this->_aPower) * 2);
}

short int DRV8835::GetAPower() {
    return this->_aPower;
}

short int DRV8835::GetBPower() {
    return this->_bPower;
}