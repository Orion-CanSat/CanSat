#include "MotorPins.h"

MotorPins::MotorPins() {
    this->_modePin = 0;
    this->_phasePin = 0;
}

MotorPins::MotorPins(char modePin, char phasePin) {
    this->_phasePin = phasePin;
    this->_modePin = modePin;
}

void MotorPins::SetPins(char modePin, char phasePin) {
    this->_phasePin = phasePin;
    this->_modePin = modePin;
}

char MotorPins::GetModePin() {
    return this->_modePin;
}

char MotorPins::GetPhasePin() {
    return this->_phasePin;
}