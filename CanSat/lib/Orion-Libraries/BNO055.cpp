#include "Modules/DataModules/BNO055.hpp"
#include "Utilities/Timeout.hpp"

#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

using namespace Orion::Modules::DataModules;

bool BNO055::InitBNO055(void* bno)
{
    return (((Adafruit_BNO055*)bno) ? ((Adafruit_BNO055*)bno)->begin() : false);
}

void BNO055::CalculateDisplacement()
{
    decPlace deltaT = (_timeOfLastUpdate - _lastTimer) / 1000;
    for (uint8_t i = 0; i < 3; i++)
        _displacement[i] += _fixedVelocity [i] * deltaT;
}

void BNO055::CalculateSpeed()
{
    decPlace deltaT = (_timeOfLastUpdate - _lastTimer) / 1000;
    for (uint8_t i = 0; i < 3; i++)
        _fixedVelocity[i] = _fixedLinearAcceleration[i] * deltaT;
}

void BNO055::NormalizeAcceleration()
{

}

BNO055::BNO055()
{
    _devicePtr = new Adafruit_BNO055();
    _type = __BNO055__;
    _initialized = Utilities::Timeout::WaitTimeout(BNO055::InitBNO055, _devicePtr, 3000);
}

bool BNO055::HasDataType(uint32_t type)
{
    switch (type)
    {
        case __ROTATIONAL_ANGLE__:
        case __ANGULAR_VELOCITY__:
        case __GRAVITATIONAL_ACCELERATION__:
        case __LINEAR_ACCELERATION__:
        case __LINEAR_VELOCITY__:
        case __LINEAR_DISPLACEMENT__:
        case __MAGNETISM__:
            return true;
        default:
            return false;
    }
}

decPlace BNO055::GetData(uint32_t type)
{
    type -= 1;
    uint32_t category = type - __AXIS__;
    uint32_t axis = category % 3;
    category -= axis;

    switch (category)
    {
        case __ROTATIONAL_ANGLE__:
            return _rotationalAngle[axis];
        case __ANGULAR_VELOCITY__:
            return _angularVelocity[axis];
        case __GRAVITATIONAL_ACCELERATION__:
            return _gravitationalAcceleration[axis];
        case __LINEAR_ACCELERATION__:
            return _fixedLinearAcceleration[axis];
        case __LINEAR_VELOCITY__:
            return _fixedVelocity[axis];
        case __MAGNETISM__:
            return _magnetism[axis];
        case __LINEAR_DISPLACEMENT__:
            return _displacement[axis];
        default:
            return NAN;
    }
}

void BNO055::Update()
{
    if (!_initialized)
        return;

    _lastTimer = _timeOfLastUpdate;

    imu::Vector<3> eulerAngles = ((Adafruit_BNO055*)_devicePtr)->getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> gyroscope = ((Adafruit_BNO055*)_devicePtr)->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> gravitationalAcceleration = ((Adafruit_BNO055*)_devicePtr)->getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    imu::Vector<3> linearAcceleration = ((Adafruit_BNO055*)_devicePtr)->getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> magnetism = ((Adafruit_BNO055*)_devicePtr)->getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    _timeOfLastUpdate = millis();

    for (uint8_t i = 0; i < 3; i++)
    {
        _rotationalAngle[i] = eulerAngles[i];
        _angularVelocity[i] = gyroscope[i];
        _gravitationalAcceleration[i] = gravitationalAcceleration[i];
        _linearAcceleration[i] = linearAcceleration[i];
        _magnetism[i] = magnetism[i];
    }

    NormalizeAcceleration();
    CalculateSpeed();
    CalculateDisplacement();
}

bool BNO055::Transmit(uint8_t* message, uint32_t len)
{
    if (len < 9)
        return false;

    if (message[3] != ' ' || message[7] != '=')
        return false;

    bool ret = false;
    char* mem = new char[len];
    char* memInit = mem;
    memcpy(mem, message, len);
    
    mem[3] = '\0';
    mem[7] = '\0';
    if (strcmp((const char*)mem, "set") == 0)
    {
        mem += 4;
        if (strcmp(mem, "dis") == 0)
        {
            mem += 4;
            decPlace val = (decPlace)atof(mem);
            for (uint8_t i = 0; i < 3; i++)
                _displacement[i] = val;

            ret = true;
        }
        else if (strcmp(mem, "vel") == 0)
        {
            mem += 4;
            decPlace val = (decPlace)atof(mem);
            for (uint8_t i = 0; i < 3; i++)
                _fixedVelocity[i] = val;
            
            ret = true;
        }
    }

    delete memInit;
    return ret;
}

BNO055::~BNO055()
{
    _initialized = false;
    delete _devicePtr;
}