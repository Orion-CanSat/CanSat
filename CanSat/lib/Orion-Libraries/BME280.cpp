#include "Modules/DataModules/BME280.hpp"
#include "Utilities/Timeout.hpp"

#include <Adafruit_BME280.h>

using namespace Orion::Modules::DataModules;

bool BME280::InitBME280(void* bme)
{
    return (((Adafruit_BME280*)bme) ? ((Adafruit_BME280*)bme)->begin() : false);
}

BME280::BME280()
{
    _devicePtr = new Adafruit_BME280();
    _type = __BME280__;
    _initialized = Utilities::Timeout::WaitTimeout(BME280::InitBME280, _devicePtr, 3000);
}

bool BME280::HasDataType(uint32_t type)
{
    switch (type)
    {
        case __TEMPERATURE__:
        case __HUMIDITY__:
        case __PRESSURE__:
        case __ALTITUDE__:
            return true;
        default:
            return false;
    }
}

decPlace BME280::GetData(uint32_t type)
{
    switch (type)
    {
        case __TEMPERATURE__:
            return _temperature;
        case __HUMIDITY__:
            return _humidity;
        case __PRESSURE__:
            return _pressure;
        case __ALTITUDE__:
            return _altitude;
        default:
            return NAN;
    }
}

void BME280::Update()
{
    if (!_initialized)
        return;
    
    _temperature = (decPlace)(((Adafruit_BME280*)_devicePtr)->readTemperature());
    _humidity = (decPlace)(((Adafruit_BME280*)_devicePtr)->readHumidity());
    _pressure = (decPlace)(((Adafruit_BME280*)_devicePtr)->readPressure());
    _altitude = (decPlace)(((Adafruit_BME280*)_devicePtr)->readAltitude(1013.25));

    _timeOfLastUpdate = millis();
}

BME280::~BME280()
{
    _initialized = false;
    delete _devicePtr;
}