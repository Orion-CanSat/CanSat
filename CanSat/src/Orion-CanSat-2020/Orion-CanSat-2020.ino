/**
 * Orion II CanSat 2020's Microprocessor's program
 * Copyright (C) 2020  richardbar, jbalatos, konstantinosk31
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 * 
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * richardbar:                  richard1996ba@gmail.com
 * jbalatos:                    jbalatos@gmail.com
 * konstantinosk31:             konstantinosk31@gmail.com
 */


//// B34D91192A8C352024468B673A460808                                                           // Check-sum md5 of the first region


#pragma region SENSOR_REGION
#define RFM9x
#define SDC
#define BME280
#define BNO055
#define GPS_Ultra
#define MOTORS
#define TSL2591                                                 // Will not use A TSL but will provide the
                                                                // necessary code for the TSL for others to use
#pragma endregion


#pragma region INCLUSE_REGION
#if defined(BME280)
    #include <Adafruit_BME280.h>
#endif

#include <TeensyThreads.h>
#include <SPI.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#pragma endregion


#pragma region DEBUG_REGION
#define DEBUG_MODE                                              // Define DEBUG MODE
                                                                // MUST be undefined or removed
                                                                // before launch
#if defined(DEBUG_MODE)
    /**
     * Will print to Serial the arguments given
     * 
     * !Warning!  Requires a valid Serial port to be opened.
     */
    #define DEBUG(...) { if (Serial) Serial.print(__VA_ARGS__); }

    /**
     * Will print to Serial the arguments given
     * plus a new line at the end
     * 
     * !Warning!  Requires a valid Serial port to be opened.
     */
    #define DEBUGLN(...) { if (Serial) Serial.println(__VA_ARGS__); }

#else
    /**
     * Will skip a DEBUG Function
     * 
     * !Warning!  This is the mode desired for the launch
     * !            Because it does not waste time.
     */
    #define DEBUG(...) { }
    #define DEBUGLN(...) { }

#endif
#pragma endregion


#pragma region MODULE_SPECIFIC_REGION
#if defined(BME280)
        #include <Adafruit_BME280.h>
#endif
#pragma endregion


#pragma region GENERAL_MODULE_REGION
#define MAX_TIMEOUT_FUNCTION 10000

#if defined(DEBUG_MODE)
        #define INIT_PAUSE 1000                                 // Time between inits
                                                                // TEST MODE or DEBUG MODE
                                                                // All defines must be removed
                                                                // before launch
#else
        #define INIT_PAUSE 4000                                 // Time between inits
#endif

#define SEA_LEVEL_PRESSURE_REF 1013.25                          // Air pressure at sea level
#pragma endregion


namespace Orion
{
    namespace Utilities
    {
        class Timeout
        {
        public:
            static bool WaitTimeout(bool(*func)(void*), void* ptr, uint32_t duration)
            {
                bool last = false;
                uint32_t startTime = millis();
                while (!last && startTime + duration > millis())
                    last = (*func)(ptr);
                return last;
            }
            static bool WaitTimeout(bool(*func)(), uint32_t duration)
            {
                bool last = false;
                uint32_t startTime = millis();
                while (!last && startTime + duration > millis())
                    last = (*func)();
                return last;
            }

            static void WaitTimeout(void(*func)(void*), void* ptr, uint32_t duration)
            {
                uint32_t startTime = millis();
                while (startTime + duration > millis())
                    (*func)(ptr);
            }
            static void WaitTimeout(void(*func)(), uint32_t duration)
            {
                uint32_t startTime = millis();
                while (startTime + duration > millis())
                    (*func)();
            }

            static void* WaitTimeout(void*(*func)(), uint32_t duration)
            {
                void* last;
                uint32_t startTime = millis();
                while (!last && startTime + duration > millis())
                    last = (*func)();
                return last;
            }
        };
    };

    #define __TEMPERATURE__ 0x0001
    #define __HUMIDITY__ 0x0002
    #define __PRESSURE__ 0x0003
    #define __ALTITUDE__ 0x0004

    struct noBaseClass{} nbc;

    class Module
    {
    public:
        Module() { }
        Module(noBaseClass) { }

        virtual uint32_t GetType() { return 0x00; }
        virtual bool HasDataType(uint32_t type) { return false; }
        virtual double GetData(uint32_t type) { return .0f; }
        
        virtual void GetNewData() { }
        virtual void AutoUpdateInterval(uint32_t interval) { }
    };

    //#if defined(BME280)
        #define __BME280__ 0x0001
        class BME280 : virtual public Module
        {
        private:
            Adafruit_BME280* _bme;
            bool _bmeInitState = false;
            double _temperature = .0f, _humidity = .0f, _pressure = .0f, _altitude = .0f;
        public:
            static bool InitBME280(void* bme)
            {
                return (((Adafruit_BME280*)bme) ? ((Adafruit_BME280*)bme)->begin() : false);
            }
            static void ForeverGetNewData(BME280* ptr, uint32_t sleepDuration)
            {
                while (true)
                {
                    ptr->GetNewData();
                    delay(sleepDuration);
                }
            }

            BME280() : Module(nbc)
            {
                _bme = new Adafruit_BME280();
                _bmeInitState = Utilities::Timeout::WaitTimeout(BME280::InitBME280, _bme, MAX_TIMEOUT_FUNCTION);
            }

            uint32_t GetType() { return __BME280__; }
            bool HasDataType(uint32_t type)
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
            double GetData(uint32_t type)
            {
                switch (type)
                {
                    case __TEMPERATURE__:
                        return _altitude;
                    case __HUMIDITY__:
                        return _humidity;
                    case __PRESSURE__:
                        return _pressure;
                    case __ALTITUDE__:
                        return _altitude;
                    default:
                        return .0f;
                }
            }

            void GetNewData()
            {
                if (_bmeInitState)
                {
                    _temperature = (double)_bme->readTemperature();
                    _humidity = (double)_bme->readHumidity();
                    _pressure = (double)_bme->readPressure();
                    _altitude = (double)_bme->readAltitude(1013.25);
                }
                else
                {
                    _temperature = .0f;
                    _humidity = .0f;
                    _pressure = .0f;
                    _altitude = .0f;
                }
            }

            void AutoUpdateInterval(uint32_t interval)
            {
                threads.addThread(BME280::ForeverGetNewData, this, interval);
            }
        };
    //#endif

    class Temperature
    {
    private:
        Module* _module;
    public:
        Temperature(Module* module)
        {
            if (module->HasDataType(__TEMPERATURE__))
                _module = module;
            else
                _module = nullptr;
        }

        double Get()
        {
            return ((_module) ? _module->GetData(__TEMPERATURE__) : .0f);
        }
    };

    class Humidity
    {
    private:
        Module* _module;
    public:
        Humidity(Module* module)
        {
            if (module->HasDataType(__HUMIDITY__))
                _module = module;
            else
                _module = nullptr;
        }

        double Get()
        {
            return ((_module) ? _module->GetData(__HUMIDITY__) : .0f);
        }
    };

    class Pressure
    {
    private:
        Module* _module;
    public:
        Pressure(Module* module)
        {
            if (module->HasDataType(__PRESSURE__))
                _module = module;
            else
                _module = nullptr;
        }

        double Get()
        {
            return ((_module) ? _module->GetData(__PRESSURE__) : .0f);
        }
    };

    class Altitude
    {
    private:
        Module* _module;
    public:
        Altitude(Module* module)
        {
            if (module->HasDataType(__ALTITUDE__))
                _module = module;
            else
                _module = nullptr;
        }

        double Get()
        {
            return ((_module) ? _module->GetData(__ALTITUDE__) : .0f);
        }
    };
}


#pragma region MODULE_REGION
Orion::Module* Bme
#pragma endregion


#pragma region Data_REGION
Orion::Temperature* Temperature;
Orion::Humidity* Humidity;
Orion::Pressure* Pressure;
Orion::Altitude* Altitude;
#pragma endregion


void setup()
{
    #if defined(DEBUG_MODE)
        Serial.begin(9600);
    #endif
    Bme = new Orion::BME280();
    Bme->AutoUpdateInterval(100);
    Temperature = new Orion::Temperature(Bme);
    Humidity = new Orion::Humidity(Bme);
    Pressure = new Orion::Pressure(Bme);
    Altitude = new Orion::Altitude(Bme);
}

void loop()
{
    DEBUG(millis());
    DEBUG(Temperature->Get());
    DEBUG(" ");
    DEBUG(Humidity->Get());
    DEBUG(" ");
    DEBUG(Pressure->Get());
    DEBUG(" ");
    DEBUGLN(Altitude->Get());
}