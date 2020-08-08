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


#ifndef NAN
#pargma warning Your compiler/platform does not support the use of NAN
#define NAN {0}
#endif

#if defined(__arm__) && defined(CORE_TEENSY)
    #define __TEENSY__
    #if defined(__AT90USB162__)
        #define __TEENSY_1_0__
    #elif defined(__ATMEGA32U4__)
        #define __TEENSY_2_0__
    #elif defined(__MKL26Z64__)
        #define __TEENSY_LC__
    #elif defined(__MK20DX128__)
        #define __TEENSY_3_0__
    #elif defined(__MK20DX256__)
        #define __TEENSY_3_1__
        #define __TEENSY_3_2__
    #elif defined(__MK64FX512__)
        #define __TEENSY_3_5__
    #elif defined(__MK66FX1M0__)
        #define __TEENSY_3_6__
    #elif defined(__IMXRT1062__)
        #if defined(BUILTIN_SDCARD)
            #define __TEENSY_4_1__
        #else
            #define __TEENSY_4_0__
        #endif
    #endif
#endif


#define __BME280__ 0x000001
#define __BNO055__ 0x000002
#define __SD__ 0x000101
#define __GPS__ 0x0010000
#define __MTK3339__ 0x010001
#define __CAMERA__ 0x010100
#define __OV5642__ 0x010101

#if defined(__BME280__)
    #include <Adafruit_BME280.h>
#endif

#if defined(__BNO055__)
    #include <Adafruit_BNO055.h>
    #include <utility/imumaths.h>
#endif

#if defined(__SD__)
#include <SD.h>
#endif

#if defined(__MTK3339__)
    #include <Adafruit_GPS.h>
#endif

#if defined(__TEENSY__)
    #include <TeensyThreads.h>
#endif

#include <SoftwareSerial.h>

#define __MAX__TIMEOUT__FUNCTION__ 10000


#define __TEMPERATURE__ 0x000001
#define __HUMIDITY__ 0x000002
#define __PRESSURE__ 0x000003
#define __ALTITUDE__ 0x000004

#define __ROTATIONAL_ANGLE__ 0x000005
#define __ROTATIONAL_ANGLE_X__ 0x000006
#define __ROTATIONAL_ANGLE_Y__ 0x000007
#define __ROTATIONAL_ANGLE_Z__ 0x000008

#define __ANGULAR_VELOCITY__ 0x000009
#define __ANGULAR_VELOCITY_X__ 0x00000A
#define __ANGULAR_VELOCITY_Y__ 0x00000B
#define __ANGULAR_VELOCITY_Z__ 0x00000C

#define __GRAVITATIONAL_ACCELERATION__ 0x00000D
#define __GRAVITATIONAL_ACCELERATION_X__ 0x00000E
#define __GRAVITATIONAL_ACCELERATION_Y__ 0x00000F
#define __GRAVITATIONAL_ACCELERATION_Z__ 0x000010

#define __LINEAR_ACCELERATION__ 0x000011
#define __LINEAR_ACCELERATION_X__ 0x000012
#define __LINEAR_ACCELERATION_Y__ 0x000013
#define __LINEAR_ACCELERATION_Z__ 0x000014

#define __LINEAR_VELOCITY__ 0x000015
#define __LINEAR_VELOCITY_X__ 0x000016
#define __LINEAR_VELOCITY_Y__ 0x000017
#define __LINEAR_VELOCITY_Z__ 0x000018

#define __LINEAR_DISPLACEMENT__ 0x00000019
#define __LINEAR_DISPLACEMENT_X__ 0x00001A
#define __LINEAR_DISPLACEMENT_Y__ 0x00001B
#define __LINEAR_DISPLACEMENT_Z__ 0x00001C

#define __MAGNETISM__ 0x00001D
#define __MAGNETISM_X__ 0x00001E
#define __MAGNETISM_Y__ 0x00001F
#define __MAGNETISM_Z__ 0x000020

#define __LATITUDE__ 0x000021
#define __LONGITUDE__ 0x000022


#define __AXIS__ 0x10000
#define __X_AXIS__ 0x10001
#define __Y_AXIS__ 0x10002
#define __Z_AXIS__ 0x10003


namespace Orion
{
    struct noBaseClass{} nbc;

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

        template<typename T>
        struct Vector
        {
            T* _vector;
            uint16_t _size;
            bool _initialized = false;
            bool Initialize(uint16_t size)
            {
                if (_initialized) this->Free();
                _vector = (T*)malloc(size * sizeof(T));
                if (!_vector) return false;
                _size = size;
                _initialized = true;
                return true;
            }

            void Free()
            {
                if (_initialized) return;
                free(_vector);
                _initialized = false;
            }

            static bool Add(Vector<T>& result, Vector<T>& v1, Vector<T>& v2)
            {
                if ((!v1._initialized) || (!v2._initialized) || v1._size != v2._size) return false;
                if ((!result._initialized) || result._size != v1._size)
                {
                    result.Free();
                    if (!result.Initialize(v1._size)) return false;
                }
                for (uint16_t i = 0; i < v1._size; i++)
                {
                    result._vector[i] = v1._vector[i] + v2._vector[i];
                }
                return true;
            }

            static bool Subtract(Vector<T>& result, Vector<T>& v1, Vector<T>& v2)
            {
                if ((!v1._initialized) || (!v2._initialized) || v1._size != v2._size) return false;
                if ((!result._initialized) || result._size != v1._size)
                {
                    result.Free();
                    if (!result.Initialize(v1._size)) return false;
                }
                for (uint16_t i = 0; i < v1._size; i++)
                {
                    result._vector[i] = v1._vector[i] - v2._vector[i];
                }
                return true;
            }

            static bool Inverse(Vector<T>& result, Vector<T>& v)
            {
                if (!v._initialized) return false;
                if ((!result._initialized) || result._size != v._size)
                {
                    result.Free();
                    if (!result.Initialize(v._size)) return false;
                }
                for (uint16_t i = 0; i < v._size; i++)
                {
                    result._vector[i] = -v._vector[i];
                }
                return true;
            }

            static bool Multiply(T& result, Vector<T>& v1, Vector<T>& v2)
            {
                if ((!v1._initialized) || (!v2._initialized) || v1._size != v2._size) return false;
                result = v1._vector[0] * v2._vector[0];
                for (uint16_t i = 1; i < v1._size; i++) result += v1._vector[i] + v2._vector[i];
                return true;
            }
        };

        template<typename T>
        struct Matrix
        {
            T** _matrix;
            uint16_t _rows;
            uint16_t _columns;
            bool _initialized = false;

            bool Initialize(uint16_t rows, uint16_t columns)
            {
                if (_initialized) this->Free();
                _matrix = (T**)malloc(rows * sizeof(T*));
                if (!_matrix) return false;
                uint16_t i;
                for (i = 0; i < rows; i++)
                {
                    _matrix[i] = (T*)malloc(columns * sizeof(T*));
                    if (!_matrix[i])
                    {
                        for (uint16_t j = 0; j < i; j++)
                            free(_matrix[j]);
                        free(_matrix);
                        return false;
                    }
                }
                _rows = rows;
                _columns = columns;
                _initialized = true;
                return true;
            }

            void Free()
            {
                if (_initialized) return;
                for (uint16_t i = 0; i < _rows; i++)
                    free(_matrix[i]);
                free(_matrix);
                _initialized = false;
            }

            static bool Multiply(Matrix<T>& result, Matrix<T>& m1, Matrix<T>& m2)
            {
                if (m1._columns != m2._rows) return false;
                if ((!result._initialized) || result._rows != m1._rows || result._columns != m2._columns)
                {
                    if (!result.Initialize(m1._rows, m2._columns)) return false;
                }
                for (uint16_t i = 0; i < m1._rows; i++)
                {
                    for (uint16_t j = 0; j < m2._columns; j++)
                    {
                        result._matrix[i][j] = 0;
                        for (uint16_t k = 0; k < m1._columns; k++)
                            result._matrix[i][j] += m1._matrix[i][k] * m2._matrix[k][j];
                    }
                }
                return true;
            }
            static bool Multiply(Vector<T>& result, Matrix<T>& m, Vector<T>& v)
            {
                if ((!m._initialized) || (!v._initialized) || (m._columns != v._size)) return false;
                if ((!result._initialized) || result._size != v._size)
                {
                    if (!result.Initialize(v._size)) return false;
                }
                for (uint16_t i = 0; i < m._rows; i++)
                {
                    result._vector[i] = m._matrix[i][0] * v._vector[0];
                    for (uint16_t j = 1; j < m._columns; j++)
                    {
                        result._vector[i] += m._matrix[i][j] * v._vector[j];
                    }
                }
                return true;
            }
        };
    }

    namespace Modules
    {
        namespace DataModules
        {
            class Module
            {
            protected:
                void* _devicePtr = nullptr;
                uint32_t _timeOfLastUpdate = 0;
                uint32_t _updateInterval;

                #if defined(__TEENSY__)
                static void AsyncUpdate(Module* module)
                {
                    while (true)
                    {
                        module->Update();
                        delay(module->_updateInterval);
                    }
                }
                #endif
            public:
                bool _initState = false;
                Module() { }
                Module(noBaseClass) { }

                virtual uint32_t GetType() { return 0x00; }
                virtual bool HasDataType(uint32_t type) { return false; }
                virtual double GetData(uint32_t type) { return .0f; }
                
                virtual void Update() { }
                #if defined(__TEENSY__)
                void AutoUpdateInterval(uint32_t interval)
                {
                    _updateInterval = interval;
                    threads.addThread(Module::AsyncUpdate, this);
                }
                #endif

                uint32_t GetLastUpdateTime() { return _timeOfLastUpdate; }

                virtual bool Transmit(uint32_t* message, uint32_t size) { return false; }
                virtual uint32_t* Receive(int32_t timout = -1) { return nullptr; }
                virtual ~Module() { }
            };

            #if defined(__BME280__)
                class BME280 : virtual public Module
                {
                private:
                    double _temperature = NAN, _humidity = NAN, _pressure = NAN, _altitude = NAN;
                public:
                    static bool InitBME280(void* bme)
                    {
                        return (((Adafruit_BME280*)bme) ? ((Adafruit_BME280*)bme)->begin() : false);
                    }

                    BME280() : Module(nbc)
                    {
                        _devicePtr = new Adafruit_BME280();
                        _initState = Utilities::Timeout::WaitTimeout(BME280::InitBME280, _devicePtr, __MAX__TIMEOUT__FUNCTION__);
                    }

                    uint32_t GetType() { return __BME280__; }
                    bool HasDataType(uint32_t type)
                    {
                        switch(type)
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
                                return _temperature;
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

                    void Update()
                    {
                        if (_initState)
                        {
                            _temperature = (double)(((Adafruit_BME280*)_devicePtr)->readTemperature());
                            _humidity = (double)(((Adafruit_BME280*)_devicePtr)->readHumidity());
                            _pressure = (double)(((Adafruit_BME280*)_devicePtr)->readPressure());
                            _altitude = (double)(((Adafruit_BME280*)_devicePtr)->readAltitude(1013.25));
                        }
                        else
                        {
                            _temperature = .0f;
                            _humidity = .0f;
                            _pressure = .0f;
                            _altitude = .0f;
                        }
                        _timeOfLastUpdate = millis();
                    }
                };
            #endif

            #if defined(__BNO055__)
                class BNO055 : virtual public Module
                {
                private:
                    double _rotationalAngleX = .0f, _rotationalAngleY = .0f, _rotationalAngleZ = .0f;
                    double _angularVelocityX = .0f, _angularVelocityY = .0f, _angularVelocityZ = .0f;
                    double _gravitationalAccelerationX = .0f, _gravitationalAccelerationY = .0f, _gravitationalAccelerationZ = .0f;
                    double _linearAccelerationX = .0f, _linearAccelerationY = .0f, _linearAccelerationZ = .0f;
                    double _normalizedAccelerationX = .0f, _normalizedAccelerationY = .0f, _normalizedAccelerationZ = .0f;
                    double _magnetismX = .0f, _magnetismY = .0f, _magnetismZ = .0f;
                    double _velocityX = .0f, _velocityY = .0f, _velocityZ = .0f;
                    double _displacementX = .0f, _displacementY = .0f, _displacementZ = .0f;
                    uint32_t _timerLast = 0;
                    Utilities::Matrix<double> RotationMatrix;
                    Utilities::Vector<double> NNAcceleration, NAcceleration;
                    
                    void CalculateSpeed()
                    {
                        _normalizedAccelerationX = _linearAccelerationX;
                        _normalizedAccelerationY = _linearAccelerationY;
                        _normalizedAccelerationZ = _linearAccelerationZ;
                        double deltaT = (_timeOfLastUpdate - _timerLast) / 1000.0;
                        _displacementX += _velocityX * deltaT * 1.0 + (_normalizedAccelerationX * deltaT * deltaT * .5);
                        _displacementY += _velocityY * deltaT * 1.0 + (_normalizedAccelerationY * deltaT * deltaT * .5);
                        _displacementZ += _velocityZ * deltaT * 1.0 + (_normalizedAccelerationZ * deltaT * deltaT * .5);
                        _velocityX += _normalizedAccelerationX * deltaT;
                        _velocityY += _normalizedAccelerationY * deltaT;
                        _velocityZ += _normalizedAccelerationZ * deltaT;
                    }

                    void NormalizeAcceleration()
                    {
                        if (!RotationMatrix._initialized || !NNAcceleration._initialized || !NAcceleration._initialized) return;

                        double phi = _rotationalAngleX * 0.01745329251;
                        double theta = _rotationalAngleY * 0.01745329251;
                        double psi = _rotationalAngleZ * 0.01745329251;

                        RotationMatrix._matrix[0][0] = cos(psi) * cos(theta);
                        RotationMatrix._matrix[0][1] = cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi);
                        RotationMatrix._matrix[0][2] = sin(phi) * sin(psi) * cos(phi) * cos(psi) * sin(theta);

                        RotationMatrix._matrix[1][0] = cos(theta) * sin(psi);
                        RotationMatrix._matrix[1][1] = cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta);
                        RotationMatrix._matrix[1][2] = cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi);

                        RotationMatrix._matrix[2][0] = -sin(theta);
                        RotationMatrix._matrix[2][1] = cos(theta) * sin(phi);
                        RotationMatrix._matrix[2][2] = cos(phi) * cos(theta);

                        NNAcceleration._vector[0] = _linearAccelerationX;
                        NNAcceleration._vector[1] = _linearAccelerationY;
                        NNAcceleration._vector[2] = _linearAccelerationZ;

                        Utilities::Matrix<double>::Multiply(NAcceleration, RotationMatrix, NNAcceleration);

                        _normalizedAccelerationX = NAcceleration._vector[0];
                        _normalizedAccelerationY = NAcceleration._vector[1];
                        _normalizedAccelerationZ = NAcceleration._vector[2];
                    }

                public:
                    static bool InitBNO055(void* bno)
                    {
                        return (((Adafruit_BNO055*)bno) ? ((Adafruit_BNO055*)bno)->begin() : false);
                    }

                    BNO055() : Module(nbc)
                    {
                        _devicePtr = new Adafruit_BNO055();
                        _initState = Utilities::Timeout::WaitTimeout(BNO055::InitBNO055, _devicePtr, __MAX__TIMEOUT__FUNCTION__);
                        RotationMatrix.Initialize(3, 3);
                        NNAcceleration.Initialize(3);
                        NAcceleration.Initialize(3);
                    }

                    uint32_t GetType() { return __BNO055__; }
                    bool HasDataType(uint32_t type)
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
                    double GetData(uint32_t type)
                    {
                        switch (type)
                        {
                            case __ROTATIONAL_ANGLE_X__:
                                return _rotationalAngleX;
                            case __ROTATIONAL_ANGLE_Y__:
                                return _rotationalAngleY;
                            case __ROTATIONAL_ANGLE_Z__:
                                return _rotationalAngleZ;
                            case __ANGULAR_VELOCITY_X__:
                                return _angularVelocityX;
                            case __ANGULAR_VELOCITY_Y__:
                                return _angularVelocityY;
                            case __ANGULAR_VELOCITY_Z__:
                                return _angularVelocityZ;
                            case __GRAVITATIONAL_ACCELERATION_X__:
                                return _gravitationalAccelerationX;
                            case __GRAVITATIONAL_ACCELERATION_Y__:
                                return _gravitationalAccelerationY;
                            case __GRAVITATIONAL_ACCELERATION_Z__:
                                return _gravitationalAccelerationZ;
                            case __LINEAR_ACCELERATION_X__:
                                return _linearAccelerationX;
                            case __LINEAR_ACCELERATION_Y__:
                                return _linearAccelerationY;
                            case __LINEAR_ACCELERATION_Z__:
                                return _linearAccelerationZ;
                            case __LINEAR_VELOCITY_X__:
                                return _velocityX;
                            case __LINEAR_VELOCITY_Y__:
                                return _velocityY;
                            case __LINEAR_VELOCITY_Z__:
                                return _velocityZ;
                            case __LINEAR_DISPLACEMENT_X__:
                                return _displacementX;
                            case __LINEAR_DISPLACEMENT_Y__:
                                return _displacementY;
                            case __LINEAR_DISPLACEMENT_Z__:
                                return _displacementZ;
                            case __MAGNETISM_X__:
                                return _magnetismX;
                            case __MAGNETISM_Y__:
                                return _magnetismY;
                            case __MAGNETISM_Z__:
                                return _magnetismZ;
                            default:
                                return .0f;
                        }
                    }

                    void Update()
                    {
                        _timerLast = _timeOfLastUpdate;
                        if (_initState)
                        {
                            imu::Vector<3> euler = ((Adafruit_BNO055*)_devicePtr)->getVector(Adafruit_BNO055::VECTOR_EULER);
                            imu::Vector<3> gyro = ((Adafruit_BNO055*)_devicePtr)->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
                            imu::Vector<3> grav = ((Adafruit_BNO055*)_devicePtr)->getVector(Adafruit_BNO055::VECTOR_GRAVITY);
                            imu::Vector<3> linAccel = ((Adafruit_BNO055*)_devicePtr)->getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
                            imu::Vector<3> magn = ((Adafruit_BNO055*)_devicePtr)->getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
                            _rotationalAngleX = euler.x();
                            _rotationalAngleY = euler.y();
                            _rotationalAngleZ = euler.z();
                            _angularVelocityX = gyro.x();
                            _angularVelocityY = gyro.y();
                            _angularVelocityZ = gyro.z();
                            _gravitationalAccelerationX = grav.x();
                            _gravitationalAccelerationY = grav.y();
                            _gravitationalAccelerationZ = grav.z();
                            _linearAccelerationX = linAccel.x();
                            _linearAccelerationY = linAccel.y();
                            _linearAccelerationZ = linAccel.z();
                            _magnetismX = magn.x();
                            _magnetismY = magn.y();
                            _magnetismZ = magn.z();
                        }
                        else
                        {
                            _rotationalAngleX = .0f;
                            _rotationalAngleY = .0f;
                            _rotationalAngleZ = .0f;
                            _angularVelocityX = .0f;
                            _angularVelocityY = .0f;
                            _angularVelocityZ = .0f;
                            _gravitationalAccelerationX = .0f;
                            _gravitationalAccelerationY = .0f;
                            _gravitationalAccelerationZ = .0f;
                            _linearAccelerationX = .0f;
                            _linearAccelerationY = .0f;
                            _linearAccelerationZ = .0f;
                            _magnetismX = .0f;
                            _magnetismY = .0f;
                            _magnetismZ = .0f;
                        }
                        _timeOfLastUpdate = millis();
                        NormalizeAcceleration();
                        CalculateSpeed();
                    }
                };
            #endif
        }
        
        namespace GPSModules
        {
            typedef struct 
            {
                double _latitude = .0f;
                double _longitude = .0f;
                double _height = .0f;
            } Coordinates;
            
            class GPS : virtual public DataModules::Module
            {
            protected:
                Coordinates _coordinates;
            public:
                static double GetDistance(Coordinates& a, Coordinates& b)
                {
                    const unsigned int R = 6371e3;
                    const double phi1 = a._latitude * 0.01745329251;
                    const double phi2 = b._latitude * 0.01745329251;
                    const double deltaPhi = (b._latitude - a._latitude) * 0.01745329251;
                    const double deltaLambda = (b._longitude - a._longitude) * 0.01745329251;

                    const double alpha = pow(sin(deltaPhi / 2), 2.0) + cos(phi1) * cos(phi2) * pow(sin(deltaLambda / 2), 2.0);
                    const double c = 2 * atan2(sqrt(alpha), sqrt(1 - alpha));

                    return R * c;
                }

                GPS() { }
                GPS(noBaseClass) { }

                uint32_t GetType() { return __GPS__; }
                bool HasDataType(uint32_t type)
                {
                    switch (type)
                    {
                    case __ALTITUDE__:
                    case __LONGITUDE__:
                    case __LATITUDE__:
                    case __LINEAR_DISPLACEMENT__:
                        return true;
                    default:
                        return false;
                    }
                }
                double GetData(uint32_t type)
                {
                    switch (type)
                    {
                    case __LONGITUDE__:
                        return _coordinates._longitude;
                    case __LATITUDE__:
                        return _coordinates._latitude;
                    case __ALTITUDE__:
                        return _coordinates._height;
                    default:
                        return .0f;
                    }
                }
        
                virtual void Update() { }
                
                virtual bool Transmit(uint32_t* message, uint32_t size) { return false; }
                virtual uint32_t* Receive(int32_t timout = -1) { return nullptr; }

                virtual ~GPS() { }
            };

            #if defined(__MTK3339__)
            class MTK3339 : virtual public GPS
            {
            public:
                MTK3339(HardwareSerial* serial) : GPS(nbc)
                {
                    _devicePtr = new Adafruit_GPS(serial);
                    ((Adafruit_GPS*)_devicePtr)->begin(9600);
                    ((Adafruit_GPS*)_devicePtr)->sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
                    ((Adafruit_GPS*)_devicePtr)->sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
                    ((Adafruit_GPS*)_devicePtr)->sendCommand(PGCMD_ANTENNA);
                    delay(300);
                    serial->println(PMTK_Q_RELEASE);
                    _initState = true;
                }

                void Update()
                {
                    if (!_initState) return;
                    if (((Adafruit_GPS*)_devicePtr)->newNMEAreceived() && !((Adafruit_GPS*)_devicePtr)->parse(((Adafruit_GPS*)_devicePtr)->lastNMEA())) return;
                    if (((Adafruit_GPS*)_devicePtr)->fix)
                    {
                        _coordinates._latitude = ((Adafruit_GPS*)_devicePtr)->latitude;
                        _coordinates._longitude = ((Adafruit_GPS*)_devicePtr)->longitude;
                        _coordinates._height = ((Adafruit_GPS*)_devicePtr)->altitude;
                    }
                }
            };
            #endif
        }
    
        namespace Camera
        {
            class Camera : virtual public DataModules::Module
            {
            protected:
                char* (*namingFunc)();

            public:
                Camera(char* (*nmFunc)()) : Module(nbc) { namingFunc = nmFunc; }
                Camera(char* (*nmFunc)(), noBaseClass)  : Module(nbc) { namingFunc = nmFunc; }

                uint32_t GetType() { return __CAMERA__; }
                bool HasDataType(uint32_t type) { return false; }
                double GetData(uint32_t type) { return NAN; }

                virtual void Update();

                virtual ~Camera() { }                
            };

            #if defined(__OV5642__)
            class OV5642 : virtual public Camera
            {
            
            };
            #endif
        }
    }
    
    namespace Data
    {
        class Data
        {
        protected:
            Modules::DataModules::Module* _module;
        public:
            Data() { }
            Data(noBaseClass) { }

            virtual uint32_t GetType() { return 0x00; }
            virtual double GetData(uint32_t selector) { return .0f; }

            virtual ~Data() { }
        };

        class Temperature : virtual public Data
        {
        public:
            Temperature(Modules::DataModules::Module* module) : Data(nbc)
            {
                if (module->HasDataType(__TEMPERATURE__))
                    _module = module;
                else
                    _module = nullptr;
            }

            uint32_t GetType() { return __TEMPERATURE__; }
            double GetData(uint32_t selector) { return ((_module) ? _module->GetData(__TEMPERATURE__) : .0f); }
        };

        class Humidity : virtual public Data
        {
        public:
            Humidity(Modules::DataModules::Module* module) : Data(nbc)
            {
                if (module->HasDataType(__HUMIDITY__))
                    _module = module;
                else
                    _module = nullptr;
            }

            uint32_t GetType() { return __HUMIDITY__; }
            double GetData(uint32_t selector) { return ((_module) ? _module->GetData(__HUMIDITY__) : .0f); }
        };

        class Pressure : virtual public Data
        {
        public:
            Pressure(Modules::DataModules::Module* module) : Data(nbc)
            {
                if (module->HasDataType(__PRESSURE__))
                    _module = module;
                else
                    _module = nullptr;
            }

            uint32_t GetType() { return __PRESSURE__; }
            double GetData(uint32_t selector) { return ((_module) ? _module->GetData(__PRESSURE__) : .0f); }
        };

        class Altitude : virtual public Data
        {
        public:
            Altitude(Modules::DataModules::Module* module) : Data(nbc)
            {
                if (module->HasDataType(__ALTITUDE__))
                    _module = module;
                else
                    _module = nullptr;
            }

            uint32_t GetType() { return __ALTITUDE__; }
            double GetData(uint32_t selector) { return ((_module) ? _module->GetData(__ALTITUDE__) : .0f); }
        };

        class RotationalAngle : virtual public Data
        {
        public:
            RotationalAngle(Modules::DataModules::Module* module) : Data(nbc)
            {
                if (module->HasDataType(__ROTATIONAL_ANGLE__))
                    _module = module;
                else
                    _module = nullptr;
            }

            uint32_t GetType() { return __ROTATIONAL_ANGLE__; }
            double GetData(uint32_t selector) { return ((_module) ? _module->GetData(__ROTATIONAL_ANGLE__ + selector - __AXIS__) : .0f); }
        };

        class AngularVelocity : virtual public Data
        {
        public:
            AngularVelocity(Modules::DataModules::Module* module) : Data(nbc)
            {
                if (module->HasDataType(__ANGULAR_VELOCITY__))
                    _module = module;
                else
                    _module = nullptr;
            }

            uint32_t GetType() { return __ANGULAR_VELOCITY__; }
            double GetData(uint32_t selector) { return ((_module) ? _module->GetData(__ANGULAR_VELOCITY__ + selector - __AXIS__) : .0f); }
        };

        class GravitationalAcceleration : virtual public Data
        {
        public:
            GravitationalAcceleration(Modules::DataModules::Module* module) : Data(nbc)
            {
                if (module->HasDataType(__GRAVITATIONAL_ACCELERATION__))
                    _module = module;
                else
                    _module = nullptr;
            }

            uint32_t GetType() { return __GRAVITATIONAL_ACCELERATION__; }
            double GetData(uint32_t selector) { return ((_module) ? _module->GetData(__GRAVITATIONAL_ACCELERATION__ + selector - __AXIS__) : .0f); }
        };

        class LinearAcceleration : virtual public Data
        {
        public:
            LinearAcceleration(Modules::DataModules::Module* module) : Data(nbc)
            {
                if (module->HasDataType(__LINEAR_ACCELERATION__))
                    _module = module;
                else
                    _module = nullptr;
            }

            uint32_t GetType() { return __LINEAR_ACCELERATION__; }
            double GetData(uint32_t selector) { return ((_module) ? _module->GetData(__LINEAR_ACCELERATION__ + selector - __AXIS__) : .0f); }
        };

        class LinearVelocity : virtual public Data
        {
        public:
            LinearVelocity(Modules::DataModules::Module* module) : Data(nbc)
            {
                if (module->HasDataType(__LINEAR_VELOCITY__))
                    _module = module;
                else
                    _module = nullptr;
            }

            uint32_t GetType() { return __LINEAR_VELOCITY__; }
            double Get(uint32_t selector) { return ((_module) ? _module->GetData(__LINEAR_VELOCITY__ + selector - __AXIS__) : .0f); }
        };

        class LinearDisplacement : virtual public Data
        {
        public:
            LinearDisplacement(Modules::DataModules::Module* module) : Data(nbc)
            {
                if (module->HasDataType(__LINEAR_DISPLACEMENT__))
                    _module = module;
                else
                    _module = nullptr;
            }

            uint32_t GetType() { return __LINEAR_DISPLACEMENT__; }
            double GetData(uint32_t selector) { return ((_module) ? _module->GetData(__LINEAR_DISPLACEMENT__ + selector - __AXIS__) : .0f); }
        };

        class Magnetism : virtual public Data
        {
        public:
            Magnetism(Modules::DataModules::Module* module) : Data(nbc)
            {
                if (module->HasDataType(__MAGNETISM_X__) && module->HasDataType(__MAGNETISM_Y__) && module->HasDataType(__MAGNETISM_Z__))
                    _module = module;
                else
                    _module = nullptr;
            }

            uint32_t GetType() { return __MAGNETISM__; }
            double GetData(uint32_t selector) { return ((_module) ? _module->GetData(__MAGNETISM__ + selector - __AXIS__) : .0f); }
        };
    }
}


void HALT(char* params, uint8_t length)
{
    if (length == 0)
        return;
    uint32_t timeToHALT = 0;
    if (!sscanf(params, "%zu", &timeToHALT) && length == 4)
        timeToHALT = *((uint32_t*)params);
    delay(timeToHALT);
}

uint8_t buzzerPin = 4;
bool buzzerState = false;
void BUZZER(char* params, uint8_t length)
{
    if (length == 0)
        buzzerState = !buzzerState;
    else if (length == 1)
        buzzerState = params[0];
    digitalWrite(buzzerPin, buzzerState);
}

void (*functionArray[])(char* params, uint8_t length) = {
    HALT,
    BUZZER
};

void RunOP(char* bcode, uint8_t length)
{
    uint16_t func = *((uint16_t*)bcode);
    functionArray[func]((char*)((int)bcode + 2), length - 2);
}

#include <RH_RF95.h>

Orion::Modules::DataModules::Module* bme;
Orion::Modules::DataModules::Module* bno;
Orion::Modules::GPSModules::GPS* gps;

Orion::Data::Data* pressure;
Orion::Data::Data* temperature;
Orion::Data::Data* humidity;
Orion::Data::Data* altitude;
Orion::Data::Data* gravitationalAcceleration;
Orion::Data::Data* magnetism;
Orion::Data::Data* rotationalAngles;
Orion::Data::Data* angularVelocity;
Orion::Data::Data* linearAcceleration;
Orion::Data::Data* linearVelocity;
Orion::Data::Data* linearDisplacement;

RH_RF95* rfm = new RH_RF95(4, 3);
bool rfmInit = false;

float data[29];

uint32_t time = 0;
uint32_t timeOfLastPacketSent = 0;

char* radioPacket = (char*)malloc(240 * sizeof(char));
char* sdPacket = (char*)malloc(1024 * sizeof(char));

HardwareSerial* gpsSerial;
File fptr;

void setup()
{
    gpsSerial = &Serial3;

    bme = new Orion::Modules::DataModules::BME280();
    bno = new Orion::Modules::DataModules::BNO055();
    gps = new Orion::Modules::GPSModules::MTK3339(gpsSerial);

    if (rfm->init() && rfm->setFrequency(433.3))
    {
        rfm->setTxPower(23, false);
        rfmInit = true;
    }

    bme->AutoUpdateInterval(50);
    bno->AutoUpdateInterval(10);
    gps->AutoUpdateInterval(10);

    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, 1);

    pressure = new Orion::Data::Pressure(bme);
    temperature = new Orion::Data::Temperature(bme);
    humidity = new Orion::Data::Humidity(bme);
    altitude = new Orion::Data::Altitude(bme);
    gravitationalAcceleration = new Orion::Data::GravitationalAcceleration(bno);
    magnetism = new Orion::Data::Magnetism(bno);
    rotationalAngles = new Orion::Data::RotationalAngle(bno);
    angularVelocity = new Orion::Data::AngularVelocity(bno);
    linearAcceleration = new Orion::Data::LinearAcceleration(bno);
    linearVelocity = new Orion::Data::LinearVelocity(bno);
    linearDisplacement = new Orion::Data::LinearDisplacement(bno);

    SD.begin(BUILTIN_SDCARD);
}

void loop()
{
    time = millis();
    data[0] = (float)time;
    data[1] = (float)pressure->GetData(0);
    data[2] = (float)temperature->GetData(0);
    data[3] = (float)humidity->GetData(0);
    data[4] = (float)altitude->GetData(0);
    data[5] = (float)gravitationalAcceleration->GetData(__X_AXIS__);
    data[6] = (float)gravitationalAcceleration->GetData(__Y_AXIS__);
    data[7] = (float)gravitationalAcceleration->GetData(__Z_AXIS__);
    data[8] = (float)magnetism->GetData(__X_AXIS__);
    data[9] = (float)magnetism->GetData(__Y_AXIS__);
    data[10] = (float)magnetism->GetData(__Z_AXIS__);
    data[11] = (float)rotationalAngles->GetData(__X_AXIS__);
    data[12] = (float)rotationalAngles->GetData(__Y_AXIS__);
    data[13] = (float)rotationalAngles->GetData(__Z_AXIS__);
    data[14] = (float)angularVelocity->GetData(__X_AXIS__);
    data[15] = (float)angularVelocity->GetData(__Y_AXIS__);
    data[16] = (float)angularVelocity->GetData(__Z_AXIS__);
    data[17] = (float)linearAcceleration->GetData(__X_AXIS__);
    data[18] = (float)linearAcceleration->GetData(__Y_AXIS__);
    data[19] = (float)linearAcceleration->GetData(__Z_AXIS__);
    data[20] = (float)linearVelocity->GetData(__X_AXIS__);
    data[21] = (float)linearVelocity->GetData(__Y_AXIS__);
    data[22] = (float)linearVelocity->GetData(__Z_AXIS__);
    data[23] = (float)linearDisplacement->GetData(__X_AXIS__);
    data[24] = (float)linearDisplacement->GetData(__Y_AXIS__);
    data[25] = (float)linearDisplacement->GetData(__Z_AXIS__);
    data[26] = (float)gps->GetData(__LONGITUDE__);
    data[27] = (float)gps->GetData(__LATITUDE__);
    data[28] = (float)gps->GetData(__ALTITUDE__);

    fptr = SD.open("Data.dat");
    for (int i = 0; i < sizeof(data); i++)
        radioPacket[i] = ((char*)&data)[i];
    radioPacket[sizeof(data)] = '\0';

    for (int i = 0; i < sizeof(data) / sizeof(data[0]); i++)
    {
        Serial.print(data[i]);
        Serial.print(" ");
    }
    Serial.println();

    if (fptr)
    {
        snprintf(sdPacket,
            1024,
            "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f %f %f %f %f %f\n",
            data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14],
            data[15], data[16], data[17], data[18], data[19], data[20], data[21], data[22], data[23], data[24], data[25], data[26], data[27], data[28], data[29]);
        fptr.write(sdPacket, strlen(sdPacket));
        fptr.close();
    }

    if (Serial.available())
    {
        String message = "  ";
        uint8_t length = message.length();

        if (length >= 2)
            RunOP((char*)message.c_str(), length);
    }
    if (rfm->available())
    {
        char* message;
        uint8_t length;
        rfm->recv((uint8_t*)message, &length);

        if (length >= 2)
            RunOP(message, length);
    }
    if (rfmInit && timeOfLastPacketSent + 50 < time)
    {
        if (rfm->send((uint8_t*)radioPacket, sizeof(data) + 1))
            rfm->waitPacketSent();
        timeOfLastPacketSent = time;
    }
}
