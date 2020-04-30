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


#define __BME280__ 0x0001
#define __BNO055__ 0x0002

#if defined(__BME280__)
    #include <Adafruit_BME280.h>
#endif
#if defined(__BNO055__)
    #include <Adafruit_BNO055.h>
    #include <utility/imumaths.h>
#endif

#include <TeensyThreads.h>


#define __MAX__TIMEOUT__FUNCTION__ 10000


#define __TEMPERATURE__ 0x0001
#define __HUMIDITY__ 0x0002
#define __PRESSURE__ 0x0003
#define __ALTITUDE__ 0x0004

#define __ROTATIONAL_ANGLE__ 0x0005
#define __ROTATIONAL_ANGLE_X__ 0x0006
#define __ROTATIONAL_ANGLE_Y__ 0x0007
#define __ROTATIONAL_ANGLE_Z__ 0x0008

#define __ANGULAR_VELOCITY__ 0x0009
#define __ANGULAR_VELOCITY_X__ 0x000A
#define __ANGULAR_VELOCITY_Y__ 0x000B
#define __ANGULAR_VELOCITY_Z__ 0x000C

#define __GRAVITATIONAL_ACCELERATION__ 0x000D
#define __GRAVITATIONAL_ACCELERATION_X__ 0x000E
#define __GRAVITATIONAL_ACCELERATION_Y__ 0x000F
#define __GRAVITATIONAL_ACCELERATION_Z__ 0x0010

#define __LINEAR_ACCELERATION__ 0x0011
#define __LINEAR_ACCELERATION_X__ 0x0012
#define __LINEAR_ACCELERATION_Y__ 0x0013
#define __LINEAR_ACCELERATION_Z__ 0x0014

#define __LINEAR_VELOCITY__ 0x0015
#define __LINEAR_VELOCITY_X__ 0x0016
#define __LINEAR_VELOCITY_Y__ 0x0017
#define __LINEAR_VELOCITY_Z__ 0x0018

#define __LINEAR_DISPLACMENT__ 0x0019
#define __LINEAR_DISPLACMENT_X__ 0x001A
#define __LINEAR_DISPLACMENT_Y__ 0x001B
#define __LINEAR_DISPLACMENT_Z__ 0x001C

#define __MAGNETISM__ 0x001D
#define __MAGNETISM_X__ 0x001E
#define __MAGNETISM_Y__ 0x001F
#define __MAGNETISM_Z__ 0x0020


#define __X_AXIS__ 0x01
#define __Y_AXIS__ 0x02
#define __Z_AXIS__ 0x03


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

        class Vector;
        class Matrix;
        class Vector
        {
        private:
            int32_t *_vec;
            uint32_t _size;

        public:
            Vector(int32_t size)
            {
                _vec = (int32_t*)malloc(_size * sizeof(int32_t));
                _size = size;
            }

            int32_t& operator[](const uint32_t possition)
            {
                return _vec[possition];
            }

            friend class Matrix;

            inline Vector operator+(Vector& v)
            {
                uint32_t n = this->_size;
                Vector* res = new Vector(n);
                for (uint32_t i = 0; i < n; i++) res->_vec[i] = this->_vec[i] + v._vec[i];
                return *res;
            }

            inline Vector operator-()
            {
                Vector v = Vector(_size);
                uint32_t n = this->_size;
                for (uint32_t i = 0; i < n; i++) v._vec[i] = - this->_vec[i];
                return v;
            }

            inline Vector operator-(Vector& v)
            {
                //if (this->_size != v._size) assert(false);
                uint32_t n = this->_size;
                Vector* res = new Vector(n);
                for (uint32_t i = 0; i < n; i++) res->_vec[i] = this->_vec[i] - v._vec[i];
                return *res;
            }

            inline int32_t operator*(Vector &v)
            {
                //if (this->_size != v._size) assert(false);
                uint32_t n = this->_size;
                int32_t res = 0;
                for(uint32_t i = 0; i < n; i++) res += this->_vec[i] * v._vec[i];
                return res;
            }

            inline Vector operator*(Matrix& m);

            template<typename T> const inline Vector operator*(const T& a) const //a is a number
            {
                uint32_t n = this->_size;
                Vector* res = new Vector(n);
                for (uint32_t i = 0; i < n; i++) res->_vec[i] = this->_vec[i] * a;
                return *res;
            }
        };

        class Matrix
        {
        private:
            int32_t** _mat = nullptr;
            uint32_t _rows, _columns;

        public:
            Matrix(uint32_t rows, uint32_t columns)
            {
                _rows = rows; _columns = columns;
                _mat = (int32_t**)malloc(_rows * sizeof(int32_t*));
                for (int i = 0; i < _rows; i++)
                    _mat[i] = (int32_t*)malloc(_columns * sizeof(int32_t));
            }

            class Proxy {
            public:
                Proxy(int32_t* _array) : _array(_array) { }

                int32_t& operator[](const uint32_t index) {
                    return _array[index];
                }

            private:
                int32_t* _array;
            };

            Proxy operator[](const int32_t index) {
                return Proxy(_mat[index]);
            }

            friend class Vector;

            inline Matrix operator+(Matrix& m) //!DOESNT WORK
            {
                uint32_t r = this->_rows, c = this->_columns;
                Matrix* res = new Matrix(r, c);
                for(uint32_t i = 0; i < r; i++){
                    for(uint32_t j = 0; j < c; j++){
                        res->_mat[i][j] = this->_mat[i][j] +  m._mat[i][j];
                    }
                }
                return *res;
            }

            inline Matrix operator-() //!DOESN'T WORK!!!
            {
                uint32_t r = _rows, c = _columns;
                Matrix res = Matrix(r, c);
                for (uint32_t i = 0; i < r; i++){
                    for(uint32_t j = 0; j < c; j++){
                        res._mat[i][j] = - this->_mat[i][j];
                    }
                }
                return res;
            }

            inline Matrix operator-(Matrix& m) //!DOESN"T WORK
            {
                uint32_t r = this->_rows, c = this->_columns;
                Matrix* res = new Matrix(r, c);
                for(uint32_t i = 0; i < r; i++){
                    for(uint32_t j = 0; j < c; j++){
                        res->_mat[i][j] = this->_mat[i][j] -  m._mat[i][j];
                    }
                }
                return *res;
            }

            inline Matrix operator*(Matrix& m)
            {
                uint32_t r = this->_rows, c = m._columns, n = this->_columns;
                Matrix* res = new Matrix(r, c);
                for(uint32_t i = 0; i < r; i++){
                    for(uint32_t j = 0; j < c; j++){
                        res->_mat[i][j] = 0;
                        for(uint32_t k = 0; k < n; k++){
                            res->_mat[i][j] += this->_mat[i][k] * m._mat[k][j];
                        }
                    }
                }
                return *res;
            }

            inline Vector operator*(Vector& v){
                uint32_t r = this->_rows, c = this->_columns;
                if(c == v._size){
                    Vector* ans = new Vector(r);
                    for(uint32_t i = 0; i < r; i++){
                        ans->_vec[i] = 0;
                        for(uint32_t k = 0; k < c; k++){
                            ans->_vec[i] += this->_mat[i][k] * v._vec[k];
                        }
                    }
                    return *ans;
                }
                else if(r == v._size){
                    Vector* ans = new Vector(c);
                    for(uint32_t i = 0; i < c; i++){
                        ans->_vec[i] = 0;
                        for(uint32_t k = 0; k < r; k++){
                            ans->_vec[i] += this->_mat[k][i] * v._vec[k];
                        }
                    }
                    return *ans;
                }
            }

            template<typename T> const inline Matrix operator*(const T& a) const
            {
                uint32_t r = this->_rows, c = this->_columns;
                Matrix* res = new Matrix(r, c);
                for(uint32_t i = 0; i < r; i++){
                    for(uint32_t j = 0; j < c; j++){
                        res->_mat[i][j] = this->_mat[i][j] * a;
                    }
                }
                return *res;
            }
        };

        inline Vector Vector::operator*(Matrix& m)
        {
            return m * (*this);
        }

        template<typename T>
        inline Vector operator*(const T lhs, const Vector& rhs)
        {
            return rhs * lhs;
        }

        template<typename T>
        inline Matrix operator*(const T lhs, const Matrix& rhs)
        {
                return rhs * lhs;
        }
    }

    namespace Modules
    {
        struct noBaseClass{} nbc;

        class Module
        {
        protected:
            void* _devicePtr = nullptr;
            bool _initState = false;
            uint32_t _timeOfLastUpdate = 0;
        public:
            Module() { }
            Module(noBaseClass) { }

            virtual uint32_t GetType() { return 0x00; }
            virtual bool HasDataType(uint32_t type) { return false; }
            virtual double GetData(uint32_t type) { return .0f; }
            
            virtual void Update() { }
            virtual void AutoUpdateInterval(uint32_t interval) { }

            uint32_t GetLastUpdateTime() { return _timeOfLastUpdate; }

            virtual bool Transmit(uint32_t* message, uint32_t size) { return false; }
            virtual uint32_t* Receive(int32_t timout = -1) { return nullptr; }
            virtual ~Module() { }
        };

        #if defined(__BME280__)
            class BME280 : virtual public Module
            {
            private:
                double _temperature = .0f, _humidity = .0f, _pressure = .0f, _altitude = .0f;
            public:
                static bool InitBME280(void* bme)
                {
                    return (((Adafruit_BME280*)bme) ? ((Adafruit_BME280*)bme)->begin() : false);
                }
                static void AsyncUpdate(BME280* Bme)
                {
                    while (true)
                    {
                        Bme->Update();
                        delay(100);
                    }
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
                void AutoUpdateInterval(uint32_t interval)
                {
                    threads.addThread(BME280::AsyncUpdate, this);
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
                double _displacmentX = .0f, _displacmentY = .0f, _displacmentZ = .0f;
                uint32_t _timerLast = 0;
                
                void CalculateSpeed()
                {
                    _normalizedAccelerationX = _linearAccelerationX;
                    _normalizedAccelerationY = _linearAccelerationY;
                    _normalizedAccelerationZ = _linearAccelerationZ;
                    double deltaT = (_timeOfLastUpdate - _timerLast) / 1000.0;
                    _velocityX += _normalizedAccelerationX * deltaT;
                    _velocityY += _normalizedAccelerationY * deltaT;
                    _velocityZ += _normalizedAccelerationZ * deltaT;
                    _displacmentX += _velocityX * deltaT * 1.0 + (_normalizedAccelerationX * deltaT * deltaT * .5);
                    _displacmentY += _velocityY * deltaT * 1.0 + (_normalizedAccelerationY * deltaT * deltaT * .5);
                    _displacmentZ += _velocityZ * deltaT * 1.0 + (_normalizedAccelerationZ * deltaT * deltaT * .5);
                }

            public:
                static bool InitBNO055(void* bno)
                {
                    return (((Adafruit_BNO055*)bno) ? ((Adafruit_BNO055*)bno)->begin() : false);
                }
                static void AsyncUpdate(BNO055* Bno)
                {
                    while (true)
                    {
                        Bno->Update();
                        delay(10);
                    }
                }

                BNO055() : Module(nbc)
                {
                    _devicePtr = new Adafruit_BNO055();
                    _initState = Utilities::Timeout::WaitTimeout(BNO055::InitBNO055, _devicePtr, __MAX__TIMEOUT__FUNCTION__);
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
                        case __LINEAR_DISPLACMENT__:
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
                        case __LINEAR_DISPLACMENT_X__:
                            return _displacmentX;
                        case __LINEAR_DISPLACMENT_Y__:
                            return _displacmentY;
                        case __LINEAR_DISPLACMENT_Z__:
                            return _displacmentZ;
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
                    CalculateSpeed();
                }
                void AutoUpdateInterval(uint32_t interval)
                {
                    threads.addThread(BNO055::AsyncUpdate, this);
                }
            };
        #endif
    }
    
    namespace Data
    {
        class Temperature
        {
        private:
            Modules::Module* _module;
        public:
            Temperature(Modules::Module* module)
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
            Modules::Module* _module;
        public:
            Humidity(Modules::Module* module)
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
            Modules::Module* _module;
        public:
            Pressure(Modules::Module* module)
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
            Modules::Module* _module;
        public:
            Altitude(Modules::Module* module)
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

        class RotationalAngle
        {
        private:
            Modules::Module* _module;
        public:
            RotationalAngle(Modules::Module* module)
            {
                if (module->HasDataType(__ROTATIONAL_ANGLE__))
                    _module = module;
                else
                    _module = nullptr;
            }

            double Get(uint8_t axis)
            {
                return ((_module) ? _module->GetData(__ROTATIONAL_ANGLE__ + axis) : .0f);
            }
        };

        class AngularVelocity
        {
        private:
            Modules::Module* _module;
        public:
            AngularVelocity(Modules::Module* module)
            {
                if (module->HasDataType(__ANGULAR_VELOCITY__))
                    _module = module;
                else
                    _module = nullptr;
            }

            double Get(uint8_t axis)
            {
                return ((_module) ? _module->GetData(__ANGULAR_VELOCITY__ + axis) : .0f);
            }
        };

        class GravitationalAcceleration
        {
        private:
            Modules::Module* _module;
        public:
            GravitationalAcceleration(Modules::Module* module)
            {
                if (module->HasDataType(__GRAVITATIONAL_ACCELERATION__))
                    _module = module;
                else
                    _module = nullptr;
            }

            double Get(uint8_t axis)
            {
                return ((_module) ? _module->GetData(__GRAVITATIONAL_ACCELERATION__ + axis) : .0f);
            }
        };

        class LinearAcceleration
        {
        private:
            Modules::Module* _module;
        public:
            LinearAcceleration(Modules::Module* module)
            {
                if (module->HasDataType(__LINEAR_ACCELERATION__))
                    _module = module;
                else
                    _module = nullptr;
            }

            double Get(uint8_t axis)
            {
                return ((_module) ? _module->GetData(__LINEAR_ACCELERATION__ + axis) : .0f);
            }
        };

        class LinearVelocity
        {
        private:
            Modules::Module* _module;
        public:
            LinearVelocity(Modules::Module* module)
            {
                if (module->HasDataType(__LINEAR_VELOCITY__))
                    _module = module;
                else
                    _module = nullptr;
            }

            double Get(uint8_t axis)
            {
                return ((_module) ? _module->GetData(__LINEAR_VELOCITY__ + axis) : .0f);
            }
        };

        class LinearDisplacment
        {
        private:
            Modules::Module* _module;
        public:
            LinearDisplacment(Modules::Module* module)
            {
                if (module->HasDataType(__LINEAR_DISPLACMENT__))
                    _module = module;
                else
                    _module = nullptr;
            }

            double Get(uint8_t axis)
            {
                return ((_module) ? _module->GetData(__LINEAR_DISPLACMENT__ + axis) : .0f);
            }
        };

        class Magnetism
        {
        private:
            Modules::Module* _module;
        public:
            Magnetism(Modules::Module* module)
            {
                if (module->HasDataType(__MAGNETISM_X__) && module->HasDataType(__MAGNETISM_Y__) && module->HasDataType(__MAGNETISM_Z__))
                    _module = module;
                else
                    _module = nullptr;
            }

            double Get(uint8_t axis)
            {
                return ((_module) ? _module->GetData(__MAGNETISM_X__ + axis - 1) : .0f);
            }
        };
    }
}


void setup()
{

}

void loop()
{
}