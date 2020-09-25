#ifndef __ORION_MODULES_DATAMODULES_BNO055_H__
#define __ORION_MODULES_DATAMODULES_BNO055_H__

#include "Orion.h"
#include "Modules/Module.hpp"

#include <stdint.h>

namespace Orion
{
    namespace Modules
    {
        namespace DataModules
        {
            class BNO055 : virtual public Module
            {
                private:
                    decPlace _rotationalAngle[3] = { NAN, NAN, NAN };
                    decPlace _angularVelocity[3] = { NAN, NAN, NAN };
                    decPlace _gravitationalAcceleration[3] = { NAN, NAN, NAN };
                    decPlace _linearAcceleration[3] = { NAN, NAN, NAN };
                    decPlace _fixedLinearAcceleration[3] = { NAN, NAN, NAN };
                    decPlace _fixedVelocity[3] = { NAN, NAN, NAN };
                    decPlace _magnetism[3] = { NAN, NAN, NAN };
                    decPlace _displacement[3] = { 0, 0, 0 };
                    uint32_t _lastTimer = 0;
                
                    static bool InitBNO055(void* bno);

                    void CalculateDisplacement();
                    void CalculateSpeed();
                    void NormalizeAcceleration();
                
                public:
                    BNO055();

                    bool HasDataType(uint32_t type);
                    decPlace GetData(uint32_t type);
                    void Update();
                    bool Transmit(uint8_t* message, uint32_t len);

                    ~BNO055();
            };
        }
    }
}

#endif//__ORION_MODULES_DATAMODULES_BNO055_H__