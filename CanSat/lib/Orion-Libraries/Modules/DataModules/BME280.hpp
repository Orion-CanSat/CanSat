#ifndef __ORION_MODULES_DATAMODULES_BME280_H__
#define __ORION_MODULES_DATAMODULES_BME280_H__

#include "Orion.h"
#include "Modules/Module.hpp"

namespace Orion
{
    namespace Modules
    {
        namespace DataModules
        {
            class BME280 : virtual public Module
            {
                private:
                    decPlace _temperature = NAN, _humidity = NAN, _pressure = NAN, _altitude = NAN;
                    static bool InitBME280(void* bme);

                public:
                    BME280();

                    bool HasDataType(uint32_t type);
                    decPlace GetData(uint32_t type);
                    void Update();

                    ~BME280();
            };
        }
    }
}

#endif//__ORION_MODULES_DATAMODULES_BME280_H__