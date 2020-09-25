#ifndef __ORION_MODULES_DATAMODULE_H__
#define __ORION_MODULES_DATAMODULE_H__

#include "Orion.h"

#include <stdint.h>

namespace Orion
{
    namespace Modules
    {
        class Module
        {
            protected:
                void* _devicePtr = nullptr;
                uint32_t _type = 0x00;
                uint32_t _timeOfLastUpdate = 0;
                int32_t _threadID = 0;
                bool _initialized = false;

                #ifdef __TEENSY__
                    uint32_t _updateInterval = 0;
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
                Module();
                
                uint32_t GetType();
                virtual bool HasDataType(uint32_t type);
                virtual decPlace GetData(uint32_t type);
                virtual void Update();
                uint32_t GetLastUpdateTime();
                virtual bool Transmit(uint8_t* message, uint32_t len);
                virtual bool Receive(uint8_t* message, uint32_t* len);

                #ifdef __TEENSY__
                    void AutoUpdateInterval(uint32_t interval);
                #endif

                virtual ~Module();
        };
    }
}

#endif//__ORION_MODULES_DATAMODULE_H__