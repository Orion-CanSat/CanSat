#include "Modules/Module.hpp"

using namespace Orion::Modules;

Module::Module()
{

}

uint32_t Module::GetType()
{
    return _type;
}

bool Module::HasDataType(uint32_t type)
{
    return false;
}

decPlace Module::GetData(uint32_t type)
{
    return NAN;
}

void Module::Update()
{

}

uint32_t Module::GetLastUpdateTime()
{
    return _timeOfLastUpdate;
}

bool Module::Transmit(uint8_t* message, uint32_t len)
{
    return false;
}

bool Module::Receive(uint8_t* message, uint32_t* len)
{
    *len = 0;
    message[0] = '\0';
    return false;
}

#ifdef __TEENSY__
    void Module::AutoUpdateInterval(uint32_t interval)
    {
        if (interval != 0)
            {
                _updateInterval = interval;
                if (_threadID)
                    thrads.restart(_threadID);
                else
                    _threadID = threads.addThread(Module::AsuncUpdate, this);
            }
            else
            {
                if (_threadID != 0)
                    threads.suspend(_threadID);
            }
    }
#endif

Module::~Module()
{
    
}