//-------------------------------------------------------------------------------------------------------------------------------------------------
// Generated by Make me a Makefile (https://github.com/Orion-CanSat/Make-me-a-Makefile)
//
// Board:            Teensy 4.1
// CPU Speed:        600000000
// Optimization:     -O3
// 
// Time of creation: 2021/01/27
//-------------------------------------------------------------------------------------------------------------------------------------------------

#include "main.hpp"

#include "Arduino.h"

#include "Orion.h"

#include "Orion/Utilities/Memory/shared_ptr.hpp"
#include "Orion/Utilities/Time/Delay.hpp"

#include "Orion/Modules/DataModules/TeensyChipTemperature.hpp"


#include "internalTempController.hpp"


Orion::Utilities::Memory::shared_ptr<Orion::Modules::DataModules::TeensyChipTemperature> _teensyChipTemperature =
    new Orion::Modules::DataModules::TeensyChipTemperature();

int main(void)
{
    Serial.begin(9600);

    if (!SetUpInternalTempController(_teensyChipTemperature.get()))
    {

    }

    while (true)
    {
        Serial.print(GetInternalTemperature());
        Serial.print(" ");
        Serial.println(RunTemperatureCheck());
        Serial.flush();

        Orion::Utilities::Time::Delay::DelayS(1);
        yield();
    }
}

