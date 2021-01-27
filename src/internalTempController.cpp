#include "wiring.h"

#include "Orion.h"
#include "Orion/Data/Temperature.hpp"
#include "Orion/Utilities/Memory/unique_ptr.hpp"

#include "internalTempController.hpp"

extern "C" uint32_t set_arm_clock(uint32_t frequency);

static int32_t speeds[] = {
    24000000,
    150000000,
    396000000,
    450000000,
    528000000,
    600000000
};

static int currentIndex = 5;
static int speedsLength = 6;

static Orion::Modules::DataModules::TeensyChipTemperature* _internalTempModule;

void TemperaturePanic()
{
    pinMode(13, OUTPUT);
    delay(30);
    digitalWrite(13, HIGH);
    delay(30);
    cli();
    while (true)
        __asm__ __volatile__("nop");
}

bool SetUpInternalTempController(Orion::Modules::DataModules::TeensyChipTemperature* internalTempModule)
{
    if (internalTempModule == nullptr)
        return false;
    _internalTempModule = internalTempModule;

    SetPanic(true);
    SetPanicCallback(TemperaturePanic);

    return true;
}


decPlace GetInternalTemperature()
{
    _internalTempModule->Update();
    return _internalTempModule->GetData(__TEMPERATURE__);
}


int32_t RunTemperatureCheck()
{
    bool changeOccured = false;
    decPlace temperature = GetInternalTemperature();
    if (temperature == NAN)
        return -1;

    if (temperature > 90)
    {
        double sum = 0;
        const int pickSampleNum = 3;
        for (int i = 0; i < pickSampleNum; i++)
        {
            sum += GetInternalTemperature();
            delay(100);
        }

        if (sum / pickSampleNum > 90)
        {
            set_arm_clock(speeds[0]);
            Panic();
        }
    }
    else if (temperature > 75)
    {
        currentIndex = 0;
        changeOccured = true;
    }
    else if ((temperature > 65) && (currentIndex > 0))
    {
        currentIndex--;
        changeOccured = true;
    }
    else if ((temperature < 60) && (currentIndex + 1 < speedsLength))
    {
        currentIndex++;
        changeOccured = true;
    }

    if (changeOccured)
        set_arm_clock(speeds[currentIndex]);

    return speeds[currentIndex];
}