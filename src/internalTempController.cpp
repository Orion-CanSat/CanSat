#include <wiring.h>

#include <Orion.h>

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

static Orion::Sensors::Sensor* _internalTempModule;

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

bool SetUpInternalTempController(Orion::Sensors::Sensor* internalTempModule)
{
    if (internalTempModule == nullptr)
        return false;
    _internalTempModule = internalTempModule;

    SetPanic(true);
    SetPanicCallback(TemperaturePanic);

    return true;
}


float GetInternalTemperature()
{
    _internalTempModule->Update();
    return _internalTempModule->GetDataType(__TEMPERATURE__);
}


int32_t RunTemperatureCheck()
{
    bool changeOccured = false;
    float temperature = GetInternalTemperature();

    /**
     * Check if internal temperature module failed to initialize
     * and then return the ERROR int value
     */
    if (temperature == NAN)
        return -1;

    /**
     * Overheating 
     */
    if (temperature > 90)
    {
        float sum = 0;
        const int pickSampleNum = 3;
        /**
         * Make sure that the cansat is overheating or the value meassured was just a spike
         */
        for (int i = 0; i < pickSampleNum; i++)
        {
            sum += GetInternalTemperature();
            delay(100);
        }

        /**
         * Slow down to slowest speed possible
         */
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