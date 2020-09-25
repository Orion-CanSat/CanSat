#include "Utilities/Timeout.hpp"

#include <Arduino.h>

using namespace Orion::Utilities;

bool Timeout::WaitTimeout(bool(*func)(void*), void* ptr, uint32_t duration)
{
    bool last = false;
    uint32_t startTime = millis();
    while (!last && startTime + duration > millis())
        last = (*func)(ptr);
    return last;
}

bool Timeout::WaitTimeout(bool(*func)(), uint32_t duration)
{
    bool last = false;
    uint32_t startTime = millis();
    while (!last && startTime + duration > millis())
        last = (*func)();
    return last;
}

void Timeout::WaitTimeout(void(*func)(void*), void* ptr, uint32_t duration)
{
    uint32_t startTime = millis();
    while (startTime + duration > millis())
        (*func)(ptr);
}

void Timeout::WaitTimeout(void(*func)(), uint32_t duration)
{
    uint32_t startTime = millis();
    while (startTime + duration > millis())
        (*func)();
}

void* Timeout::WaitTimeout(void*(*func)(void*), void* ptr, uint32_t duration)
{
    void* last = nullptr;
    uint32_t startTime = millis();
    while (!last && startTime + duration > millis())
        last = (*func)(ptr);
    return last;
}

void* Timeout::WaitTimeout(void*(*func)(), uint32_t duration)
{
    void* last = nullptr;
    uint32_t startTime = millis();
    while (!last && startTime + duration > millis())
        last = (*func)();
    return last;
}