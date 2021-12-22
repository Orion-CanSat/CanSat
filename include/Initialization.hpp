#ifndef __ORION_INITIALIZATION_H__
#define __ORION_INITIALIZATION_H__

#include <Orion/Sensors/Sensor.hpp>
#include <Orion/Sensors/BME280.hpp>
#include <Orion/Sensors/BNO055.hpp>
#include <Orion/Sensors/TeensyChipTemperature.hpp>
#include <Orion/Sensors/GPS/Coordinates.hpp>
#include <Orion/Sensors/GPS/GPS.hpp>
#include <Orion/Sensors/GPS/MTK3339.hpp>

/**
 * Initializes all the Pins for later use
 */
void InitializePins();

/**
 * Initialize all the Sensors for the CanSat
 * @param bme280 The BME280 pointer
 * @param bno055 The BNO055 pointer
 * @param mtk3339 The MTK3339 pointer
 * @param chipTemperature The Internal Teensy Chip Temperature Module
 * @returns True if Initialization was successful and false if not
 */
bool InitializeDevices(Orion::Sensors::Sensor* bme280,
                       Orion::Sensors::Sensor* bno055,
                       Orion::Sensors::Sensor* mtk3339,
                       Orion::Sensors::Sensor* chipTemperature);

#endif//__ORION_INITIALIZATION_H__