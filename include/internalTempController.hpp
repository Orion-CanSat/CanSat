#ifndef __ORION_INTERNAL_TEMP_CONTROLLER_H__
#define __ORION_INTERNAL_TEMP_CONTROLLER_H__


#include "Orion.h"
#include "Orion/Utilities/Memory/shared_ptr.hpp"
#include "Orion/Utilities/Memory/unique_ptr.hpp"
#include "Orion/Modules/DataModules/TeensyChipTemperature.hpp"
#include "Orion/Data/Temperature.hpp"


// Sets up the Internal Temperature Controller.
bool SetUpInternalTempController(Orion::Modules::DataModules::TeensyChipTemperature* internalTempModule);
// Allows access to the Internal Processing Unit Temperature.
decPlace GetInternalTemperature();
// Will adjust the speed of the Processing Unit to maintain a temperature acceptable (Read DOCS)
int32_t RunTemperatureCheck();


#endif//__ORION_INTERNAL_TEMP_CONTROLLER_H__