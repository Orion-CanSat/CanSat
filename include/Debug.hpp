#ifndef __ORION_DEBUG_H__
#define __ORION_DEBUG_H__


#include <Orion/Utilities/IO/IO.hpp>
#include <Orion/Utilities/IO/TTY.hpp>
#include <Orion/Utilities/IO/FileOut.hpp>

/**
 * @brief Add trace log to serial and file
 * 
 * @param traceMessage Message to be included to the log
 */
void Trace(const char* traceMessage);

/**
 * @brief Add debug log to serial and file
 * 
 * @param debugMessage Message to be included to the log
 */
void Debug(const char* debugMessage);

/**
 * @brief Add Info log to serial and file
 * 
 * @param infoMessage Message to be included to the log
 */
void Info(const char* infoMessage);

/**
 * @brief Add Warn log to serial and file
 * 
 * @param warnMessage Message to be included to the log
 */
void Warn(const char* warnMessage);

/**
 * @brief Add Error log to serial and file
 * 
 * @param errorMessage Message to be included to the log
 */
void Error(const char* errorMessage);

/**
 * @brief Add Fatal log to serial and file
 * 
 * @param fatalMessage Message to be included to the log
 */
void Fatal(const char* fatalMessage);


#endif//__ORION_DEBUG_H__