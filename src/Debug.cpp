#include "Debug.hpp"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static Orion::Utilities::IO::FileOut* fileLogger = NULL;

static bool isFileLoggerInitialized = false;

/**
 * @brief Initialize the IO_File logger
 * 
 */
static void InitializeFileLogger() {
    fileLogger = new Orion::Utilities::IO::FileOut("loggger");
    isFileLoggerInitialized = (fileLogger != NULL);
}

/**
 * @brief Output the message to the serial and to the file with the appropriate type and color
 * 
 * @param type Type of text
 * @param colorIndex Color of the serial
 * @param message Message to be put to the put to serial and file
 */
static void AddText(const char* type, Orion::Utilities::IO::ColorsIndex colorIndex, const char* message) {
    if (!isFileLoggerInitialized)
        InitializeFileLogger();

    uint32_t messageLength = strlen(message);
    uint32_t typeLength = strlen(type);
    uint32_t totalTypeLength = typeLength + 4;

    char* spaces = (char*)malloc((totalTypeLength + 1) * sizeof(char));
    memset(spaces, ' ', totalTypeLength * sizeof(char));
    spaces[totalTypeLength] = '\0';

    const char* delim = "\n";

    char* messageClone = (char*)malloc((messageLength + 1) * sizeof(char));
    memcpy(messageClone, message, (messageLength + 1) * sizeof(char));

    char* ptr = strtok(messageClone, delim);

    orionout << Orion::Utilities::IO::ResetTTY << "[" << Orion::Utilities::IO::FGColors[colorIndex] << type << Orion::Utilities::IO::ResetTTY << "]: " << Orion::Utilities::IO::FGColors[colorIndex] << ptr << Orion::Utilities::IO::ResetTTY << Orion::Utilities::IO::endl;
    (*fileLogger) << "[" << type << "]: " << ptr << Orion::Utilities::IO::endl;

    ptr = strtok(NULL, delim);

    while (ptr != NULL) {
        orionout << spaces << Orion::Utilities::IO::FGColors[colorIndex] << ptr << Orion::Utilities::IO::ResetTTY << Orion::Utilities::IO::endl;
        (*fileLogger) << spaces << ptr << Orion::Utilities::IO::endl;


        ptr = strtok(NULL, delim);
    }

    free(messageClone);
    free(spaces);
}


void Trace(const char* traceMessage) {
    AddText("Trace", Orion::Utilities::IO::ColorsIndex::BrightWhite, traceMessage);
}

void Debug(const char* debugMessage) {
    AddText("Debug", Orion::Utilities::IO::ColorsIndex::BrightYellow, debugMessage);
}

void Info(const char* infoMessage) {
    AddText("Info", Orion::Utilities::IO::ColorsIndex::BrightBlue, infoMessage);
}

void Warn(const char* warnMessage) {
    AddText("Warn", Orion::Utilities::IO::ColorsIndex::BrightMagenta, warnMessage);
}

void Error(const char* errorMessage) {
    AddText("Error", Orion::Utilities::IO::ColorsIndex::BrightRed, errorMessage);
}

void Fatal(const char* fatalMessage) {
    AddText("Fatal", Orion::Utilities::IO::ColorsIndex::Red, fatalMessage);
}