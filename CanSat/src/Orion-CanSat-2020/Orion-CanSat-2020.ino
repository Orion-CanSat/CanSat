/**
 * Orion II CanSat 2020's Microprocessor's program
 * Copyright (C) 2020  richardbar, jbalatos, konstantinosk31
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 * 
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * richardbar:                  richard1996ba@gmail.com
 * jbalatos:                    jbalatos@gmail.com
 * konstantinosk31:             konstantinosk31@gmail.com
 */


//// 1FCA63E0834E0EC3ACC6E45FBD08C948                                                           // Check-sum md5 of the first include


#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_TSL2591.h>

#include <SD.h>

#include <SPI.h>
#include <Wire.h>

#include <utility/imumaths.h>


#pragma region SENSOR_REGION
//// #define SDC
//// #define BME
//// #define BNO
//// #define TSL
#pragma endregion


#pragma region BRIGHTNESS_TSL_MODULE_REGION
//// #define BRIGHTNESS_TSL_MAX
//// #define BRIGHTNESS_TSL_HIGH
//// #define BRIGHTNESS_TSL_MEDIUM
//// #define BRIGHTNESS_TSL_LOW


// !Warning!    Do not modify from here
// !            till end of region
#if defined(BRIGHTNESS_TSL_MAX)
        #define TSL_GAIN 0x00
        #define TSL_TIMMING 0x00
#elif defined(BRIGHTNESS_TSL_HIGH)
        #define TSL_GAIN 0x10
        #define TSL_TIMMING 0x02
#elif defined(BRIGHTNESS_TSL_MEDIUM)
        #define TSL_GAIN 0x20
        #define TSL_TIMMING 0x04
#elif defined(BRIGHTNESS_TSL_LOW)
        #define TSL_GAIN 0x30
        #define TSL_TIMMING 0x05
#elif defined(TSL)
        #error Please select TSL Brightness from BRIGHTNESS_TSL_MODULE_REGION
#endif

#pragma endregion


#pragma region DEBUG_REGION
//// #define DEBUG_MODE                                                                         // Define DEBUG MODE
                                                                                                // MUST be undefined or removed
                                                                                                // before launch
#if defined(DEBUG_MODE)

        /**
         * Will print to Serial the arguments given
         * 
         * !Warning!	Requires a valid Serial port to be opened.
         */
        #define DEBUG(...) { if (Serial) Serial.print(__VA_ARGS__); }

        /**
         * Will print to Serial the arguments given
         * plus a new line at the end
         * 
         * !Warning!	Requires a valid Serial port to be opened.
         */
        #define DEBUGLN(...) { if (Serial) Serial.println(__VA_ARGS__); }

#else

        /**
         * Will skip a DEBUG Function
         * 
         * !Warning!	This is the mode desired for the launch
         * !            Because it does not waste time.
         */
        #define DEBUG(...) { }
        #define DEBUGLN(...) { }

#endif

#pragma endregion


/**
 * GLOBAL CONSTANTS
 * Will be used for setup purposes
 */
#pragma region CONSTANTS_REGION
#define RF_FREQ 433.0                                                                           // RF Frequency for the
                                                                                                // RFM95 or 9x or 65
                                                                                                // !Warning!	Base's Frequency.
                                                                                                // Must be set to the same
                                                                                                // Frequency
                                                                                                // TODO: Change it to a different Frequency


#define RF_RST 9                                                                                // Reset pin number

#define RF_INT 2                                                                                // RF Interrupt pin
                                                                                                // Will Interrupt program'same
                                                                                                // execution if pin pulled HIGH
                                                                                                // 
                                                                                                // !Warning!	Must be Interrupt 
                                                                                                // !            Friendly.

#define RF_CS 9                                                                                 // RF's Chip Select pin
                                                                                                // Used for the SPI Protocol


#define SDC_CS BUILTIN_SDCARD                                                                   // SD card's Chip Select pin
                                                                                                // Used for the SPI Protocol

#define CAM_CS 20                                                                               // Arducam's Chip Select pin
                                                                                                // Used for the SPI Protocol


#define SCK_PIN 13                                                                              // SPI Clock

#define MOSI_PIN 11                                                                             // SPI Master out
                                                                                                // Slave In

#define MISO_PIN 12                                                                             // SPI Master In
                                                                                                // Slave Out

#define MAX_TIMEOUT_FUNCTION 1000

#if defined(DEBUG_MODE)
        #define INIT_PAUSE 2000                                                                 // Time between inits
                                                                                                // TEST MODE or DEBUG MODE
                                                                                                // All defines must be removed
                                                                                                // before launch
#else
        #define INIT_PAUSE 4000                                                                 // Time between inits
#endif

#define SEA_LEVEL_PRESSURE_REF 1013.25                                                          // Air pressure at sea level
#pragma endregion


#pragma region MODULE_STATE_REGION 
#if defined(SDC)                                                                                // Declaring all initialization
        bool Sdc_init_state = false;                                                            // status variables for GLOBAL
#endif                                                                                          // use in the functions

#if defined(BNO)
        bool Bno_init_state = false;
#endif

#if defined(BME)
        bool Bme_init_state = false;
#endif
                                                                                                // Beware: Must introduce an init
#if defined(TSL)                                                                                // function for each variable or
        bool Tsl_init_state = false;                                                            // else the module will not work
#endif                                                                                          // by default                                                                 
#pragma endregion


#pragma region MODULE_REGION
#if defined(BNO)
        Adafruit_BNO055 bno = Adafruit_BNO055(55);
#endif

#if defined(BME)
        Adafruit_BME280 bme;
#endif

#if defined (TSL)
        Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
#endif
#pragma endregion


#pragma region VARIABLE_REGION
char radiopacket[300];
#pragma endregion


/**
 * Creates a timeout function
 * Will stop function if exceeds time limit
 * 
 * !Warning!	Some functions can not be timed out.
 */
bool waitTimeout(bool(*func)(), uint32_t dur) {
        bool last = false;                                                                      // Create a state variable
        uint32_t start_time = millis();                                                         // Get current time
        while (!last && millis() < start_time + dur)                                            // Check if function has not ended or time has not exceeded time limit
                last = (*func)();                                                               // Execute function if hasn't started
        return last;                                                                            // Return function's output or false
}

void setup() {
        pinMode(SCK_PIN, OUTPUT);                                                               // Set the SPI Clock to OUTPUT
        pinMode(MOSI_PIN, OUTPUT);                                                              // Set the MOSI to Output
        pinMode(MISO_PIN, OUTPUT);                                                              // Set the MISO to Output

        #if (defined DEBUG_MODE)
                Serial.begin(11500);
                delay(INIT_PAUSE);
        #endif

        Wire.begin();                                                                           // Start I2C Protocol
        SPI.begin();                                                                            // Start SPI Protocol

        digitalWrite(SDC_CS, HIGH);                                                             // De-activates SD SPI
        digitalWrite(RF_CS, HIGH);                                                              // De-activates RF SPI
        digitalWrite(CAM_CS, HIGH);                                                             // De-activates Camera SPI

        delay(INIT_PAUSE);

        #if defined (SDC)
                if (Sdc_init_state = waitTimeout(InitSDC, MAX_TIMEOUT_FUNCTION)) {

                }
                else {

                }
        #endif

        #if defined(BME)
                if (Bme_init_state = waitTimeout(InitBME, MAX_TIMEOUT_FUNCTION)) {

                }
                else {
                
                }
        #endif

        #if defined(BNO)
                if (Bno_init_state = waitTimeout(InitBNO, MAX_TIMEOUT_FUNCTION)) {
                
                }
                else {
                
                }
        #endif

        #if defined(TSL)
                if (Tsl_init_state = waitTimeout(InitTSL, MAX_TIMEOUT_FUNCTION)) {

                }
                else {

                }
        #endif
}

void loop() {
        #if defined(BME)                                                                        // Check if BME sensor is
                                                                                                // defined to be used
                UseBME();                                                                       // Gather data from BME
        #endif
        #if defined(BNO)                                                                        // Check if BNO sensor is
                                                                                                // defined to be used
                UseBNO();                                                                       // Gather data from BNO
        #endif
        #if defined(TSL)                                                                        // Check if TSL sensor is
                                                                                                // defined to be used
                UseTSL();                                                                       // Gather data from TSL
        #endif
        
        // TODO: Move to Save Data function
        DEBUGLN(radiopacket);                                                                   // Print to screen if DEBUG is
                                                                                                // defined data collected

        radiopacket[0] = '\0';                                                                  // Empty data
}

/**
 * Get size of a pointer
 * 
 * !Warning! 	pointer must end with a null
 * !            character at the end.
 */
uint32_t FindSize(char* str) {
        uint32_t size = 0;
        while (str[size] != '\0') size++;                                                       // Check if array at index is null, end of array
                                                                                                // else increment index by one till it's the end
        return size;                                                                            // Return last index
}


#pragma region INITIALIZATION_REGION
#if defined(SDC)
        /**
         * initialize the SD card
         * 
         * !Warning!    Might end in an endless condition
         * !            Depends on microprocessor and SD
         * !            module used
         */
        bool InitSDC() {
                if (SD.begin(SDC_CS)) {                                                         // SD module/card initialized successfully
                        DEBUGLN("SD Init complete");                                            // Print that the SD module/card did
                                                                                                // initialize successfully
                        return true;
                }
                else {                                                                          // SD module/card did not initialize successfully
                        DEBUGLN("SD Init failed");                                              // Print that the SD module/card did
                                                                                                // not initialize successfully
                        return false;
                }
        }
#endif

#if defined(BME)
        /**
         * Initialize the BME sensor
         * 
         * !Warning!    BME will throw multible "BME Init failed"
         */
        bool InitBME() {
                if (bme.begin()) {                                                              // BNO  initialized successfully
                        DEBUGLN("BME Init complete");                                           // Print that BNO did initialize
                        return true;
                }
                else {                                                                          // BNO did not initialize successfully
                        DEBUGLN("BME Init faliled");                                            // Print that BME did not initialize
                        return false;
                }
        }
#endif

#if defined(BNO)
        /**
         * Initialize the BNO sensor
         * 
         * !Warning!    Might end in an endless condition
         * !            Depends on microprocessor and wiring
         */
        bool InitBNO() {
                if (bno.begin()) {                                                              // BNO initialized successfully
                        DEBUGLN("BNO Init complete");                                           // Print that BNO did initialize
                        return true;
                }
                else {                                                                          // BNO did not initialize successfully
                        DEBUGLN("BNO Init failed");                                             // Print that BNO did not initialize
                        return false;
                }
        }
#endif

#if defined(TSL)
        /**
         * Initialize the TSL sensor
         * 
         * !Warning!    Might end in an endless condition
         * !            Depends on microprocessor and wiring
         */
        bool InitTSL() {
                if (tsl.begin()) {                                                              // TSL initialized successfully
                        DEBUGLN("TSL Init complete");                                           // Print that TSL did initialize

                        tsl.setGain((tsl2591Gain_t)TSL_GAIN);                                   // Set TSL's Brightness settings
                        tsl.setTiming(TSL_TIMMING);                                             // !Warning!    The Dimmer the environment
                                                                                                // !            the longer it takes to take
                                                                                                // !            accurate measurements

                        return true;
                }
                else {                                                                          // TSL did not initialize successfully
                        DEBUGLN("TSL Init failed");                                             // Print that TSL did not initialize
                        return true;
                }
        }
#endif
#pragma endregion


#pragma region USE_REGION
#if defined(BME)
        /**
         * Collect Temperature, Pressure, Humidity
         * guesses Altitude and add it to the radio
         * packet in order for it to be sent to base
         * 
         * !Warning!    Bme must be initialized
         * 
         * ? Output Format:
         * ? (Temperature), (Pressure), (Humidity), (Altitude)
         */
        void UseBME() {
                if (!Bme_init_state) {
                        sprintf(radiopacket + strlen(radiopacket),                              // Add to radiopacket
                                "+ + + + ");                                                    // 4 pluses - no values
                        return;
                }

                                                                                                // Bme had a successful initialization
                sprintf(radiopacket + strlen(radiopacket),
                        "%.2lf %.2lf %.2lf %.1f ",                                              // Format decimal with two decimals
                        bme.readTemperature(),                                                  // Temperature
                        bme.readPressure(),                                                     // Pressure
                        bme.readHumidity(),                                                     // Humidity
                        bme.readAltitude(SEA_LEVEL_PRESSURE_REF));                              // Altitude
        }
#endif

#if defined(BNO)
        /**
         * Collect angles of rotation, angular velocity,
         * gravitational acceleration, linear acceleration,
         * magnetism and add it to the radio packet in order
         * for it to be sent to base
         * 
         * !Warning!    Bno must be initialized
         * 
         * ? Output Format:
         * ? (rotational angle {x, y, z}), (angular velocity {x, y, z}),
         * ? (gravitational acceleration {x, y, z}), (linear acceleration {x, y, z}),
         * ? (magnetism {x, y, z})
         */
        void UseBNO() {
                if (!Bno_init_state) {
                        sprintf(radiopacket + strlen(radiopacket),                              // Add to radiopacket
                        "+ + + + + + + + + + + + + + + ");                                      // 15 pluses - no values
                        return;
                }

                                                                                                // Bno had a successful initialization
                #if defined(DEBUG) && defined(TEST_MODE)                                        // !Warning!    Should not be executed on launch
                                                                                                // !            Useless unless you're debugging
                        uint8_t system, gyroCal, accelCal, magnCal = 0;                         // Define Calibration variables
                        bno.getCalibration(&system, &gyroCal, &accelCal, &magnCal);             // Get Calibration variables from bno

                        DEBUG(F("System Calibration = "));
                        DEBUGLN(system, DEC);                                                   // Get Decimal value of system variable

                        DEBUG(F("Gyro Calibration = "));
                        DEBUGLN(gyroCal, DEC);                                                  // Get Decimal value of gyroscopic calibration

                        DEBUG(F("Acceleration Calibration = "));
                        DEBUGLN(accelCal);                                                      // Get Decimal value of accelerometer calibration

                        DEBUG(F("Magnetic Calibration = "));
                        DEBUGLN(magnCal);                                                       // Get Decimal value of magnetic calibration
                #endif

                imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);            // Axis–angle representation
                                                                                                // * Degress
                sprintf(radiopacket + strlen(radiopacket),
                        "%.2lf %.2lf %.2lf",                                                    // Format decimal with two decimals
                        euler.x(),                                                              // Rotation angle in the X angle
                        euler.y(),                                                              // Rotation angle in the Y angle
                        euler.z());                                                             // Rotation angle in the Z angle

                imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);         // * rad/s
                sprintf(radiopacket + strlen(radiopacket),
                        " %.2lf %.2lf %.2lf",                                                   // Format decimal with two decimals
                        gyro.x(),                                                               // Angular velocity in the X axis
                        gyro.y(),                                                               // Angular velocity in the Y axis
                        gyro.z());                                                              // Angular velocity in the Z axis

                imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);           // * m/s2
                sprintf(radiopacket + strlen(radiopacket),
                        " %.2lf %.2lf %.2lf",                                                   // Format decimal with two decimals
                        grav.x(),                                                               // Gravitational acceleration in the X axis
                        grav.y(),                                                               // Gravitational acceleration in the Y axis
                        grav.z());                                                              // Gravitational acceleration in the Z axis

                imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);   // * m/s2
                sprintf(radiopacket + strlen(radiopacket),
                        " %.2lf %.2lf %.2lf",
                        linAccel.x(),                                                           // Linear acceleration in the X axis
                        linAccel.y(),                                                           // Linear acceleration in the Y axis
                        linAccel.z());                                                          // Linear acceleration in the Z axis

                imu::Vector<3> magn = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);      // * μTesla
                sprintf(radiopacket + strlen(radiopacket),
                        " %.2lf %.2lf %.2lf ",                                                  // Format decimal with two decimals
                        magn.x(),                                                               // Magnetism in the X axis
                        magn.y(),                                                               // Magnetism in the Y axis
                        magn.z());                                                              // Magnetism in the Z axis
        }
#endif

#if defined(TSL)
        /**
         * Collect luminosity
         * 
         * !Warning!    TSL must be initialized
         * 
         * ? Output Format:
         * ? (infrared luminosity), (full luminosity)
         */
        void UseTSL() {
                if (!Tsl_init_state) {
                        sprintf(radiopacket + strlen(radiopacket),                              // Add to radiopacket
                                " + +");                                                        // 2 pluses - no values
                        return;
                }

                                                                                                // TSL had a successful initialization
                uint32_t luminosity = tsl.getFullLuminosity();                                  // Get TSL luminosity readings (full spectrum)
                                                                                                // * lux

                uint16_t ir, fullLuminosity;

                ir = luminosity >> 16;                                                          // Get luminosity in infrared
                fullLuminosity = luminosity && 0xffff;                                          // Get full luminosity 

                sprintf(radiopacket + strlen(radiopacket),
                        " %d %d",                                                               // Format decimal
                        ir,                                                                     // Infrared luminosity
                        fullLuminosity);                                                        // Full luminosity
        }
#endif
#pragma endregion