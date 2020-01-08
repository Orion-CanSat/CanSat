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


//// 992DC9E2FC6CAB471FAB75ADDF22AA73                                                           // Check-sum md5 of the first include


#pragma region SENSOR_REGION
//// #define RFM
//// #define SDC
//// #define BME
//// #define BNO
//// #define TSL
//// #define GPS
#pragma endregion

#pragma region INCLUSE_REGION
#if defined(RFM)
        #include <RH_RF95.h>
#endif

#if defined(SDC)
        #include <SD.h>
#endif

#if defined(BME)
        #include <Adafruit_BME280.h>
#endif

#if defined(BNO)
        #include <Adafruit_BNO055.h>
        #include <utility/imumaths.h>
#endif

#if defined(TSL)
        #include <Adafruit_TSL2591.h>
#endif

#if defined(GPS)
        #include <Adafruit_GPS.h>
#endif

#include <SPI.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#pragma endregion


#pragma region BRIGHTNESS_TSL_MODULE_REGION
//// #define BRIGHTNESS_TSL_MAX
//// #define BRIGHTNESS_TSL_HIGH
//// #define BRIGHTNESS_TSL_MEDIUM
//// #define BRIGHTNESS_TSL_LOW


// !Warning!    Do not modify from here
// !            till end of region
#if defined(TSL)
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
        #else
                #error Please select TSL Brightness from BRIGHTNESS_TSL_MODULE_REGION
        #endif
#endif
#pragma endregion


#pragma region DEBUG_REGION
#define DEBUG_MODE                                                                              // Define DEBUG MODE
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
#define MODE 0x00                                                                               // 0x00 for CanSat and 0x01 for GroundStation
                                                                                                // Will change order of execution of the SaveData
                                                                                                // function. If 0x00, Program will gather data and send them
                                                                                                // to Ground. if 0x00, Program will wait for data and store them.
                                                                                                // !Warning!    If deleted, program will not work. 

#define RFM_FREQ 433.0                                                                          // RF Frequency for the RFM9x or 65
                                                                                                // !Warning!	Base's Frequency.
                                                                                                // Must be set to the same
                                                                                                // Frequency
                                                                                                // TODO: Change it to a different Frequency


#define RFM_RST 3                                                                               // Reset pin number

#define RFM_INT 2                                                                               // RF Interrupt pin
                                                                                                // Will Interrupt program'same
                                                                                                // execution if pin pulled HIGH
                                                                                                // 
                                                                                                // !Warning!	Must be Interrupt 
                                                                                                // !            Friendly.

#define RFM_CS 9                                                                                // RF's Chip Select pin
                                                                                                // Used for the SPI Protocol

#define CANSAT_NODE_ADDRESS 0x01
#define GROUND_NODE_ADDRESS 0x02

#if defined(SDC)
        #define SDC_CS BUILTIN_SDCARD                                                           // SD card's Chip Select pin
#endif                                                                                          // Used for the SPI Protocol

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

#define BUZZER_PIN 4
#pragma endregion


#pragma region MODULE_STATE_REGION
#if defined(RFM)                                                                                // Declaring all initialization
        bool Rfm_init_state = false;                                                            // status variables for GLOBAL
#endif                                                                                          // use in the functions

#if defined(SDC)
        bool Sdc_init_state = false;
#endif

#if defined(BNO)
        bool Bno_init_state = false;
#endif

#if defined(BME)
        bool Bme_init_state = false;
#endif

#if defined(TSL)
        bool Tsl_init_state = false;
#endif

                                                                                                // Beware: Must introduce an init
#if defined(GPS)                                                                                // function for each variable or
        bool Gps_init_state = false;                                                            // else the module will not work
#endif                                                                                          // by default 
#pragma endregion


#pragma region MODULE_REGION
#if defined(RFM)
        RH_RF95 rfm(RFM_CS, RFM_INT);
#endif

#if defined(BNO)
        Adafruit_BNO055 bno = Adafruit_BNO055(55);
#endif

#if defined(BME)
        Adafruit_BME280 bme;
#endif

#if defined (TSL)
        Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
#endif

#if defined(GPS)
        #define GPSSerial Serial3
        Adafruit_GPS gps(&GPSSerial);
        #define GPSECHO false
#endif
#pragma endregion


#pragma region VARIABLE_REGION
char radiopacket[300];
#if defined(SDC)
        File dataFile;
#endif
bool buzzer_state;
unsigned long buzzer_timer;
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
        pinMode(SCK_PIN, OUTPUT);                                                               // Set the SPI Clock to Output
        pinMode(MOSI_PIN, OUTPUT);                                                              // Set the MOSI to Output
        pinMode(MISO_PIN, OUTPUT);                                                              // Set the MISO to Output
        pinMode(RFM_RST, OUTPUT);                                                               // Set the RFM Reset pin to Output
        pinMode(RFM_CS, OUTPUT);                                                                // Set the RFM Chip Select pin to Output
        pinMode(RFM_INT, INPUT);                                                                // Set the RFM Interrupt pin to Input
        pinMode(BUZZER_PIN, OUTPUT);

        #if (defined DEBUG_MODE)
                Serial.begin(11500);
                delay(INIT_PAUSE);
        #endif

        Wire.begin();                                                                           // Start I2C Protocol
        SPI.begin();                                                                            // Start SPI Protocol

#if defined(SDC)
        digitalWrite(SDC_CS, HIGH);                                                             // De-activates SD SPI
#endif
        digitalWrite(RFM_CS, HIGH);                                                             // De-activates RFM SPI
        digitalWrite(CAM_CS, HIGH);                                                             // De-activates Camera SPI
        digitalWrite(BUZZER_PIN, LOW);

        buzzer_state = false;

        delay(INIT_PAUSE);

        #if defined(RFM)
                if (Rfm_init_state = waitTimeout(InitRFM, MAX_TIMEOUT_FUNCTION)) {

                }
                else {

                }
        #endif

        #if defined(SDC)
                if (Sdc_init_state = waitTimeout(InitSDC, MAX_TIMEOUT_FUNCTION)) {
                        SD.remove("data.txt");
                }
                else {

                }
        #endif

        #if defined(RFM)
                sprintf(radiopacket,
                        "%s\0",
                        (Rfm_init_state)? "RFM Init complete": "RFM Init failed");

                SaveData();

                radiopacket[0] = "\0";
        #endif

        #if defined(SDC)
                sprintf(radiopacket,
                        "%s\0", (Sdc_init_state)?
                        "SDC Init complete": "SDC Init failed");

                SaveData();
        
                radiopacket[0] = "\0";
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

        #if defined(GPS)
                if (Gps_init_state = waitTimeout(InitGPS, MAX_TIMEOUT_FUNCTION)) {
                  
                }
                else {
                  
                }
        #endif

        PrepareHeader();                                                                        // Send how data fill be formatted

        sprintf(radiopacket, "\0");
}

void loop() {

        if (buzzer_timer + 1000 < millis()) {
                digitalWrite(BUZZER_PIN, !buzzer_state);
                buzzer_state = !buzzer_state;
                buzzer_timer = millis();
        }

        sprintf(radiopacket + strlen(radiopacket),
                "%lu ",
                millis());

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
        #if defined(GPS)                                                                        // Check if GPS is
                                                                                                // defined to be used
                UseGPS();                                                                       // Gather data from GPS
        #endif

        SaveData();                                                                             // Save and Send data
        
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

void PrepareHeader() {
        sprintf(radiopacket, "%s%s%s%s",
        #if defined(BME)
                "Temperature Pressure Humidity Altitude ",
        #else
                "",
        #endif


        #if defined(BNO)
                "rotational_angle_x y z angular_velocity_x y z gravitational_acceleration_x y z linear_acceleration_x y z}) magnetism_x y z ",
        #else
                "",
        #endif


        #if defined(TSL)
                "infrared_luminosity full_luminosity ",
        #else
                "",
        #endif


        #if defined(GPS)
                "latitude longitude altitude speed");
        #else
                "");
        #endif

        SaveData();
}

#pragma region INITIALIZATION_REGION
#if defined(RFM)
        /**
         * Initialize the RF module
         * 
         * !Warning!    Might end in an endless condition
         * !            Depends on microprocessor and
         * !            connections made.
         */
        bool InitRFM() {
                digitalWrite(RFM_RST, LOW);                                                     // Reset the RFM9x Module. Might not work if not
                delay(100);                                                                     // Reset.
                digitalWrite(RFM_RST, HIGH);
                delay(100);

                if (rfm.init() && rfm.setFrequency(RFM_FREQ)) {                                 // Check if RFM initialized and successfully set Frequency.

                        rfm.setTxPower(23, false);                                              // Set Transmit Power to x db for one meter.

                        rfm.setThisAddress(CANSAT_NODE_ADDRESS);                                // Used to create an end to end communication. Will not allow
                        rfm.setHeaderFrom(CANSAT_NODE_ADDRESS);                                 // for communication outside specified Nodes
                        rfm.setHeaderTo(GROUND_NODE_ADDRESS);

                        rfm.setPromiscuous(false);                                              // Allow / Disallow communication outside specified Nodes

                        return true;
                }
                else {
                        return false;
                }
        }
#endif

#if defined(SDC)
        /**
         * initialize the SD card
         * 
         * !Warning!    Might end in an endless condition
         * !            Depends on microprocessor and SD
         * !            module used
         */
        bool InitSDC() {

                if (SD.begin(SDC_CS)) {
                        return true;
                }
                else {
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
                        sprintf(radiopacket,                                                    // Print that BNO did initialize
                                "BME Init complete\0");

                        SaveData();                                                             // Save and Send data
                        return true;
                }
                else {                                                                          // BNO did not initialize successfully
                        sprintf(radiopacket,                                                    // Print that BME did not initialize
                                "BME Init faliled\0");

                        SaveData();                                                             // Save and Send data
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
                        sprintf(radiopacket,                                                    // Print that BNO did initialize
                                "BNO Init complete\0");

                        SaveData();                                                             // Save and Send data
                        return true;
                }
                else {                                                                          // BNO did not initialize successfully
                        sprintf(radiopacket,                                                    // Print that BNO did not initialize
                                "BNO Init failed\0");

                        SaveData();                                                             // Save and Send data
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
                        sprintf(radiopacket,                                                    // Print that TSL did initialize
                                "TSL Init complete\0");

                        SaveData();                                                             // Save and Send data

                        tsl.setGain((tsl2591Gain_t)TSL_GAIN);                                   // Set TSL's Brightness settings
                        tsl.setTiming(TSL_TIMMING);                                             // !Warning!    The Dimmer the environment
                                                                                                // !            the longer it takes to take
                                                                                                // !            accurate measurements

                        return true;
                }
                else {                                                                          // TSL did not initialize successfully
                        sprintf(radiopacket,                                                    // Print that TSL did not initialize
                                "TSL Init failed\0");

                        SaveData();                                                             // Save and Send Data
                        return true;
                }
        }
#endif

#if defined(GPS)
        /**
         * Initialize the UART GPS
         * 
         * !Warning!    Might end in an endless condition
         * !            Depends on microprocessor and wiring
         */
        bool InitGPS() {
                gps.begin(9600);
                gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
                gps.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ);
                gps.sendCommand(PGCMD_ANTENNA);
                delay(1000);
                GPSSerial.println(PMTK_Q_RELEASE);

                sprintf(radiopacket,
                        "GPS Init complete\0");

                SaveData();                                                                     // Save and Send data
                return true;

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

                        sprintf(radiopacket, "System Calibration = %d\0", system);                // Get Decimal value of system variable
                        SaveData();

                        sprintf(radiopacket, "Gyro Calibration = %d\0", gyroCal);                 // Get Decimal value of gyroscopic calibration                                                 

                        sprintf(radiopacket, "Acceleration Calibration = %d\0", accelCal);        // Get Decimal value of accelerometer calibration
                        SaveData();

                        sprintf(radiopacket, "Magnetic Calibration = %d\0", magnCal);             // Get Decimal value of magnetic calibration
                        SaveData();
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

#if defined(GPS)
        /**
         * Collect GPS data
         * 
         * !Warning!    GPS must be initialized
         * 
         * ? Output Format:
         * ? (latitude), (longitude), (altitude), (speed)
         */
        void UseGPS() {
                if (!Gps_init_state) {
                        sprintf(radiopacket + strlen(radiopacket),
                                " + + +");
                        return;
                }
                if (gps.newNMEAreceived()) {
                        if (!gps.parse(gps.lastNMEA())) {
                                return;
                        }
                }

                char gpsPacket[200] = "\0";

                sprintf(gpsPacket,
                        "Time: %d:%d:%d\nDate: %d/%d/20%d\nFix: %d\nQuality: %d\0",
                        (int)gps.hour,
                        (int)gps.minute,
                        (int)gps.seconds,
                        (int)gps.day,
                        (int)gps.month,
                        (int)gps.year,
                        (int)gps.fix,
                        (int)gps.fixquality);
  
                if (gps.fix) {
                        float latitude = gps.latitude * ((gps.lat == 'N')? 1.0 : -1.0);
                        float longtitude = gps.longitude * ((gps.lon == "E")? 1.0 : -1.0);

                        sprintf(gpsPacket + strlen(gpsPacket),
                                "\nLocation: %.4f, %.4f, %.4f\nSpeed: %d\nSatellites: %d\0",
                                latitude,
                                longtitude,
                                gps.altitude,
                                gps.speed,
                                gps.satellites);

                        sprintf(radiopacket + strlen(radiopacket),
                                " %.4f %.4f %.4f %.2f",
                                latitude,
                                longtitude,
                                gps.altitude,
                                gps.speed);
                }
                else {
                        sprintf(radiopacket + strlen(radiopacket),
                                " + + +");
                }

                Serial.println(gpsPacket);
        }
#endif
#pragma endregion


#pragma region DATA_HANDLER_REGION

void SaveData() {
        DEBUGLN(radiopacket);

#if defined(SDC)
        if (Sdc_init_state) {
                dataFile = SD.open("data.txt", FILE_WRITE);

                if (!dataFile) {
                        DEBUGLN("File opening failed");
                }
                else {
                        dataFile.println(radiopacket);

                        dataFile.close();
                }
        }
#endif

#if defined(RFM)
        if (Rfm_init_state) {
                rfm.setModeRx();
                rfm.send(radiopacket, strlen(radiopacket) + 1);
                rfm.waitPacketSent();
        }
#endif
}

#pragma endregion