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


//// AD98944C4E993C2D58F8B2487D2C165B                                                           // Check-sum md5 of the first region


#pragma region SENSOR_REGION
//// #define RFM
//// #define SDC
//// #define BME
//// #define BNO
//// #define TSL
                                                                                                // Will not use A TSL but will provide the
                                                                                                // necessary code for the TSL for others to use
//// #define GPS
//// #define CAM
//// #define MOT
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

#if defined(CAM)
        #include <ArduCAM.h>
        #include "memorysaver.h"
#endif

#if defined(MOT)
        #include <MotorPins.h>
        #include <DRV8835.h>
#endif

#include <SPI.h>
//#include <i2c_t3.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <inttypes.h>
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
//#define MODE 0x00                                                                             // 0x00 for CanSat and 0x01 for GroundStation
                                                                                                // Will change order of execution of the SaveData
                                                                                                // function. If 0x00, Program will gather data and send them
                                                                                                // to Ground. if 0x01, Program will wait for data and store them.
                                                                                                // !Warning!    If deleted, program will not work.
                                                                                                // TODO: Implement `MODE` in program

#define RFM_FREQ 433.3                                                                          // RF Frequency for the RFM9x or 65
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

#define CAM_CS 0                                                                                // Arducam's Chip Select pin
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
#define PI 3.14159265                                                                           // Define PI to 3.14... for the conversion
                                                                                                // between Degrees and Rad
#define c cos                                                                                   // Define cosine to c for faster typing
#define s sin                                                                                   // Define sine to s for faster typing
#pragma endregion


#pragma region MODULE_STATE_REGION
bool Bzr_init_state = false;
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

#if defined(GPS)
        bool Gps_init_state = false;
#endif  
                                                                                                // Beware: Must introduce an init
#if defined(CAM)                                                                                // function for each variable or
        bool Cam_init_state = false;                                                            // else the module will not work
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

#if defined(CAM)
        ArduCAM cam(OV5642, CAM_CS);
#endif

#if defined(MOT)
        MotorPins A_Motor(15, 14);
        MotorPins B_Motor(17, 16);
        DRV8835 driver(A_Motor, B_Motor);
#endif
#pragma endregion


#pragma region VARIABLE_REGION
char radiopacket[300];
char* command;
double* vel = (double*)malloc(3 * sizeof(double));
double* acc = (double*)malloc(3 * sizeof(double));
double* ang = (double*)malloc(3 * sizeof(double));
double* nacc = (double*)malloc(3 * sizeof(double));
double** rm = (double**)malloc(3 * sizeof(double*));
double* pos = (double*)malloc(3 * sizeof(double));
#if defined(SDC)
        File dataFile;
#endif
bool buzzer_state;
unsigned long buzzer_timer;
char* image;
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
        #if (defined DEBUG_MODE)
                Serial.begin(11500);
                delay(INIT_PAUSE);
        #endif

        #if defined(SDC)
        while (Serial.available() == 4 || millis() < 10000);                                    // Wait for CanSat to receive 4 bytes or wait for 10s
                if (Serial.available() == 4) {                                                  // Check if can read 4 bytes from Serial => Computer is connected
                        if (Sdc_init_state = waitTimeout(InitSDC, MAX_TIMEOUT_FUNCTION)) {
                                File dataf = SD.open("data.txt");
                                if (dataf) {
                                        uint64_t fsize = dataf.size();
                                        char fsizeBuf[21];
                                        sprintf(fsizeBuf, "%" PRIu64, fsize);
                                        Serial.println(fsizeBuf);
                                        delay(4000);
                                        for (uint64_t i = 0; i < fsize; i++) {
                                                Serial.print(dataf.read());
                                        }
                                }
                        }
                }
        #endif
        pinMode(SCK_PIN, OUTPUT);                                                               // Set the SPI Clock to Output
        pinMode(MOSI_PIN, OUTPUT);                                                              // Set the MOSI to Output
        pinMode(MISO_PIN, OUTPUT);                                                              // Set the MISO to Output
        pinMode(RFM_RST, OUTPUT);                                                               // Set the RFM Reset pin to Output
        pinMode(RFM_CS, OUTPUT);                                                                // Set the RFM Chip Select pin to Output
        pinMode(RFM_INT, INPUT);                                                                // Set the RFM Interrupt pin to Input
        pinMode(BUZZER_PIN, OUTPUT);

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

        for (uint8_t i = 0; i < 3; i++) {
                rm[i] = (double*)malloc(3 * sizeof(double));
                acc[i] = 0;
                nacc[i] = 0;
                vel[i] = 0;
                pos[i] = 0;
        }
        

        #if defined(RFM)
                if (Rfm_init_state = waitTimeout(InitRFM, MAX_TIMEOUT_FUNCTION)) {

                }
                else {

                }
        #endif

        #if defined(SDC)
                if (Sdc_init_state) {
                        SD.remove("data.txt");
                }
                else if (Sdc_init_state = waitTimeout(InitSDC, MAX_TIMEOUT_FUNCTION)) {
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

                radiopacket[0] = '\0';
        #endif

        #if defined(SDC)
                sprintf(radiopacket,
                        "%s\0", (Sdc_init_state)?
                        "SDC Init complete": "SDC Init failed");

                SaveData();
        
                radiopacket[0] = '\0';
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

        #if defined(CAM)
                if (Cam_init_state = waitTimeout(InitCAM, 3 * MAX_TIMEOUT_FUNCTION)) {
                        sprintf(radiopacket, "Cam Init successful");
                        SaveData();
                }
                else {
                        sprintf(radiopacket, "Cam Init failed");
                        SaveData();
                }
        #endif
        
        PrepareHeader();                                                                        // Send how data fill be formatted

        sprintf(radiopacket, "\0");
}

void loop() {

        //waitTimeout(UseCAM, 500);
        if (millis() - buzzer_timer > 1000 && Bzr_init_state && IsNotMovingOrMovingSlowly()) {  // Checks if buzzer is initialized
                                                                                                // and if it is not moving
                digitalWrite(BUZZER_PIN, !buzzer_state);                                        // Change state of Buzzer pin
                buzzer_state = !buzzer_state;
                buzzer_timer = millis();                                                        // Reset buzzer timer
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
                for (int i = 0; i < 50; i++) gps.read();
                                                                                                // defined to be used
                UseGPS();                                                                       // Gather data from GPS
        #endif

        SaveData();                                                                             // Save and Send data
        
        radiopacket[0] = '\0';                                                                  // Empty data

        delay(1000);
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

bool IsNotMovingOrMovingSlowly() {
        #if defined(BNO)
                double velocity = sqrt(pow(vel[0], 2) + pow(vel[1], 2) + pow(vel[2], 2));
                return (velocity < 1);
        #else
                return true;
        #endif
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
                gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
                gps.sendCommand(PGCMD_ANTENNA);
                delay(1000);
                GPSSerial.println(PMTK_Q_RELEASE);

                sprintf(radiopacket,
                        "GPS Init complete\0");

                SaveData();                                                                     // Save and Send data
                return true;

        }
#endif

#if defined(CAM)

        bool CheckCameraSPI() {
                cam.write_reg(ARDUCHIP_TEST1, 0x55);                                            // Write 0x55 in test register to test
                                                                                                // if the SPI connection is working

                uint8_t testRegisterValue = cam.read_reg(ARDUCHIP_TEST1);                       // Read from test register. The value
                                                                                                // returned should be 0x55
                if (testRegisterValue != 0x55)                                                  // If not then return false
                        return false;
                return true;
        }

        bool EvaluateCamera() {
                uint8_t vid, pid;
                cam.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
                cam.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
                if ((vid != 0x56) || (pid != 0x42))
                        return false;
                return true;
        }

        /**
         * Initialize the ArduCAM 5MP plus
         * 
         * !Warning!    Might end in an endless condition
         * !            Depends on microprocessor and wiring
         */
        bool InitCAM() {
                cam.CS_LOW();                                                                   // Activates the Camera SPI
                if (!waitTimeout(CheckCameraSPI, MAX_TIMEOUT_FUNCTION)) {                       // Checks if Camera can not successfully
                  return false;                                                                 // communicate with the microcontroller
                }
                if (!waitTimeout(EvaluateCamera, MAX_TIMEOUT_FUNCTION)) {                       // Check if cammera is what it was programmed
                  return false;
                }
                cam.set_format(JPEG);                                                           // Set pictures format to JPEG.
                                                                                                // Used for the best size to quality ration
                cam.InitCAM();                                                                  // initializes the camera
                cam.OV5642_set_JPEG_size(OV5642_320x240);                                       // Set default image size to 320 x 240
                                                                                                // chosen for the small image size
                delay(1000);
                cam.CS_HIGH();
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
                                "+ + +");                                                       // 3 pluses - no values
                        return;
                }

                                                                                                // Bme had a successful initialization
                sprintf(radiopacket + strlen(radiopacket),
                        "%.2lf %.2lf %.2lf",                                                    // Format decimal with two decimals
                        bme.readTemperature(),                                                  // Temperature
                        bme.readPressure(),                                                     // Pressure
                        bme.readHumidity());                                                    // Humidity
                        //bme.readAltitude(SEA_LEVEL_PRESSURE_REF));                            // Altitude //! Can be computed on the Server
        }
#endif

#if defined(BNO)

        /**
         * TODO: Make function
         */
        void GetPossition() {

        }

        /**
         * Used for the Normalization of the BNO velocity
         * which will be later used for the inertial possitioning
         * system.
         * 
         * TODO: Include the possitioning system
         */
        void NormalizeAccelerations() {
                for (uint8_t i = 0; i < 3; i++)
                        ang[i] *= PI / 180;                                                     // Degrees to Rad conversion
                
                rm[0][0] = c(ang[2]) * c(ang[1]);                                               // Fills out the rotation matrix
                rm[0][1] = c(ang[2]) * s(ang[0]) * s(ang[1]) - c(ang[0]) * s(ang[2]);           // for the normalazation of the accelerations
                rm[0][2] = s(ang[0]) * s(ang[2]) + c(ang[0]) * c(ang[2]) * s(ang[1]);
                rm[1][0] = c(ang[1]) * s(ang[2]);
                rm[1][1] = c(ang[0]) * c(ang[2]) + s(ang[0]) * s(ang[2]) * s(ang[1]);
                rm[1][2] = c(ang[0]) * s(ang[1]) * s(ang[2]) - c(ang[2]) * s(ang[0]);
                rm[2][0] =  (  -1  ) * s(ang[1]);
                rm[2][1] = c(ang[1]) * s(ang[0]);
                rm[2][2] = c(ang[0]) * c(ang[1]);

                for (uint8_t i = 0; i < 3; i++)
                        nacc[i] = rm[i][0] * acc[0] + rm[i][1] * acc[1] + rm[i][2] * acc[2];    // Normalize Accelerations     
        }

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
                        " + + + + + + + + +");                                                 // 9 pluses - no values
                        return;
                }

                                                                                                // Bno had a successful initialization
                #if defined(DEBUG) && defined(TEST_MODE)                                        // !Warning!    Should not be executed on launch
                                                                                                // !            Useless unless you're debugging
                        uint8_t system, gyroCal, accelCal, magnCal = 0;                         // Define Calibration variables
                        bno.getCalibration(&system, &gyroCal, &accelCal, &magnCal);             // Get Calibration variables from bno

                        sprintf(radiopacket, "System Calibration = %d\0", system);              // Get Decimal value of system variable
                        SaveData();

                        sprintf(radiopacket, "Gyro Calibration = %d\0", gyroCal);               // Get Decimal value of gyroscopic calibration                                                 

                        sprintf(radiopacket, "Acceleration Calibration = %d\0", accelCal);      // Get Decimal value of accelerometer calibration
                        SaveData();

                        sprintf(radiopacket, "Magnetic Calibration = %d\0", magnCal);           // Get Decimal value of magnetic calibration
                        SaveData();
                #endif

                imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);            // Axis–angle representation
                                                                                                // * Degress
                ang[0] = euler.x();
                ang[1] = euler.y();
                ang[2] = euler.z();
                sprintf(radiopacket + strlen(radiopacket),
                        " %.2lf %.2lf %.2lf",                                                   // Format decimal with two decimals
                        ang[0],                                                                 // Rotation angle in the X angle
                        ang[1],                                                                 // Rotation angle in the Y angle
                        ang[2]);                                                                // Rotation angle in the Z angle

                imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);         // * rad/s
                sprintf(radiopacket + strlen(radiopacket),
                        " %.2lf %.2lf %.2lf",                                                   // Format decimal with two decimals
                        gyro.x(),                                                               // Angular velocity in the X axis
                        gyro.y(),                                                               // Angular velocity in the Y axis
                        gyro.z());                                                              // Angular velocity in the Z axis

                /*imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);         // * m/s2
                 *sprintf(radiopacket + strlen(radiopacket),
                 *        " %.2lf %.2lf %.2lf",                                                 // Format decimal with two decimals
                 *        grav.x(),                                                             // Gravitational acceleration in the X axis
                 *        grav.y(),                                                             // Gravitational acceleration in the Y axis
                 *        grav.z());                                                            // Gravitational acceleration in the Z axis
                 */

                imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);   // * m/s2

                acc[0] = linAccel.x();
                acc[1] = linAccel.y();
                acc[2] = linAccel.z();
                sprintf(radiopacket + strlen(radiopacket),
                        " %.2lf %.2lf %.2lf",
                        acc[0],                                                                 // Linear acceleration in the X axis
                        acc[1],                                                                 // Linear acceleration in the Y axis
                        acc[2]);                                                                // Linear acceleration in the Z axis

                /*imu::Vector<3> magn = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);    // * μTesla
                 *sprintf(radiopacket + strlen(radiopacket),
                 *        " %.2lf %.2lf %.2lf",                                                 // Format decimal with two decimals
                 *        magn.x(),                                                             // Magnetism in the X axis
                 *        magn.y(),                                                             // Magnetism in the Y axis
                 *        magn.z());                                                            // Magnetism in the Z axis
                 */

                NormalizeAccelerations();
                GetPossition();
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
         * ? (latitude), (longitude), (altitude)
         */
        void UseGPS() {
                if (!Gps_init_state) {
                        sprintf(radiopacket + strlen(radiopacket),
                                "+ +");
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
                        sprintf(gpsPacket + strlen(gpsPacket),
                                "\nLocation: %.4f, %.4f, %.4f\nSpeed: %d\nSatellites: %d\0",
                                gps.latitude,
                                gps.longitude,
                                gps.altitude,
                                gps.speed,
                                gps.satellites);

                        sprintf(radiopacket + strlen(radiopacket),
                                " %.4f %.4f",
                                gps.latitude,
                                gps.longitude);
                                //gps.altitude);
                }
                else {
                        sprintf(radiopacket + strlen(radiopacket),
                                " + +");
                }

                DEBUGLN(gpsPacket);
        }
#endif

#if defined(CAM)
        /**
         * Takes picture and stores it in memory
         * 
         * !Warning!    CAM must be initialized
         */
        bool UseCAM() {
                uint8_t temp = 0, temp_last = 0;
                byte buffer[250];
                int i = 0;
                uint32_t pos = 0;
                bool isHeader = false;
                uint32_t cameraTimer = millis();
                cam.flush_fifo();
                cam.clear_fifo_flag();
                cam.start_capture();
                while(!cam.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) if(millis() - cameraTimer > 600) return false;
                uint32_t imagelength = cam.read_fifo_length(), length = imagelength;
                if (length >= MAX_FIFO_SIZE) return false;
                if (length == 0) return false;
                if (image) free(image);
                image = (char*)malloc(length * sizeof(char));
                if (!image) return false;
                cam.CS_LOW();
                cam.set_fifo_burst();

                while (imagelength--) {
                        temp_last = temp;
                        temp = SPI.transfer(0x00);
                        if (temp == 0xD9 && temp_last == 0xFF) {
                                buffer[i++] = temp;
                                cam.CS_HIGH();
                                for (int j = 0; j < i; i++) {
                                        image[pos++] = buffer[j];
                                }
                                isHeader = false;
                                i = 0;
                        }
                        if (isHeader) {
                                if (i < 255) 
                                        buffer[i++] = temp;
                                else {
                                        cam.CS_HIGH();
                                        for (int j = 0; j < 256; j++)
                                                image[pos++] = buffer[j];
                                        cam.CS_LOW();
                                        cam.set_fifo_burst();
                                }
                       }
                       else if (temp == 0xD8 && temp_last == 0xFF) {
                               isHeader = true;
                               buffer[i++] = temp_last;
                               buffer[i++] = temp;
                       }
                }
                cam.CS_HIGH();
                File out;
                out = SD.open("image.jpg");
                for (uint32_t j = 0; j < length; j++) {
                        out.write(image[j]);
                }
                out.close();
                return true;
        }
#endif
#pragma endregion


#pragma region DATA_HANDLER_REGION

void SaveData() {
        DEBUGLN(radiopacket);
        if (Serial.available()) {
                command = (char*)Serial.readString().c_str();
                #if defined(RFM)
                        rfm.send((uint8_t*)command, strlen(command) + 1);
                #endif
                ApplyCommand();
        }
        #if defined(RFM)
        else if (rfm.waitAvailableTimeout(60)) {
                uint8_t len;
                rfm.recv((uint8_t*)command, &len);
                DEBUGLN(command);
                ApplyCommand();
        }
        #endif
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
                rfm.send((uint8_t*)radiopacket, strlen(radiopacket) + 1);
                rfm.waitPacketSent();
        }
#endif
}

#pragma endregion


#pragma region COMMANDS_REGION

void bzr() {
        command[strlen(command) - 1] = '\0';
        if (strcmp(command, "on") == 0)
                digitalWrite(BUZZER_PIN, HIGH);
        else if (strcmp(command, "pulse") == 0)
                Bzr_init_state = true;
        else if (strcmp(command, "npulse") == 0) {
                Bzr_init_state = false;
                digitalWrite(BUZZER_PIN, LOW);
        }
        else
                digitalWrite(BUZZER_PIN, LOW);
}

void lmp() {
        command[strlen(command) - 1] = '\0';
        DEBUG("LMP received. Arguments: \"");
        DEBUG(command);
        DEBUGLN("\"");
}

#if defined(RFM)
void rfc() {
        if (!Rfm_init_state) return;
        if (strlen(command) != 5) return;
        uint8_t* com1;
        strncpy((char*)com1, command, 4);
        if (strcmp(command, "sleep") == 0) {
                rfm.sleep();
                DEBUGLN("Set mode to Sleep");
        }
        else if (strcmp((char*)com1, "freq") == 0) {
                if (command[4] - '0' >= 0 && command[4] - '0' <= 9) {
                        if (rfm.setFrequency(433.0 + (double)(command[4] - '0') / 10)) {
                                DEBUG("Set frequency to ");
                                DEBUGLN(433.0 + (double)(command[4] - '0') / 10);
                        }
                }
        }
}
#endif

void rmp() {
        command[strlen(command) - 1] = '\0';
        DEBUG("RMP received. Arguments: \"");
        DEBUG(command);
        DEBUGLN("\"");
        
}

#pragma endregion


#pragma region APPLY_COMMAND_REGION
char* commands[] = {
        "bzr",
        "lmp",
#if defined(RFM)
        "rfc",
#endif
        "rmp"
};

void (*functions[])() = {
        bzr,
        lmp,
#if defined(RFM)
        rfc,
#endif
        rmp
};

/**
 * TODO: Change to BinarySearchFunctions
 */
int16_t SearchFunctions(char comm[]) {
        //// return BinarySearchFunctions(comm, 0, sizeof(commands) / sizeof(commands[0]));
        return LinearSearchFunction(comm);
}

/**
 * Searches all commands array untill it finds
 * the correct index to give back
 * 
 * !Warning!    Can not index all items
 * 
 * TODO: Needs fixing
 */
int16_t BinarySearchFunctions(char comm[], uint16_t min, uint16_t max) {
        uint16_t mid = (min + max) / 2;
        uint32_t comp = strcmp(commands[mid], comm);
        DEBUG(mid);
        DEBUG(" ");
        DEBUGLN(comp);
        if (max < min) return -1;
        if (comp > 0) return BinarySearchFunctions(comm, mid + 1, max);
        else if (comp < 0) return BinarySearchFunctions(comm, min, mid - 1);
        else if (comp == 0) return mid;
}

int16_t LinearSearchFunction(char comm[]) {
        for (uint16_t i = 0; i < sizeof(commands) / sizeof(commands[0]); i++)
                if (strcmp(commands[i], comm) == 0) return i;
        return -1;
}

void ApplyCommand() {
        char comm[3] = { command[0], command[1], command[2] };
        command += 3;
        int16_t pos = SearchFunctions(comm);
        if (pos == -1) return;
        (*functions[pos])();
}

#pragma endregion