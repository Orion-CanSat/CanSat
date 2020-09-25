#ifndef __ORION_H__
#define __ORION_H__

#ifndef NAN
    #define NAN (float)0xFFFFFFFF
    #pragma warning Your platform or compiler does not natively support the use of Not-A-Number or aka NAN.
#endif

#ifndef PI
    #define PI (float)3.1415926535897932384626433832795
#endif

#define decPlace float

/**
 * Block of code to determine board type
 */
#ifdef __arm__
    #ifdef CORE_TEENSY
        #define __TEENSY__
        #if defined(__AT90USB162__)
            #define __TEENSY_1_0__
        #elif defined(__ATMEGA32U4__)
            #define __TEENSY_2_0__
        #elif defined(__MKL26Z64__)
            #define __TEENSY_LC__
        #elif defined(__MK20DX128__)
            #define __TEENSY_3_0__
        #elif defined(__MK20DX256__)
            #define __TEENSY_3_1__
            #define __TEENSY_3_2__
        #elif defined(__MK64FX512__)
            #define __TEENSY_3_5__
        #elif defined(__MK66FX1M0__)
            #define __TEENSY_3_6__
        #elif defined(__IMXRT1062__)
            #if defined(BUILTIN_SDCARD)
                #define __TEENSY_4_1__
            #else
                #define __TEENSY_4_0__
            #endif
        #endif
    #endif
#endif


#define __BME280__ 0x000001
#define __BNO055__ 0x000002
#define __SD__ 0x000101
#define __GPS__ 0x0010000
#define __MTK3339__ 0x010001
#define __CAMERA__ 0x010100
#define __OV5642__ 0x010101


#define __TEMPERATURE__ 0x000001
#define __HUMIDITY__ 0x000002
#define __PRESSURE__ 0x000003
#define __ALTITUDE__ 0x000004

#define __ROTATIONAL_ANGLE__ 0x000005
#define __ROTATIONAL_ANGLE_X__ 0x000006
#define __ROTATIONAL_ANGLE_Y__ 0x000007
#define __ROTATIONAL_ANGLE_Z__ 0x000008

#define __ANGULAR_VELOCITY__ 0x000009
#define __ANGULAR_VELOCITY_X__ 0x00000A
#define __ANGULAR_VELOCITY_Y__ 0x00000B
#define __ANGULAR_VELOCITY_Z__ 0x00000C

#define __GRAVITATIONAL_ACCELERATION__ 0x00000D
#define __GRAVITATIONAL_ACCELERATION_X__ 0x00000E
#define __GRAVITATIONAL_ACCELERATION_Y__ 0x00000F
#define __GRAVITATIONAL_ACCELERATION_Z__ 0x000010

#define __LINEAR_ACCELERATION__ 0x000011
#define __LINEAR_ACCELERATION_X__ 0x000012
#define __LINEAR_ACCELERATION_Y__ 0x000013
#define __LINEAR_ACCELERATION_Z__ 0x000014

#define __LINEAR_VELOCITY__ 0x000015
#define __LINEAR_VELOCITY_X__ 0x000016
#define __LINEAR_VELOCITY_Y__ 0x000017
#define __LINEAR_VELOCITY_Z__ 0x000018

#define __LINEAR_DISPLACEMENT__ 0x00000019
#define __LINEAR_DISPLACEMENT_X__ 0x00001A
#define __LINEAR_DISPLACEMENT_Y__ 0x00001B
#define __LINEAR_DISPLACEMENT_Z__ 0x00001C

#define __MAGNETISM__ 0x00001D
#define __MAGNETISM_X__ 0x00001E
#define __MAGNETISM_Y__ 0x00001F
#define __MAGNETISM_Z__ 0x000020

#define __LATITUDE__ 0x000021
#define __LONGITUDE__ 0x000022


#define __AXIS__ 0x10000
#define __X_AXIS__ 0x10001
#define __Y_AXIS__ 0x10002
#define __Z_AXIS__ 0x10003

#endif//__ORION_H__