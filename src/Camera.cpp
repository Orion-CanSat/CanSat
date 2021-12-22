#include "Camera.hpp"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <SPI.h>

#include <Orion/Utilities/Time/Delay.hpp>
#include <Orion/Utilities/Time/Timeout.hpp>

#include <ArduCAM.h>
#include "memorysaver.h"


#define MAX_IMAGE_SIZE 230400

static uint8_t _cameraCS;
static bool _cameraInit = false;
static ArduCAM* _arducam = NULL;
static uint8_t(*_namingFuncion)(uint16_t, char*, uint8_t) = DefaultNamingFunction;
static uint8_t _image[MAX_IMAGE_SIZE];
static uint32_t _imageSize = 0;

/**
 * @brief Checks if the SPI communication is working
 * 
 * @return true if the SPI communication is working
 * @return false if the SPI communication is not working
 */
static bool CheckCameraSPI() {
    if (_arducam == NULL)
        return false;

    _arducam->write_reg(ARDUCHIP_TEST1, 0x55);

    uint8_t testRegisterValue = _arducam->read_reg(ARDUCHIP_TEST1);

    return testRegisterValue == 0x55;
}

static bool EvaluateCamera() {
    uint8_t vid;
    uint8_t pid;
    
    _arducam->rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
    _arducam->rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);

    return (vid == 0x56) && (pid == 0x42);
}

bool OrionCameraInitialize(uint8_t cameraCS) {
    if (_cameraInit && (_cameraCS == cameraCS))
        return true;

    if (_arducam == NULL)
        _arducam = (ArduCAM*)malloc(sizeof(ArduCAM));

    if (_arducam != NULL) {
        _cameraCS = cameraCS;

        *_arducam = ArduCAM(OV5642, _cameraCS);
        Orion::Utilities::Time::Delay::DelayMS(200);
        _arducam->CS_LOW();
        
        Orion::Utilities::Time::Delay::DelayMS(200);

        if (Orion::Utilities::Time::Timeout::WaitTimeout(CheckCameraSPI, 2000)) {
            Orion::Utilities::Time::Delay::DelayMS(200);
            if (Orion::Utilities::Time::Timeout::WaitTimeout(EvaluateCamera, 2000)) {
                _arducam->set_format(JPEG);

                _arducam->InitCAM();
                Orion::Utilities::Time::Delay::DelayMS(200);
                _arducam->OV5642_set_JPEG_size(OV5642_320x240);

                Orion::Utilities::Time::Delay::DelayMS(200);
                
                _arducam->CS_HIGH();

                Orion::Utilities::Time::Delay::DelayMS(200);
                    
                _cameraInit = true;
            }
            else
                _cameraInit = false;
        }
        else
            _cameraInit = false;
    }
    else
        _cameraInit = false;
    return _cameraInit;
}

bool OrionCameraSetNamingFunction(uint8_t(*namingFuncion)(uint16_t, char*, uint8_t)) {
    if (namingFuncion == NULL)
        return false;
    
    char* fileName = NULL;
    uint8_t fileNameSizeRequired = namingFuncion(1, NULL, 0);
    
    if (!fileNameSizeRequired)
        return false;

    fileName = (char*)malloc((fileNameSizeRequired + 1) * sizeof(char*));
    if (!fileName)
        return false;
    fileName[fileNameSizeRequired] = '\0';

    uint8_t fileNameSize = namingFuncion(1, fileName, fileNameSizeRequired);

    if ((fileNameSizeRequired == fileNameSize) && (strlen(fileName) == fileNameSize)) {
        _namingFuncion = namingFuncion;
        return true;
    }
    else {
        return false;
    }
}

bool OrionCameraTakePicture() {
    uint8_t temp = 0;
    uint8_t temp_last = 0;
    int32_t i = 0;
    int32_t k = 0;
    uint8_t buffer[256];
    uint32_t pos = 0;
    bool isHeader = false;
    uint32_t cameraTimer = millis();

    _arducam->flush_fifo();
    _arducam->clear_fifo_flag();

    _arducam->start_capture();

    while (!_arducam->get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
        if (millis() - cameraTimer > 600)
            return false;

    uint32_t imageLength = _arducam->read_fifo_length();

    if ((imageLength > MAX_FIFO_SIZE) || (imageLength > MAX_IMAGE_SIZE) || (imageLength == 0))
        return false;

    _arducam->CS_LOW();
    _arducam->set_fifo_burst();

    while (imageLength--) {
        temp_last = temp;
        temp = SPI.transfer(0x00);

        if ((temp == 0xD9) && (temp_last == 0xFF)) {
            buffer[i++] = temp;
            
            _arducam->CS_HIGH();

            for (int32_t j = 0; j < i; i++) 
                _image[pos++] = buffer[j];

            isHeader = true;
            i = 0;
        }
        if (isHeader) {
            if (i < 255)
                buffer[i++] = temp;
            
            else {
                _arducam->CS_HIGH();

                for (int32_t j = 0; j < 256; j++);
            }
        }
    }
    return true;
}

void OrionCameraFinalize() {

}


uint8_t DefaultNamingFunction(uint16_t photoID, char* fname, uint8_t fnameLength) {
    if (!fname) {
        char temp[UINT8_MAX];
        snprintf(temp, UINT8_MAX * sizeof(char), "%" PRIu16 ".jpg", photoID);
        return strlen(temp);
    }
    snprintf(fname, fnameLength * sizeof(char), "%" PRIu16 ".jpg", photoID);
    return strlen(fname);
}
