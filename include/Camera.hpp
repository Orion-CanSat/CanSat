#ifndef __ORION_CAMERA_H__
#define __ORION_CAMERA_H__

#include <stdint.h>


/**
 * @brief Initializes the Camera Module
 * 
 * @param cameraSD The chip select pin used by the board
 * 
 * @return true If the Camera module started successfully
 * @return false If the Camer module failed to start
 */
bool OrionCameraInitialize(uint8_t cameraCS);

/**
 * @brief Sets the functions which will be used to name the file
 * 
 * The function takes the number of the photo, first argument, and with the help of the
 * size, thrird argument, of the file name, second argument, writes the correct file name
 * plus extension to the second argument. The function returns the number of bytes writen
 * to the second argument. If the second argument is NULL, the the function should just return
 * the number of bytes required to copy the file name to the array.
 * 
 * @param namingFunction The function which will be used to name the files in the SD
 * 
 * @returns true if the function was set successfully
 * @returns false if the function was not set successfully
 */
bool OrionCameraSetNamingFunction(uint8_t(*namingFuncion)(uint16_t, char*, uint8_t));

/**
 * @brief Take picture and store it in the SD
 * 
 * @return true if the picture was taken and stored in the SD successfully
 * @return false if the picture was not taken or was not stored in the SD successfully
 */
bool OrionCameraTakePicture();

/**
 * @brief Finalizes the Camera Module
 */
void OrionCameraFinalize();


/**
 * @brief Outputs the name of the photoID and the extension jpg
 * 
 * @param photoID The ID of the photo
 * @param fname The pointer of the character array for the file name to be writen. If it is NULL then the function will return the required number of bytes to write the file name
 * @param fnameLength The number of bytes that fname has
 * 
 * @return The number of bytes required if the fname is NULL or the number of bytes writen if fname is not equal to NULL
 */
uint8_t DefaultNamingFunction(uint16_t photoID, char* fname, uint8_t fnameLength);


#endif//__ORION_CAMERA_H__