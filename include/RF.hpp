#ifndef __ORION_RF_H__
#define __ORION_RF_H__

#include <stdint.h>

typedef struct {
    uint64_t OrionIdentifier;
    uint16_t OrionNumberOfPackages;
    uint16_t OrionPacketNumber;
    uint8_t OrionData[230];    
} Packet;


/**
 * @brief Inits the RM Module
 * 
 * @return true If the RM module started successfully
 * @return false If the RF module failed to start
 */
bool RFInit();

/**
 * @brief Check if an Packet is available to be read
 * 
 * @return true If there is a pakcet
 * @return false If there is not a packet
 */
bool RFAvailablePacket();

/**
 * @brief Add data to a queue to be sent
 * 
 * @return true if the data was successfully added to queue
 * @return false if the data failed to be added to the queue
 */
bool RFQueue(uint8_t* data, uint32_t size);

/**
 * @brief Send packet to the groundstation
 * 
 * @returns true if the data was successfully sent to the groundstation
 * @returns false if the data was not successfully sent to the groundstation
 */
bool RFSendPacket();

/**
 * @brief Send x number of packets to the groundstation
 * 
 * @returns true if the x packets were sent to the groundstation
 * @returns false if the x packets were not sent to the groundstation
 */
bool RFSendPackets(uint32_t packetsToSend);

/**
 * @brief Take the data and send it to the groundstation
 * 
 * @returns true if the queue was emptied successfully
 * @returns false if the queue was not emptied successfully
 */
bool RFEmptyQueue();


#endif//__ORION_RF_H__