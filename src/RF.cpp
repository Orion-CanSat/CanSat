#ifdef ARDUINO
    #include <Arduino.h>
#endif

#include "RF.hpp"
#include "Debug.hpp"

#include <stdint.h>

#include <Orion.h>
#include <Orion/Utilities/Time/Delay.hpp>
#include <Orion/Utilities/Time/Time.hpp>

#include <RH_RF95.h>

#define MAX_QUEUED 10

static Packet* _packets = (Packet*)NULL;
static int8_t _queued = 0;
static int32_t _packageID = 0;
static bool _rfInitialized = false;
static uint32_t _lastPacketSentTime = 0;
static RH_RF95* _rf = (RH_RF95*)NULL;

bool RFInit() {
    _packets = (Packet*)malloc(MAX_QUEUED * sizeof(Packet));
    if (!_packets) {
        Error("RF: RFInit: Could not allocate space for packets");
        return _rfInitialized;
    }

#if defined(__IMXRT1062__)
    _rf = new RH_RF95(10, 2);
#elif defined(__MK66FX1M0__)
    _rf = new RH_RF95(9, 2);
#endif
    
    // Set reset pin to output and reset RFM.
    pinMode(3, OUTPUT);
    digitalWrite(3, HIGH);
    Orion::Utilities::Time::Delay::DelayMS(100);
    digitalWrite(3, LOW);
    Orion::Utilities::Time::Delay::DelayMS(100);

    if (!_rf) {
        Error("RF: RFInit: Could not create new RF95 object");
        return _rfInitialized;
    }
    else if (!_rf->init()) {
        Error("RF: RFInit: Could not initialize the RF");
        return _rfInitialized;
    }
    else if (!_rf->setFrequency(433.0)) {
        Error("RF: RFInit: Could not set the RF frequency to 433.0 Hz");
        return _rfInitialized;
    }
    else {
        // Highest output transmission power
        _rf->setTxPower(23, false);
        _rfInitialized = true;
    }

    return _rfInitialized;
}

bool RFAvailablePacket() {
    if (!_rfInitialized) {
        Error("RF: RFAvailablePacket: RF is not initialized");
        return false;
    }

    return _rf->available();
}

bool RFQueue(uint8_t* data, uint32_t size) {
    if (!_rfInitialized || _queued || size > 230)
        return false;

    if (_queued >= MAX_QUEUED) {
        Info("RF: RFQueue: Max packet queued reached");
        Info("RF: RFQueue: Attempting to send one packet to clear up one spot");

        if (!RFSendPacket()) {
            Error("RF: RFQueue: Could not empty one spot for packet to be queued");
            return false;
        }
    }

    // IMXRT1062 and MK66FX1M0 processors use little endian format
#if (defined(__IMXRT1062__) || defined(__MK66FX1M0__))
    _packets[_queued].OrionIdentifier = ORION_IDENTIFIER_BE;
#endif
    _packets[_queued].OrionIdentifier = _packageID++;
    _packets[_queued].OrionPackageSize = size;
    _packets[_queued].OrionNumberOfPackages = 1;
    _packets[_queued].OrionPacketNumber = 1;

    // Copy data to packet
    memcpy(_packets[_queued++].OrionData, data, size);

    return true;
}

bool RFSendPacket() {
    if (!_rfInitialized) {
        Error("RF: RFSendPacket: RF is not initialized");
        return false;
    }
    else if (_queued == 0) {
        Error("RF: RFSendPacket: No packets are queued");
        return false;
    }

    // Make sure each packet has a 50ms delay from the previous
    uint32_t currentTime = Orion::Utilities::Time::Time::CurrentTimeMSFromBoot();
    if (_lastPacketSentTime + 50 >= currentTime)
        Orion::Utilities::Time::Delay::DelayMS(50 - (currentTime - _lastPacketSentTime));

    // Documentation: http://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html#ae7d3743511fdcc1b41f5f8e6b0964c1f
    bool sent = _rf->send((uint8_t*)&_packets[0], sizeof(Packet));

    if (!sent) {
        Error("RF: RFSendPacket: Could not send packet");
        return false;
    }

    for (int i = 0; i < MAX_QUEUED - 1; i++)
        _packets[i] = _packets[i + 1];

    _lastPacketSentTime = Orion::Utilities::Time::Time::CurrentTimeMSFromBoot();

    Debug("RF: RFSendPacket: Successfully sent packet");
    return sent;
}

bool RFSendPackets(uint32_t packetsToSend) {
    // Sets packetsToSend equal to max(packetsToSend, _queued)
    packetsToSend = (packetsToSend >= _queued) ? _queued : packetsToSend;

    for (uint32_t i = 0; i < packetsToSend; i++)
    {
        if (!RFSendPacket()) {
            Error("RF: RFSendPackets: Sending a packet failed");
            return false;
        }
    }

    return true;
}

bool RFEmptyQueue() {
    if (!_rfInitialized) {
        Error("RF: RFEmptyQueue: RF is not initialized");
        return false;
    }
    else if (_queued == 0) {
        Info("RF: RFEmptyQueue: No packets are queued");
        return true;
    }

    _packageID -= _queued;
    _queued = 0;

    return true;
}