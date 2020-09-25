#include "Utilities/Cryptos/Crypto.hpp"

#include <Arduino.h>

using namespace Orion::Utilities::Cryptos;

void Crypto::FreeInputOutput()
{
    if (_inputSize && _input)
        delete _input;
    if (_outputSize && _output)
        delete _output;
    
    _inputSize = _outputSize = 0;
}

Crypto::Crypto()
{

}

bool Crypto::SetPrivateKey(uint8_t* key, uint32_t len)
{
    if (_privateKey)
        delete _privateKey;
    
    if (!_privateKey)
        return false;

    _privateKey = new uint8_t[len];
    memcpy(_privateKey, key, len);

    return true;
}

bool Crypto::SetPublicKey(uint8_t* key, uint32_t len)
{
    if (_publicKey)
        delete _publicKey;
    
    if (!_publicKey)
        return false;

    _publicKey = new uint8_t[len];
    memcpy(_publicKey, key, len);

    return true;
}

bool Crypto::Encrypt(uint8_t* message, uint32_t len)
{
    FreeInputOutput();

    _output = new uint8_t[len];

    if (!_output)
        return false;

    memcpy(_output, message, len);
    _outputSize = len;
    return true;
}

bool Crypto::Decrypt(uint8_t* message, uint32_t len)
{
    FreeInputOutput();

    _output = new uint8_t[len];

    if (!_output)
        return false;

    memcpy(_output, message, len);
    _outputSize = len;
    return true;
}

uint32_t Crypto::GetOutputSize()
{
    return _outputSize;
}

uint32_t Crypto::GetOutput(uint8_t* out)
{
    memcpy(out, _output, _outputSize);
    return _outputSize;
}

Crypto::~Crypto()
{
    FreeInputOutput();
    if (_privateKey)
        delete _privateKey;
    if (_publicKey)
        delete _publicKey;
}