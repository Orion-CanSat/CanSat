#ifndef __ORION_UTILITIES_CRYPTOS_CRYPTO_H__
#define __ORION_UTILITIES_CRYPTOS_CRYPTO_H__

#include "Orion.h"

#include <stdint.h>

namespace Orion
{
    namespace Utilities
    {
        namespace Cryptos
        {
            class Crypto
            {
                protected:
                    uint8_t* _privateKey = nullptr;
                    uint8_t* _publicKey = nullptr;

                    uint8_t* _input = nullptr;
                    uint32_t _inputSize = 0;
                    uint8_t* _output = nullptr;
                    uint32_t _outputSize = 0;

                    void* _cipher;

                    bool _initialized = false;

                    void FreeInputOutput();

                public:
                    Crypto();

                    bool SetPrivateKey(uint8_t* key, uint32_t len);
                    bool SetPublicKey(uint8_t* key, uint32_t len);

                    virtual bool Encrypt(uint8_t* message, uint32_t len);
                    virtual bool Decrypt(uint8_t* message, uint32_t len);

                    uint32_t GetOutputSize();
                    uint32_t GetOutput(uint8_t* out);

                    ~Crypto();
            };
        }
    }
}

#endif//__ORION_UTILITIES_CRYPTOS_CRYPTO_H__