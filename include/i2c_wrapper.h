
#include <stdint.h>

class i2c_wrapper {

    public:
		i2c_wrapper(uint8_t device_address);

		uint8_t device;

        static void initialize();
        static void enable(bool isEnabled);

        int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout=i2c_wrapper::readTimeout);
        int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout=i2c_wrapper::readTimeout);
        int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout=i2c_wrapper::readTimeout);
        int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout=i2c_wrapper::readTimeout);


        static bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
        static bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
        static bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);

        static uint16_t readTimeout;

    private:



        int8_t sendStart();
        int8_t sendSlaveAddress(uint8_t devAddr);
        int8_t getDevice();
        int8_t setDevice(uint8_t device_address);
        int8_t sendData(uint8_t data);
        int8_t nAck();

//        static I2C_TransferReturn_TypeDef transfer(I2C_TransferSeq_TypeDef *seq, uint16_t timeout=i2c_wrapper::readTimeout);
};
