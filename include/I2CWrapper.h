
#include <stdint.h>

class I2CWrapper {

    public:
		I2CWrapper(uint8_t device_address);

		uint8_t device;

        void setup(void);
        void enable(void);

        uint16_t readBytes(uint8_t slave_address, uint8_t reg_addr, uint8_t *byte_read);

        static uint16_t readTimeout;

    private:

        void readByte(uint8_t* byte_read);


        uint8_t sendStart();
        uint8_t sendSlaveAddress(uint8_t devAddr, uint8_t read_write);
        uint8_t getDevice();
        uint8_t setDevice(uint8_t device_address);
        uint8_t sendData(uint8_t data);
        uint8_t nAck();
		uint8_t waitAddrRead();
		uint8_t waitAddrWrite();
		uint8_t waitStart();
		uint8_t waitWrite();
		uint8_t waitRead();


};
