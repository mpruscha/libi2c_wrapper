/* inspired by i2cdevlib */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

#include "i2c_wrapper.h"

i2c_wrapper::i2c_wrapper() {
}

void i2c_wrapper::initialize() {

	rcc_periph_clock_enable(RCC_I2C1);

	//	i2c_peripheral_disable(I2C1);

	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO8 | GPIO9);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO8 | GPIO9);

	/* i2c peripheral runs on APB1, and we set the APB1 frequency to 21MHz in clock_setup() */
	i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_21MHZ);

	i2c_set_fast_mode(I2C1);

	/* we would like to have 400kHz, that is 21Mhz/400kHz=52.5. Closest: 53, gives 396226Hz~400kHz i2c clock */
	i2c_set_ccr(I2C1, 30);

}

void i2c_wrapper::enable(bool isEnabled) {

	i2c_peripheral_enable(I2C1);

}

//I2C_TransferReturn_TypeDef i2c_wrapper::transfer(I2C_TransferSeq_TypeDef *seq, uint16_t timeout) {
//
//  I2C_TransferReturn_TypeDef ret;
//  /* Do a polled transfer */
//  ret = I2C_TransferInit(I2C0, seq);
//
//  while (ret == i2cTransferInProgress && timeout--) {
//    ret = I2C_Transfer(I2C0);
//  }
//
//  return(ret);
//
//}
//
//
//uint16_t i2c_wrapper::readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;
//
//
//int8_t i2c_wrapper::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout) {
//    uint8_t b;
//    uint8_t count = readByte(devAddr, regAddr, &b, timeout);
//    *data = b & (1 << bitNum);
//    return count;
//}
//
//int8_t i2c_wrapper::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout) {
//
//	uint8_t count, b;
//    if ((count = readByte(devAddr, regAddr, &b, timeout)) != 0) {
//        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
//        b &= mask;
//        b >>= (bitStart - length + 1);
//        *data = b;
//    }
//    return count;
//}
//
//
//int8_t i2c_wrapper::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout) {
//    return readBytes(devAddr, regAddr, 1, data, timeout);
//}
//
//int8_t i2c_wrapper::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout) {
//
//  I2C_TransferSeq_TypeDef seq;
//
//  uint8_t regid[1];
//
//  seq.addr = devAddr << 1;
//  seq.flags = I2C_FLAG_WRITE_READ;
//
//  /* Select register to be read */
//  regid[0] = regAddr;
//  seq.buf[0].data = regid;
//  seq.buf[0].len = 1;
//
//  /* 1 bytes reg */
//  seq.buf[1].data = data;
//  seq.buf[1].len = length;
//
//  if (transfer(&seq, timeout) == i2cTransferDone) {
//	  return seq.buf[1].len;
//  } else {
//	  return false;
//  }
//
//}
//
//
//bool i2c_wrapper::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
//    uint8_t b;
//    readByte(devAddr, regAddr, &b);
//    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
//    return writeByte(devAddr, regAddr, b);
//}
//
//
//bool i2c_wrapper::writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
//
//    uint8_t b;
//    if (readByte(devAddr, regAddr, &b) != 0) {
//        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
//        data <<= (bitStart - length + 1); // shift data into correct position
//        data &= mask; // zero all non-important bits in data
//        b &= ~(mask); // zero all important bits in existing byte
//        b |= data; // combine data with existing byte
//        return writeByte(devAddr, regAddr, b);
//    } else {
//        return false;
//    }
//}
//
//
//bool i2c_wrapper::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
//
//  I2C_TransferSeq_TypeDef seq;
//
//  uint8_t writeData[3];
//
//  seq.addr = devAddr << 1;
//  seq.flags = I2C_FLAG_WRITE;
//
//  /* Select register to be written */
//  writeData[0] = regAddr;
//  seq.buf[0].data = writeData;
//
//  /* Only 1 byte reg */
//  writeData[1] = data;
//  seq.buf[0].len = 2;
//
//  if (transfer(&seq) == i2cTransferDone) {
//  	  return true;
//    } else {
//  	  return false;
//    }
//
//}
