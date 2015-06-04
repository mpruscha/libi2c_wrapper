/* inspired by i2cdevlib */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

#include "I2CManager.h"


uint16_t sr1_mask = 0;
uint16_t sr2_mask = 0;


I2CManager::I2CManager(uint8_t device_address) {

	device = device_address;

}

void I2CManager::setup(void) {

	rcc_periph_clock_enable(RCC_I2C1);

	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO8 | GPIO9);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO8 | GPIO9);

    // Force reset if the bus is stuck in the BUSY state
    if (I2C_SR2(I2C1) & I2C_SR2_BUSY) {
        I2C_CR1(I2C1) |= I2C_CR1_SWRST;
        I2C_CR1(I2C1) &= ~I2C_CR1_SWRST;
    }

	/* i2c peripheral runs on APB1, and we set the APB1 frequency to 21MHz in clock_setup() */
	i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_21MHZ);

	i2c_set_fast_mode(I2C1);

	/* we would like to have 400kHz, that is 21Mhz/400kHz=52.5. Closest: 53, gives 396226Hz~400kHz i2c clock */
	i2c_set_ccr(I2C1, 90);

	i2c_peripheral_enable(I2C1);

}

void I2CManager::enable(void) {

	i2c_peripheral_enable(I2C1);

}

uint16_t I2CManager::readBytes(uint8_t slave_address, uint8_t reg_addr, uint8_t *byte_read) {

	uint8_t slave_address_7bit = slave_address; // << 1;

	// wait until busy flag is off
	while ((I2C_SR2(I2C1) & I2C_SR2_BUSY));

	// disable pos
	I2C_CR1(I2C1) &= ~I2C_CR1_POS;

	i2c_enable_ack(I2C1);

	sendStart();

	sendSlaveAddress(slave_address_7bit, I2C_WRITE);

	/* send register to read from */
	sendData(reg_addr);

	// generate restart
	sendStart();

	sendSlaveAddress(slave_address_7bit, I2C_READ);

	//disable ack
	i2c_disable_ack(I2C1);

	//clear addr flag
	I2C_SR1(I2C1) &= ~I2C_SR1_ADDR;

	i2c_send_stop(I2C1);

	//wait until rxne flag is set
	while (!(I2C_SR1(I2C1) & I2C_SR1_RxNE))


//    *byte_read = (uint8_t)(I2C_DR(I2C1)); /* MSB */



 	*byte_read = i2c_get_data(I2C1);

	return (uint16_t) I2C_SR1(I2C1);

}

uint8_t I2CManager::nAck()
{
	I2C_CR1(I2C1) &= ~I2C_CR1_ACK;

	return 0;
}

uint8_t I2CManager::sendData(uint8_t data)
{

	i2c_send_data(I2C1, data); /* temperature register */

	waitWrite();

	return 0;
}


uint8_t I2CManager::sendStart()
{
	// wait until busy flag is off
	while ((I2C_SR2(I2C1) & I2C_SR2_BUSY));

	i2c_send_start(I2C1);

	/* Waiting for START is send and switched to master mode. */
	waitStart();

	return 0;

}

uint8_t I2CManager::sendSlaveAddress(uint8_t devAddr, uint8_t read_write)
{
	i2c_send_7bit_address(I2C1, devAddr, read_write);

	if(read_write == I2C_WRITE)
	{
		waitAddrWrite();
	}
	else if (read_write == I2C_READ)
	{
		waitAddrRead();
	}

	return 0;
}



/***** Wait functions  ***********************************************************/

uint8_t I2CManager::waitStart()
{
    sr1_mask = I2C_SR1_SB;
    sr2_mask = I2C_SR2_MSL | I2C_SR2_BUSY;

	while (!(I2C_SR1(I2C1) & sr1_mask));
	while (!(I2C_SR2(I2C1) & sr2_mask));

	return 0;
}

uint8_t I2CManager::waitAddrRead()
{
	sr1_mask = I2C_SR1_ADDR;
	sr2_mask = I2C_SR2_MSL | I2C_SR2_BUSY;

	while (!(I2C_SR1(I2C1) & sr1_mask));
	while (!(I2C_SR2(I2C1) & sr2_mask));

	return 0;
}

uint8_t I2CManager::waitAddrWrite()
{
    sr1_mask = I2C_SR1_ADDR | I2C_SR1_TxE;
	sr2_mask = I2C_SR2_MSL | I2C_SR2_BUSY | I2C_SR2_TRA;

	while (!(I2C_SR1(I2C1) & sr1_mask));
	while (!(I2C_SR2(I2C1) & sr2_mask));

	return 0;
}


uint8_t I2CManager::waitWrite()
{
    sr1_mask = I2C_SR1_BTF | I2C_SR1_TxE;
    sr2_mask = I2C_SR2_MSL | I2C_SR2_BUSY | I2C_SR2_TRA;

	while (!(I2C_SR1(I2C1) & sr1_mask));
	while (!(I2C_SR2(I2C1) & sr2_mask));

	return 0;
}


uint8_t I2CManager::waitRead()
{
    sr1_mask = I2C_SR1_RxNE;
    sr2_mask = I2C_SR2_MSL | I2C_SR2_BUSY;

	while (!(I2C_SR1(I2C1) & sr1_mask));
	while (!(I2C_SR2(I2C1) & sr2_mask));

	return 0;
}


/*************** Interface **********************************************************/

// sets i2c device, on STM32/libopencm3 e.g. I2C1
uint8_t I2CManager::setDevice(uint8_t device_address)
{

	device = device_address;

	return 0;
}

// gets i2c device, on STM32/libopencm3 e.g. I2C1
uint8_t I2CManager::getDevice()
{
	return device;
}
