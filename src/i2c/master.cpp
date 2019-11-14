#include "master.hpp"

extern "C"{
#include <osapi.h>
#include <user_interface.h>

#include "../espmissingincludes.h"
}

using namespace i2c;

namespace{
const uint32_t clock_stretch_timeout_usec = 2000;

uint32_t ICACHE_FLASH_ATTR gpio_to_func(uint8_t gpio){
	switch(gpio){
		case 0:
			return FUNC_GPIO0;
		case 1:
			return FUNC_GPIO1;
		case 2:
			return FUNC_GPIO2;
		case 3:
			return FUNC_GPIO3;
		case 4:
			return FUNC_GPIO4;
		case 5:
			return FUNC_GPIO5;
		case 6:
		case 7:
		case 8:
		case 11:
		default:
			return 0;
		case 9:
			return FUNC_GPIO9;
		case 10:
			return FUNC_GPIO10;
		case 12:
			return FUNC_GPIO12;
		case 13:
			return FUNC_GPIO13;
		case 14:
			return FUNC_GPIO14;
		case 15:
			return FUNC_GPIO15;
	}
}

uint32_t ICACHE_FLASH_ATTR gpio_to_mux(uint8_t gpio){
	switch(gpio){
		case 0:
			return PERIPHS_IO_MUX_GPIO0_U;
		case 1:
			return PERIPHS_IO_MUX_U0TXD_U;
		case 2:
			return PERIPHS_IO_MUX_GPIO2_U;
		case 3:
			return PERIPHS_IO_MUX_U0RXD_U;
		case 4:
			return PERIPHS_IO_MUX_GPIO4_U;
		case 5:
			return PERIPHS_IO_MUX_GPIO5_U;
		case 9:
			return PERIPHS_IO_MUX_SD_DATA2_U;
		case 10:
			return PERIPHS_IO_MUX_SD_DATA3_U;
		case 12:
			return PERIPHS_IO_MUX_MTDI_U;
		case 13:
			return PERIPHS_IO_MUX_MTCK_U;
		case 14:
			return PERIPHS_IO_MUX_MTMS_U;
		case 15:
			return PERIPHS_IO_MUX_MTDO_U;
		default:
		case 6:
		case 7:
		case 8:
		case 11:
			return 0;
	}
}

}

inline void ICACHE_FLASH_ATTR master::busy_loop(uint32_t num_cpu_ticks){
	auto start = get_cpu_ticks();
	while(get_cpu_ticks() - start < num_cpu_ticks){}
}

inline void ICACHE_FLASH_ATTR master::set_scl_low(){
	GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, this->scl_bit_mask);
}

inline void ICACHE_FLASH_ATTR master::set_scl_high(){
	GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, this->scl_bit_mask);
}

inline void ICACHE_FLASH_ATTR master::set_sda_low(){
	GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, this->sda_bit_mask);
}

inline void ICACHE_FLASH_ATTR master::set_sda_high(){
	GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, this->sda_bit_mask);
}

// return false if bus is free
// return true if bus is busy
inline bool ICACHE_FLASH_ATTR master::wait_for_free_bus(){
	uint32_t pin_mask = this->sda_bit_mask | this->scl_bit_mask;
	uint32_t start = get_cpu_ticks();

	while(get_cpu_ticks() - start < this->clock_stretch_timeout_cpu_ticks){
		uint32_t gpio = GPIO_REG_READ(GPIO_IN_ADDRESS);
		if((gpio & pin_mask) == pin_mask){
//			os_printf("!!! i2c bus free!!! i = %d\n", i);
			return false;
		}
	};
	return true;
}

inline bool ICACHE_FLASH_ATTR master::read_scl(){
	uint32_t gpio = GPIO_REG_READ(GPIO_IN_ADDRESS);
	return (gpio & this->scl_bit_mask) != 0;
}

inline bool ICACHE_FLASH_ATTR master::read_sda(){
	uint32_t gpio = GPIO_REG_READ(GPIO_IN_ADDRESS);
	return (gpio & this->sda_bit_mask) != 0;
}

inline bool ICACHE_FLASH_ATTR master::busy_loop_while_scl_is_low(uint32_t timeout_cpu_ticks){
	auto start = get_cpu_ticks();

	do{
		if(this->read_scl()){
			return false;
		}
	}while(get_cpu_ticks() - start < timeout_cpu_ticks);

	return true;
}

inline void ICACHE_FLASH_ATTR master::send_start(){
	// SCL must be high already
	this->busy_loop(this->time_quant_cpu_ticks / 2);
	this->set_sda_low();
	this->busy_loop(this->time_quant_cpu_ticks / 2);
}

inline void ICACHE_FLASH_ATTR master::send_stop(){
	this->set_scl_low();
	this->busy_loop(this->time_quant_cpu_ticks / 2);
	this->set_sda_low();
	this->busy_loop(this->time_quant_cpu_ticks / 2);

	this->set_scl_high();
	if(this->busy_loop_while_scl_is_low(this->clock_stretch_timeout_cpu_ticks)){
		os_printf("WARNING: I2C clock stretch timeout exceeded whien sending stop\n");
	}
	this->busy_loop(this->time_quant_cpu_ticks / 2);
	this->set_sda_high();
	this->busy_loop(this->time_quant_cpu_ticks / 2);
}

inline void ICACHE_FLASH_ATTR master::release_line(){
	this->set_scl_low();
	// Give slave some time to release SDA if it is controlled by slave due to sending ACK/NACK to master.
	this->busy_loop(this->time_quant_cpu_ticks);

	ASSERT(this->read_sda())
	this->set_scl_high();
	if(this->busy_loop_while_scl_is_low(this->clock_stretch_timeout_cpu_ticks)){
		os_printf("WARNING: I2C clock stretching timeout exceeded while releasing the line\n");
	}
}

inline status ICACHE_FLASH_ATTR master::send_byte(uint8_t byte){
	for(uint8_t mask = 0x80; mask; mask >>= 1){
		// set bit to SDA in the middle of low SCL cycle
		this->set_scl_low();

		this->busy_loop(this->time_quant_cpu_ticks / 2);

		if(byte & mask){
			this->set_sda_high();
		}else{
			this->set_sda_low();
		}

		this->busy_loop(this->time_quant_cpu_ticks / 2);

		this->set_scl_high();

		// wait while clock is stretched by slave
		if(this->busy_loop_while_scl_is_low(this->clock_stretch_timeout_cpu_ticks)){
			// timeout hit
			this->set_sda_high(); // release SDA line

			return i2c::status::clock_stretching_timeout;
		}
		// stretching ended before timeout

		// we need to give slave time to read the SDA after clock stretching
		this->busy_loop(this->time_quant_cpu_ticks);
	}

	// Check for ACK/NACK

	this->set_scl_low();
	this->busy_loop(this->time_quant_cpu_ticks / 2);
	this->set_sda_high(); // release SDA for slave to take control of it
	this->busy_loop(this->time_quant_cpu_ticks / 2);

	this->set_scl_high();
	// wait while clock is stretched by slave
	if(this->busy_loop_while_scl_is_low(this->clock_stretch_timeout_cpu_ticks)){
		// timeout hit
		return i2c::status::clock_stretching_timeout;
	}
	this->busy_loop(this->time_quant_cpu_ticks / 2);
	bool sda = this->read_sda();
	this->busy_loop(this->time_quant_cpu_ticks / 2);

	if(sda){
		return i2c::status::nack;
	}

	return i2c::status::ok;
}

ICACHE_FLASH_ATTR master::master(uint32_t frequency_hz, uint8_t scl_gpio_number, uint8_t sda_gpio_number){
	this->scl_bit_mask = 1 << scl_gpio_number;
	this->sda_bit_mask = 1 << sda_gpio_number;

	this->time_quant_cpu_ticks = (system_get_cpu_freq() * 1000000 / frequency_hz) / 2;
	this->clock_stretch_timeout_cpu_ticks = clock_stretch_timeout_usec * system_get_cpu_freq();

	// os_printf("this->time_quant_cpu_ticks = %d\n", this->time_quant_cpu_ticks);
	// os_printf("this->clock_stretch_timeout_cpu_ticks = %d\n", this->clock_stretch_timeout_cpu_ticks);

	ETS_GPIO_INTR_DISABLE();

	// Set pins function to GPIO
	PIN_FUNC_SELECT(gpio_to_mux(sda_gpio_number), gpio_to_func(sda_gpio_number));
	PIN_FUNC_SELECT(gpio_to_mux(scl_gpio_number), gpio_to_func(scl_gpio_number));

	// Set SCL and SDA pins to open drain mode
	GPIO_REG_WRITE(GPIO_PIN_ADDR(GPIO_ID_PIN(sda_gpio_number)), GPIO_REG_READ(GPIO_PIN_ADDR(GPIO_ID_PIN(sda_gpio_number))) | GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_ENABLE));
	GPIO_REG_WRITE(GPIO_ENABLE_ADDRESS, GPIO_REG_READ(GPIO_ENABLE_ADDRESS) | this->sda_bit_mask);
	GPIO_REG_WRITE(GPIO_PIN_ADDR(GPIO_ID_PIN(scl_gpio_number)), GPIO_REG_READ(GPIO_PIN_ADDR(GPIO_ID_PIN(scl_gpio_number))) | GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_ENABLE));
	GPIO_REG_WRITE(GPIO_ENABLE_ADDRESS, GPIO_REG_READ(GPIO_ENABLE_ADDRESS) | this->scl_bit_mask);

	ETS_GPIO_INTR_ENABLE();

	// Set SCL and SDA high
	this->set_scl_high();
	this->set_sda_high();

	// give other devices on i2c line to boot up
	// before initializating
	os_delay_us(100000); // 100 ms
}


status ICACHE_FLASH_ATTR master::send(
		uint8_t address,
		std::array<uint8_t, 1>::const_iterator begin,
		std::array<uint8_t, 1>::const_iterator end,
		bool send_stop
	)
{
	if(this->wait_for_free_bus()){
		return i2c::status::bus_busy;
	}

	this->send_start();

	auto status = this->send_byte(address << 1);

	for(auto i = begin; i != end; ++i){
		if(status != i2c::status::ok){
			this->send_stop();
			return status;
		}
		status = this->send_byte(*i);
	}

	if(send_stop){
		this->send_stop();
	}else{
		this->release_line();
	}

	if(status == i2c::status::nack){
		// NACK is ok if it was after sending last byte.
		status = i2c::status::ok;
	}

	return status;
}

status ICACHE_FLASH_ATTR master::recv(
		uint8_t address,
		std::array<uint8_t, 1>::iterator begin,
		std::array<uint8_t, 1>::iterator end,
		bool send_stop
	)
{
	if(this->wait_for_free_bus()){
		return i2c::status::bus_busy;
	}

	this->send_start();

	// send address
	{
		auto status = this->send_byte((address << 1) | 1);
		if(status != i2c::status::ok){
			this->send_stop();
			return status;
		}
	}

	// SCL is high
	// SDA is low (slave acknowledged address receipt)

	for(auto i = begin; i != end;){
		uint8_t byte = 0;

		// At this point: SCL is high; SDA is low either by slave acknowledging address receipt, or by master acknowledging data byte receipt.
		// Release SDA in the middle of low SCL cycle.
		this->set_scl_low();
		this->busy_loop(this->time_quant_cpu_ticks / 2);
		this->set_sda_high(); // release SDA

		for(uint8_t mask = 0x80; mask; mask >>= 1){
			this->busy_loop(this->time_quant_cpu_ticks / 2);
			this->set_scl_high();

			// wait while clock is stretched by slave
			if(this->busy_loop_while_scl_is_low(this->clock_stretch_timeout_cpu_ticks)){
				// timeout hit
				return i2c::status::clock_stretching_timeout;
			}
			// stretching ended before timeout
			// read SDA
			this->busy_loop(this->time_quant_cpu_ticks / 2);
			if(this->read_sda()){
				byte |= mask;
			}
			this->busy_loop(this->time_quant_cpu_ticks / 2);

			this->set_scl_low();
			this->busy_loop(this->time_quant_cpu_ticks / 2);
		}

		*i = byte;
		++i;

		if(i != end){
			// There are more bytes to receive, send ACK.
			this->set_sda_low();
		}else{
			// No more bytes to receive, send NACK.
			this->set_sda_high();
		}
		this->busy_loop(this->time_quant_cpu_ticks / 2);

		this->set_scl_high();

		// wait while clock is stretched by slave
		if(this->busy_loop_while_scl_is_low(this->clock_stretch_timeout_cpu_ticks)){
			// timeout hit
			this->set_sda_high(); // release SDA line

			return i2c::status::clock_stretching_timeout;
		}

		// Give slave time to read SDA.
		this->busy_loop(this->time_quant_cpu_ticks);
	}

	if(send_stop){
		this->send_stop();
	}else{
		this->release_line();
	}

	return i2c::status::ok;
}

status ICACHE_FLASH_ATTR master::send_recv(
		uint8_t address,
		std::array<uint8_t, 1>::const_iterator sendBegin,
		std::array<uint8_t, 1>::const_iterator sendEnd,
		std::array<uint8_t, 1>::iterator recvBegin,
		std::array<uint8_t, 1>::iterator recvEnd
	)
{
	{
		auto status = this->send(address, sendBegin, sendEnd, false);
		if(status != i2c::status::ok){
			os_printf("i2c send_recv ERROR on send: %x\n", status);
			return status;
		}
	}
	{
		auto status = this->recv(address, recvBegin, recvEnd);
		if(status != i2c::status::ok){
			os_printf("i2c send_recv ERROR on recv: %x\n", status);
			return status;
		}
	}

	return i2c::status::ok;
}
