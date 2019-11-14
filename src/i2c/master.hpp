#pragma once

#include <array>

#include "../util.hpp"

namespace i2c{

enum class status{
	ok,
	bus_busy,
	nack,
	clock_stretching_timeout
};

class master{
	uint32_t time_quant_cpu_ticks;
	uint32_t clock_stretch_timeout_cpu_ticks;
	uint16_t scl_bit_mask;
	uint16_t sda_bit_mask;

	// return false if bus is free
	// return true if bus is busy
	bool wait_for_free_bus();

	void busy_loop(uint32_t num_cpu_ticks);

	/**
	 * @return true in case timeout was hit.
	 * @return false if SCL went hight before timeout was reached.
	 */
	bool busy_loop_while_scl_is_low(uint32_t timeout_cpu_ticks);

	void set_scl_low();
	void set_scl_high();
	void set_sda_low();
	void set_sda_high();

	bool read_sda();
	bool read_scl();

	void send_start();
	void send_stop();
	void release_line();
	status send_byte(uint8_t byte);

public:
	master(uint32_t frequency_hz, uint8_t scl_gpio_number, uint8_t sda_gpio_number);

	status send(
			uint8_t address,
			std::array<uint8_t, 1>::const_iterator begin,
			std::array<uint8_t, 1>::const_iterator end,
			bool send_stop = true
		);

	status recv(
			uint8_t address,
			std::array<uint8_t, 1>::iterator begin,
			std::array<uint8_t, 1>::iterator end,
			bool send_stop = true
		);

	// Send and then receive
	status send_recv(
			uint8_t address,
			std::array<uint8_t, 1>::const_iterator sendBegin,
			std::array<uint8_t, 1>::const_iterator sendEnd,
			std::array<uint8_t, 1>::iterator recvBegin,
			std::array<uint8_t, 1>::iterator recvEnd
		);
};

}
