#pragma once

#include "../i2c/master.hpp"

namespace bh1750fvi{

class light_sensor{
	i2c::master& i2c;

	const uint8_t address;

public:
	light_sensor(i2c::master& i2c, bool addressLow = false) :
			i2c(i2c),
			address(addressLow ? 0x23 : 0x6c)
	{}

	/**
	 * @brief Command light sensor to start illuminance measurement.
	 * Measurement is done in high resolution mode with resolution of 1 lux.
	 * Measurement takes 180 ms, result should be read after that time period.
	 * @return true in case of success.
	 * @return false in case of error.
	 */
	bool start_measurement();

	/**
	 * @brief Read illuminance measurement.
	 * @return Illuminance from 0 to 65535 in luxes, [lx].
	 * @return Negative number means error.
	 */
	int32_t read_measurement();
};

}
