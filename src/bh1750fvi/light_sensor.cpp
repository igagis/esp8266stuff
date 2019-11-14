#include "light_sensor.hpp"

extern "C" {
#include <c_types.h>
#include <osapi.h>
}


using namespace bh1750fvi;


bool ICACHE_FLASH_ATTR light_sensor::start_measurement(){
	std::array<uint8_t, 1> cmd = {{0x10}};

	if(this->i2c.send(this->address, cmd.begin(), cmd.end()) != i2c::status::ok){
		os_printf("light_sensor::startMeasurement(): i2c error\n");
		return false;
	}

	return true;
}

int32_t ICACHE_FLASH_ATTR light_sensor::read_measurement(){
	std::array<uint8_t, 2> result;

	if(this->i2c.recv(this->address, result.begin(), result.end()) != i2c::status::ok){
		os_printf("light_sensor::readMeasurement(): i2c error\n");
		return -1;
	}

	int32_t res = (uint16_t(result[0]) << 8) | uint16_t(result[1]);

	// multiply by factor 1.2, according to BH1750FVI datasheet
	res = res * 12 / 10;

	return res;
}
