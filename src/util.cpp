#include "util.hpp"

extern"C"{
#include <user_interface.h>
}

uint32_t ICACHE_FLASH_ATTR get_cpu_ticks_per_second(){
	auto freq = system_get_cpu_freq();
	switch(freq){
		default:
			os_printf("WARNING: unknown CPU frequency: %d. Will be using 80 MHz for calculations.\n", freq);
			// fall-through
		case 80:
			return 80 * 1000000;
		case 160:
			return 160 * 1000000;
	}
}

void ICACHE_FLASH_ATTR __cxa_pure_virtual() {
	os_printf("FATAL: pure virtual function is called");
	while(1){}
}
