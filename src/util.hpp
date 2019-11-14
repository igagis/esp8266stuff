#pragma once

extern "C"{
#include <osapi.h>
}

#define ASSERT_ALWAYS(x) if(!(x)){os_printf("ASSERT: failed at %s:%d", __FILE__, __LINE__); while(true){}} //infinite loop should trigger the watchdog

#ifdef DEBUG
#	define ASSERT(x) ASSERT_ALWAYS(x)
#else
#	define ASSERT(x)
#endif

template <class T> class singleton{
protected:
	ICACHE_FLASH_ATTR singleton(){
		T::instance = static_cast<T*>(this);
	}
public:
	static T& ICACHE_FLASH_ATTR inst(){
		ASSERT(T::isntance)
		return *T::instance;
	}
};

/**
 * @brief Get current CPU ticks.
 *
 * This function returns count of CPU ticks, which are constantly incremented at CPU frequency.
 *
 * @return Current CPU ticks.
 */
inline uint32_t ICACHE_FLASH_ATTR get_cpu_ticks(){
    uint32_t r;

    asm volatile ("rsr %0, ccount" : "=r"(r));
    return r;
}

/**
 * @brief Get number of CPU ticks per second.
 *
 * Effectively returns the current CPU frequency in Hertz.
 *
 * @return CPU frequency, Hz.
 */
uint32_t get_cpu_ticks_per_second();

/**
 * @brief Disable all interrupts.
 *
 * @return old interrupt level.
 */
inline uint32_t ICACHE_FLASH_ATTR disable_interrupts(){
    uint32_t level;

	__asm__ __volatile__("rsil %0,15 ; esync" : "=a"(level));

    return level;
}

/**
 * @brief enable interrupts up to given interrupt level.
 *
 * @param up_to_level - interrupt level up to which to enable the interrupts.
 */
inline void ICACHE_FLASH_ATTR enable_interrupts(uint32_t up_to_level){
	__asm__ __volatile__("wsr %0,ps ; esync"::"a"(up_to_level): "memory");
}

/**
 * @brief Convenience class to disable/enable interrupts.
 */
class interrupts_guard{
	uint32_t interrupt_level;
public:
	ICACHE_FLASH_ATTR interrupts_guard(){
		this->interrupt_level = disable_interrupts();
	}

	ICACHE_FLASH_ATTR ~interrupts_guard(){
		enable_interrupts(this->interrupt_level);
	}
};

// this is a default function for pure virtual functions in C++.
extern"C" void __cxa_pure_virtual();
