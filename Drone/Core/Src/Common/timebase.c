#include "timebase.h"
#include "stm32f4xx.h"

static uint32_t lastCycles;
static uint32_t nowUs;
static uint32_t cycleRemainder;

static uint32_t Time_AdvanceLocked(uint32_t cycles){
	uint32_t cyclesPerUs = SystemCoreClock / 1000000U;
	uint32_t d = cycles - lastCycles;
	lastCycles = cycles;

	nowUs          += d / cyclesPerUs;
	cycleRemainder += d % cyclesPerUs;
	if(cycleRemainder >= cyclesPerUs){
		cycleRemainder -= cyclesPerUs;
		nowUs++;
	}
	return nowUs;
}

uint32_t Time_Us(void){
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	uint32_t us = Time_AdvanceLocked(DWT->CYCCNT);
	__set_PRIMASK(primask);
	return us;
}

uint32_t Time_UsAt(uint32_t cycles){
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	uint32_t now = DWT->CYCCNT;
	uint32_t us  = Time_AdvanceLocked(now);
	uint32_t ago = (now - cycles) / (SystemCoreClock / 1000000U);
	__set_PRIMASK(primask);
	return us - ago;
}
