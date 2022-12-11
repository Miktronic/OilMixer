/*
 * WDT_wrapper.c
 *
 * Created: 12/17/2019 12:41:09 PM
 *  Author: adam
 */ 

#include "wdt.h"
#include "WDT_wrapper.h"

void enable_WDT(void)
{
	wdt_enable();
}

void disable_WDT(void)
{
	wdt_disable();
}

void set_WDT_timeout(uint8_t timeoutIndex)
{
	wdt_set_timeout_period(timeoutIndex);
}

void reset_WDT(void)
{
	wdt_reset();
}