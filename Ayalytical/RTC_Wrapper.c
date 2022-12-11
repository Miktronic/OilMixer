/*
 * RTC_Wrapper.c
 *
 * Created: 9/5/2019 8:17:56 AM
 *  Author: adam
 */ 

#include "pmic.h"
#include "rtc.h"
#include "sysclk.h"

#include "RTC_Wrapper.h"

//static void update_sys_time_ms(uint32_t time)
//{
	//rtc_set_alarm(time);
//}

void init_RTC(void)
{
	//Setup also includes appropriate defines in conf_rtc.h and conf_clock.h
	//Setup for a 1.024kHz clock generated from the 32kHz external clock source
	sysclk_init();
	rtc_init();
}

//void set_RTC_alarm(uint32_t setTime)
//{
	////
//}