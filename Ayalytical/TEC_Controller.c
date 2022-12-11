/*
 * TEC_Controller.c
 *
 * Created: 10/16/2019 12:18:46 PM
 *  Author: adam
 */ 

#include "delay.h"
#include "ioport.h"
#include "pwm.h"
#include "TEC_Controller.h"

//Ayalytical board uses the TEC in parallel full bridge mode
//OC ADJ set to 4.1A before shutdown based on 68kohm resistor used

pwm_config TECPWMA_config; //TEC PWM A configuration
pwm_config TECPWMB_config; //TEC PWM B configuration
pwm_config TECFAN_config; //TEC fan configuration

//Initializes microcontroller pins for TEC controller
void init_TEC_controller(void)
{
	//Datasheet recommends holding reset lines low on startup
	ioport_configure_pin(_TEC_RST_A, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(_TEC_RST_B, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	
	//Fault and over temp warning pins
	ioport_configure_pin(_TEC_FAULT, IOPORT_DIR_INPUT);
	ioport_configure_pin(_TEC_OTW, IOPORT_DIR_INPUT);
	
	ioport_configure_pin(_FAN_FR_STBY, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	
	//PWM control pins, initialized to 0Hz
	pwm_init(&TECPWMA_config, TEC_PWM_A_TC, TEC_PWM_A_CHAN, 0);
	pwm_init(&TECPWMB_config, TEC_PWM_B_TC, TEC_PWM_B_CHAN, 0);
	pwm_init(&TECFAN_config, TEC_FAN_TC, TEC_FAN_CHAN, 0);
	
	ioport_set_pin_high(_TEC_RST_A);
	ioport_set_pin_high(_TEC_RST_B);
}

void disable_TEC_controller(void)
{
	//Datasheet recommends holding reset lines low on power down to prevent unknown state transitions
	ioport_set_pin_low(_TEC_RST_A);
	ioport_set_pin_low(_TEC_RST_B);
	
	//Disable pwm's
	pwm_stop(&TECPWMA_config);
	pwm_stop(&TECPWMB_config);
	pwm_stop(&TECFAN_config);
}

void set_TEC_pwm_A_duty(uint8_t duty)
{
	pwm_start(&TECPWMA_config, duty);
}

void set_TEC_pwm_A_freq(uint32_t freq)
{
	//pwm_set_frequency(&TECPWMA_config, freq);
	pwm_set_TEC_frequency_32bit(&TECPWMA_config, freq);
}

void enable_TEC_A(void)
{
	ioport_set_pin_high(TEC_PWM_A);
}

void disable_TEC_A(void)
{
	ioport_set_pin_low(TEC_PWM_A);
}

void stop_TEC_pwm_A(void)
{
	pwm_stop(&TECPWMA_config);
}

void set_TEC_pwm_B_duty(uint8_t duty)
{
	pwm_start(&TECPWMB_config, duty);
}

void set_TEC_pwm_B_freq(uint32_t freq)
{
	//pwm_set_frequency(&TECPWMB_config, freq);
	pwm_set_TEC_frequency_32bit(&TECPWMB_config, freq);
}

void enable_TEC_B(void)
{
	ioport_set_pin_high(TEC_PWM_B);
}

void disable_TEC_B(void)
{
	ioport_set_pin_low(TEC_PWM_B);
}

void stop_TEC_pwm_B(void)
{
	pwm_stop(&TECPWMB_config);
}

void set_TEC_fan_duty(uint8_t duty)
{
	pwm_set_duty_cycle_percent(&TECFAN_config, duty);
}

void set_TEC_fan_freq(uint16_t freq)
{
	pwm_set_frequency(&TECFAN_config, freq);
}

uint8_t get_TEC_fault_status(void)
{
	//Read and return fault pin of TEC
	return !ioport_get_pin_level(_TEC_FAULT);
}

uint8_t get_TEC_OTW_status(void)
{
	//Read and return OTW pin of TEC
	return !ioport_get_pin_level(_TEC_OTW);
}

void reset_TEC_A(void)
{
	//Send reset pulse to TEC A/B
	ioport_set_pin_low(_TEC_RST_A);
	delay_ms(50);
	ioport_set_pin_high(_TEC_RST_A);
}

void reset_TEC_B(void)
{
	//Send reset pulse to TEC C/D
	ioport_set_pin_low(_TEC_RST_B);
	delay_ms(50);
	ioport_set_pin_high(_TEC_RST_B);
}

void recover_from_OTSD(void)
{
	//Send both reset pulses
	reset_TEC_A();
	reset_TEC_B();
}

void clear_fan_fault(void)
{
	ioport_set_pin_low(_FAN_FR_STBY);
	delay_ms(50);
	ioport_set_pin_high(_FAN_FR_STBY);
}

void enter_fan_standby(void)
{
	//Control chip will enter standby if FR_STBY is low and all inputs are low
	ioport_set_pin_low(_FAN_FR_STBY);
	ioport_set_pin_low(FAN_PWM);
}

//Same as pwm_set_frequency in HAL but with uint32_t input
void pwm_set_TEC_frequency_32bit(struct pwm_config *config, uint32_t freq_hz)
{
	uint32_t cpu_hz = sysclk_get_cpu_hz();
	uint16_t smallest_div;
	uint16_t dividor;

	/* Avoid division by zero. */
	Assert(freq_hz != 0);

	/* Calculate the smallest divider for the requested frequency
	   related to the CPU frequency */
	smallest_div = cpu_hz / freq_hz / 0xFFFF;
	if (smallest_div < 1) {
		dividor = 1;
		config->clk_sel = PWM_CLK_DIV1;
	} else if (smallest_div < 2) {
		dividor = 2;
		config->clk_sel = PWM_CLK_DIV2;
	} else if (smallest_div < 4) {
		dividor = 4;
		config->clk_sel = PWM_CLK_DIV4;
	} else if (smallest_div < 8) {
		dividor = 8;
		config->clk_sel = PWM_CLK_DIV8;
	} else if (smallest_div < 64) {
		dividor = 64;
		config->clk_sel = PWM_CLK_DIV64;
	} else if (smallest_div < 256) {
		dividor = 256;
		config->clk_sel = PWM_CLK_DIV256;
	} else {
		dividor = 1024;
		config->clk_sel = PWM_CLK_DIV1024;
	}

	/* Calculate the period from the just found divider */
	config->period = cpu_hz / dividor / freq_hz;

	/* Make sure our period is at least 100 ticks so we are able to provide
	   a full range (0-100% duty cycle */
	if (config->period < 100) {
		/* The period is too short. */
		config->clk_sel = PWM_CLK_OFF;
		config->period = 0;
		Assert(false);
	}
}