/*
 * ADC_Wrapper.c
 *
 * Created: 8/16/2019 3:26:01 PM
 *  Author: adam
 */ 

#include "adc.h"
#include "ADC_Wrapper.h"

void init_ADC(void)
{
	struct adc_config ADCConfig;
	struct adc_channel_config ADCChanConfig;
	
	adc_read_configuration(&SYSTEM_ADC, &ADCConfig);				// Read current configuration of ADC module
	adcch_read_configuration(&SYSTEM_ADC, ADC_CH0, &ADCChanConfig); // Read current channel configuration
	
	adcch_set_input(&ADCChanConfig, ADCCH_POS_PIN1, ADCCH_NEG_NONE, 1); // PA1 positive, no negative
	
	adc_set_conversion_parameters(&ADCConfig, ADC_SIGN_ON, ADC_RES_12, ADC_REF_VCC);	// Signed, 12-bit read, Vcc/1.6 pin as reference
	adc_set_conversion_trigger(&ADCConfig, ADC_TRIG_FREERUN_SWEEP, 1, 0);				// Freerun mode
	adc_set_clock_rate(&ADCConfig, 62500UL); // Sets ADC prescaler to be 512 -> 32MHz sysclk / 62500 = 512
	adc_write_configuration(&SYSTEM_ADC, &ADCConfig);
	adcch_write_configuration(&SYSTEM_ADC, ADC_CH0, &ADCChanConfig);	
		
	//Enable ADC
	adc_enable(&SYSTEM_ADC);
}

void enable_ADC(void)
{
	adc_enable(&SYSTEM_ADC);
}

void disable_ADC(void)
{
	adc_disable(&SYSTEM_ADC);
}