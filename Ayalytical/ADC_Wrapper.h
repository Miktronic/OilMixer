/*
 * ADC_Wrapper.h
 *
 * Created: 8/16/2019 3:26:12 PM
 *  Author: adam
 */ 


#ifndef ADC_WRAPPER_H_
#define ADC_WRAPPER_H_

//////////////////////////////////////////////////////////////////////////
// ADC channel definitions
//////////////////////////////////////////////////////////////////////////
#define SYSTEM_ADC					ADCA
#define FAN_CS_ADC_CHAN				ADC_CH0
#define FAN_CS_ADC_POS_PIN			ADCCH_POS_PIN0
#define FAN_CS_ADC_NEG_PIN			ADCCH_NEG_NONE
#define TEMP_SENSE_ADC_CHAN			ADC_CH1
#define TEMP_SENSE_ADC_POS_PIN		ADCCH_POS_PIN1
#define TEMP_SENSE_ADC_NEG_PIN		ADCCH_NEG_NONE
#define PUMP_SENSE_ADC_CHAN			ADC_CH2
#define PUMP_SENSE_ADC_POS_PIN		ADCCH_POS_PIN6
#define PUMP_SENSE_ADC_NEG_PIN		ADCCH_NEG_NONE
#define HEATER_SENSE_ADC_CHAN		ADC_CH3
#define HEATER_SENSE_ADC_POS_PIN	ADCCH_POS_PIN7
#define HEATER_SENSE_ADC_NEG_PIN	ADCCH_NEG_NONE
//
//
//#define TEMP_SENSE_ADC			ADCA		//Analog temperature sensor ADC pin
////#define TEMP_SENSE_ADC_CHAN		ADCCH_POS_PIN1
//#define TEMP_SENSE_ADC_CHAN		ADC_CH1
//#define VCC_ADC					ADCA		//Vcc feedback ADC pin
////#define VCC_ADC_CHAN			ADCCH_POS_PIN2
//#define VCC_ADC_CHAN			ADC_CH2
//#define PUMP_SENSE_ADC			ADCA
//#define PUMP_SENSE_ADC_CHAN		ADCCH_POS_PIN6
//#define HEATER_SENSE_ADC		ADCA
//#define HEATER_SENSE_ADC_CHAN	ADCCH_POS_PIN7

//#define FAN_CS_ADC				ADCA
////#define FAN_CS_ADC_CHAN			ADCCH_POS_PIN0
//#define FAN_CS_ADC_CHAN			ADC_CH0

//////////////////////////////////////////////////////////////////////////
// Function prototypes
//////////////////////////////////////////////////////////////////////////
void init_ADC(void);
void enable_ADC(void);
void disable_ADC(void);

#endif /* ADC_WRAPPER_H_ */