/*
 * RTD_Wrapper.h
 *
 * Created: 10/18/2019 8:52:49 AM
 *  Author: adam
 */ 


#ifndef RTD_WRAPPER_H_
#define RTD_WRAPPER_H_

//////////////////////////////////////////////////////////////////////////
// RTD chip register addresses
//////////////////////////////////////////////////////////////////////////
enum RTD_REG_ADDR
{
	READ_CONFIG =			0x00,
	READ_MSB =				0x01,
	READ_LSB =				0x02,
	READ_HIGH_FAULT_MSB =	0x03,
	READ_HIGH_FAULT_LSB =	0x04,
	READ_LOW_FAULT_MSB =	0x05,
	READ_LOW_FAULT_LSB =	0x06,
	READ_FAULT_STATUS =		0x07,
	WRITE_CONFIG =			0x80,
	WRITE_HIGH_FAULT_MSB =	0x83,
	WRITE_HIGH_FAULT_LSB =	0x84,
	WRITE_LOW_FAULT_MSB =	0x85,
	WRITE_LOW_FAULT_LSB =	0x86
};

enum RTD_CONFIG_REG
{
	VBIAS =			0x80,
	CONVMODE =		0x40, //1=Auto, 0=Normally off
	ONE_SHOT =		0x20, //1=Auto clear
	MODE_3WIRE =	0x10, //1=3-wire mode, 0=2/4 wire mode
	FAULT_DET1 =	0x08, //00=no action, 01=automatic delay, 10=manual delay cycle 1, 11=manual delay cycle 2
	FAULT_DET0 =	0x04,
	FAULT_CLEAR =	0x02, //1=Clear
	FILTER_SEL =	0x01, //1=50Hz, 0=60Hz
};

//////////////////////////////////////////////////////////////////////////
// Function prototypes
//////////////////////////////////////////////////////////////////////////
void init_RTD_sensor(void);
void read_RTD_config_reg(void);
uint16_t get_RTD_sensor_reading(void);
void set_RTD_high_fault_thresh(uint16_t highFaultThresh);
void set_RTD_low_fault_thresh(uint16_t lowFaultThresh);
uint8_t check_fault_status(void);
void clear_fault_status(void);

#endif /* RTD_WRAPPER_H_ */