/*
 * RTD_Wrapper.c
 *
 * Created: 10/18/2019 8:52:34 AM
 *  Author: adam
 */ 

#include <avr/io.h>

#include "SPI_Wrapper.h"
#include "RTD_Wrapper.h"

uint8_t RTDRxBuffer[16];
uint8_t RTDTxBuffer[16];

void init_RTD_sensor(void)
{
	RTDTxBuffer[0] = WRITE_CONFIG;
	RTDTxBuffer[1] = VBIAS | ONE_SHOT | MODE_3WIRE | FAULT_CLEAR; //Setup config (Vbias on, 1-shot, 3-wire, 60Hz)
	
	send_msg(&RTD_SPI_PORT, _RTD_CS, RTDTxBuffer, 2);
}

void read_RTD_config_reg(void)
{	
	uint8_t RTDMSB = 0;
	uint8_t RTDLSB = 0;
	RTDTxBuffer[0] = READ_MSB;
	read_msg(&RTD_SPI_PORT, _RTD_CS, RTDTxBuffer, &RTDMSB);
	RTDTxBuffer[1] = READ_LSB;
	read_msg(&RTD_SPI_PORT, _RTD_CS, RTDTxBuffer, &RTDLSB);
}

uint16_t get_RTD_sensor_reading(void)
{
	uint16_t sensorVal = 0; //Temp reading is a 15-bit value
	
	RTDTxBuffer[0] = WRITE_CONFIG;
	RTDTxBuffer[1] = VBIAS | ONE_SHOT | MODE_3WIRE | FAULT_CLEAR;
	send_msg(&RTD_SPI_PORT, _RTD_CS, RTDTxBuffer, 2);
	
	//Read MSB val
	RTDTxBuffer[0] = READ_MSB;
	read_msg(&RTD_SPI_PORT, _RTD_CS, &RTDTxBuffer[0], &RTDRxBuffer[0]);
	sensorVal = (RTDRxBuffer[0] << 8);
	
	//Read LSB val
	RTDTxBuffer[0] = READ_LSB;
	read_msg(&RTD_SPI_PORT, _RTD_CS, &RTDTxBuffer[0], &RTDRxBuffer[0]);
	sensorVal |= RTDRxBuffer[0];
	
	//Last bit is a fault flag
	if (!(sensorVal & 0x01))
	{
		sensorVal = (sensorVal >> 1); //Shift out fault flag
	}
	else
	{
		sensorVal = 0;
	}
	
	return sensorVal;
}

void set_RTD_high_fault_thresh(uint16_t highFaultThresh)
{
	//Write MSB of high fault threshold
	RTDTxBuffer[0] = WRITE_HIGH_FAULT_MSB;
	RTDTxBuffer[1] = highFaultThresh >> 8;
	send_msg(&RTD_SPI_PORT, _RTD_CS, RTDTxBuffer, 2);
	
	//Write LSB of high fault threshold
	RTDTxBuffer[0] = WRITE_HIGH_FAULT_LSB;
	RTDTxBuffer[1] = highFaultThresh & 0x00FF;
	send_msg(&RTD_SPI_PORT, _RTD_CS, RTDTxBuffer, 2);
}

void set_RTD_low_fault_thresh(uint16_t lowFaultThresh)
{
	//Write MSB of low fault threshold
	RTDTxBuffer[0] = WRITE_LOW_FAULT_MSB;
	RTDTxBuffer[1] = lowFaultThresh >> 8;
	send_msg(&RTD_SPI_PORT, _RTD_CS, RTDTxBuffer, 2);
	
	//Write LSB of low fault threshold
	RTDTxBuffer[0] = WRITE_LOW_FAULT_LSB;
	RTDTxBuffer[1] = lowFaultThresh & 0x00FF;
	send_msg(&RTD_SPI_PORT, _RTD_CS, RTDTxBuffer, 2);
}

uint8_t check_fault_status(void)
{
	uint8_t curFaults = 0x00;
	RTDTxBuffer[0] = READ_CONFIG;
	read_msg(&RTD_SPI_PORT, _RTD_CS, RTDTxBuffer, RTDRxBuffer);
	curFaults = RTDRxBuffer[0];	
	
	return curFaults;
}

void clear_fault_status(void)
{
	uint8_t curRegConfig = 0x00;
	uint8_t txData = 0x00;
	
	//Read current register configuration
	RTDTxBuffer[0] = READ_CONFIG;
	read_msg(&RTD_SPI_PORT, _RTD_CS, RTDTxBuffer, RTDRxBuffer);
	
	//Writing a 1 to bit D1 with D5,D3,D2 at 0 will clear fault bits
	txData = (curRegConfig | FAULT_CLEAR) & !ONE_SHOT & !FAULT_DET1 & !FAULT_DET0;
	send_msg(&RTD_SPI_PORT, _RTD_CS, &txData, 1);
}
