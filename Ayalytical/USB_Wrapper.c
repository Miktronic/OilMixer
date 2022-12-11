/*
 * USB_Wrapper.c
 *
 * Created: 8/30/2019 2:20:24 PM
 *  Author: adam
 */ 

#include "udc.h"
#include "udi_cdc.h"
#include "USB_Wrapper.h"

static uint8_t dataTransferAuth = FALSE;
uint16_t dataCount = 0;
uint8_t *rxDataBuf;
uint8_t *dataReceived;

bool my_callback_cdc_enable(void)
{
	dataTransferAuth = TRUE;
	return true;
}

void my_callback_cdc_disable(void)
{
	dataTransferAuth = FALSE;
}

void my_callback_rx_notify(void)
{
	if (dataTransferAuth)
	{
		*dataReceived = udi_cdc_is_rx_ready();
		if (*dataReceived)
		{
			dataCount = udi_cdc_get_nb_received_data();
			for (uint8_t i = 0; i < dataCount; i++)
			{
				*rxDataBuf = (uint8_t)udi_cdc_getc();
				rxDataBuf++;
			}
		}
		
		rxDataBuf -= dataCount; //Reset pointer to start of buffer
	}
}

uint16_t get_last_msg_length(void)
{
	return dataCount;
}

void send_cmd_response(uint8_t *TxBuff, uint8_t TxLen)
{	
	for (uint8_t i = 0; i < TxLen; i++)
	{
		udi_cdc_putc(TxBuff[i]);
	}
}

void init_USB(uint8_t *receiveFlag, uint8_t *receivedDataBufUSB[])
{
	udc_start();
	rxDataBuf = receivedDataBufUSB;
	dataReceived = receiveFlag;
}

uint8_t check_USB_ready(void)
{
	return dataTransferAuth;
}