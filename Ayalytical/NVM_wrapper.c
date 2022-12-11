/*
 * NVM_wrapper.c
 *
 * Created: 12/13/2019 8:37:51 AM
 *  Author: adam
 */ 

#include "nvm.h"
#include "common_nvm.h"
#include "NVM_wrapper.h"

void init_NVM(void)
{
	nvm_init(INT_EEPROM);
}

void write_PID_to_NVM(uint32_t memAdr, uint32_t *data, uint32_t dataLen)
{
	nvm_write(INT_EEPROM, memAdr, data, dataLen);
}

void read_PID_from_NVM(uint32_t memAdr, uint32_t *data, uint32_t dataLen)
{
	nvm_read(INT_EEPROM, memAdr, data, dataLen);
}

void write_to_NVM(uint32_t memAdr, uint8_t data[], uint32_t dataLen)
{
	nvm_write(INT_EEPROM, memAdr, data, dataLen);
}

void read_from_NVM(uint32_t memAdr, uint8_t data[], uint32_t dataLen)
{
	nvm_read(INT_EEPROM, memAdr, data, dataLen);
}