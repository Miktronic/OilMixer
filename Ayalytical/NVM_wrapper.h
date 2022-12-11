/*
 * NVM_wrapper.h
 *
 * Created: 12/13/2019 8:38:40 AM
 *  Author: adam
 */ 


#ifndef NVM_WRAPPER_H_
#define NVM_WRAPPER_H_



//////////////////////////////////////////////////////////////////////////
// Function prototypes
//////////////////////////////////////////////////////////////////////////
void init_NVM(void);
void write_PID_to_NVM(uint32_t memAdr, uint32_t *data, uint32_t dataLen);
void read_PID_from_NVM(uint32_t memAdr, uint32_t *data, uint32_t dataLen);
void write_to_NVM(uint32_t memAdr, uint8_t data[], uint32_t dataLen);
void read_from_NVM(uint32_t memAdr, uint8_t data[], uint32_t dataLen);

#endif /* NVM_WRAPPER_H_ */