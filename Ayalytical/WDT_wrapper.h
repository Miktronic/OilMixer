/*
 * WDT_wrapper.h
 *
 * Created: 12/17/2019 12:41:27 PM
 *  Author: adam
 */ 


#ifndef WDT_WRAPPER_H_
#define WDT_WRAPPER_H_

#define WDT_OFF 11

//////////////////////////////////////////////////////////////////////////
// Function Prototypes
//////////////////////////////////////////////////////////////////////////
void enable_WDT(void);
void disable_WDT(void);
void set_WDT_timeout(uint8_t timeoutIndex);
void reset_WDT(void);

#endif /* WDT_WRAPPER_H_ */