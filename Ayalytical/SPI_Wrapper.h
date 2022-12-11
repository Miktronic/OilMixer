/*
 * SPI_Wrapper.h
 *
 * Created: 8/16/2019 4:18:36 PM
 *  Author: adam
 */ 

#include "ioport.h"

#ifndef SPI_WRAPPER_H_
#define SPI_WRAPPER_H_

#define RTD_SPI_BAUD 1000000
#define STEPPER_SPI_BAUD 1000000

#define RTD_SPI_PORT		SPIC
#define STEPPER_SPI_PORT	SPIE

//////////////////////////////////////////////////////////////////////////
// Pin definitions
//////////////////////////////////////////////////////////////////////////
#define _RTD_RDY			IOPORT_CREATE_PIN(PORTC, 3) //TDI temperature sensor SPI lines
#define _RTD_CS				IOPORT_CREATE_PIN(PORTC, 4)
#define RTD_SDI				IOPORT_CREATE_PIN(PORTC, 5)
#define RTD_SDO				IOPORT_CREATE_PIN(PORTC, 6)
#define RTD_SCK				IOPORT_CREATE_PIN(PORTC, 7)

#define _ROD_POS_CS			IOPORT_CREATE_PIN(PORTH, 4)
#define _MIXER_CS			IOPORT_CREATE_PIN(PORTJ, 4)
#define _LOWER_MIXER_CS		IOPORT_CREATE_PIN(PORTK, 4)
#define _PUMP_CS			IOPORT_CREATE_PIN(PORTF, 4)
#define _TEST_CS			IOPORT_CREATE_PIN(PORTE, 4)
#define STEPPER_SCK			IOPORT_CREATE_PIN(PORTE, 7) //Stepper motor SPI lines
#define STEPPER_SDO			IOPORT_CREATE_PIN(PORTE, 6)
#define STEPPER_SDI			IOPORT_CREATE_PIN(PORTE, 5)

//////////////////////////////////////////////////////////////////////////
// Function Prototypes
//////////////////////////////////////////////////////////////////////////
void init_SPI_IO(void);
void init_SPI(void);
void disable_SPI(SPI_t *SPIPort);
void select_SPI_device(SPI_t *SPIPort, ioport_pin_t chipSelectPin);
void send_msg(SPI_t *SPIPort, ioport_pin_t cs, uint8_t *msgBytes, uint8_t len);
void deselect_SPI_device(SPI_t *SPIPort);
void send_msg(SPI_t *SPIPort, ioport_pin_t cs, uint8_t *msgBytes, uint8_t len);
void read_msg(SPI_t *SPIPort, ioport_pin_t cs, uint8_t *regAddr, uint8_t *msgBuf);
void send_and_read_msg(SPI_t *SPIPort, ioport_pin_t _cs, uint8_t *readBuf, uint8_t *sendBuf, uint8_t len);

#endif /* SPI_WRAPPER_H_ */