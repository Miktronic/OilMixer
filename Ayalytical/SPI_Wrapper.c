/*
 * SPI_Wrapper.c
 *
 * Created: 8/16/2019 4:18:19 PM
 *  Author: adam
 */ 

#include <avr/io.h>
#include "spi_master.h"
#include "SPI_Wrapper.h"
#include "RealMain.h"

struct spi_device RTDSPIConfig;
struct spi_device stepperSPIConfig;

//Configures all IO required for SPI comms
void init_SPI_IO(void)
{
	//Configures required pins for RTD SPI comms
	ioport_configure_pin(_RTD_CS, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);	//Configure slave chip select as high output	
	ioport_configure_pin(RTD_SDI, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);	//Configure slave data in as high output
	ioport_configure_pin(RTD_SDO, IOPORT_DIR_INPUT);						//Configure slave data out as input
	ioport_configure_pin(RTD_SCK, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);	//Configure slave clock as high output
	
	//Configures required pins for stepper SPI comms	
	ioport_configure_pin(_TEST_CS, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(_ROD_POS_CS, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);		//Configure slave chip selects as high outputs
	ioport_configure_pin(_MIXER_CS, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(_LOWER_MIXER_CS, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(_PUMP_CS, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(STEPPER_SDI, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);		//Configure slave data in as high output
	ioport_configure_pin(STEPPER_SDO, IOPORT_DIR_INPUT);							//Configure slave data out as input
	ioport_configure_pin(STEPPER_SCK, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);		//Configure slave clock as high output
}

void init_SPI(void)
{	
	//Init IO for SPI comms
	init_SPI_IO();
	
	//Set chip selects for SPI configs (stepper chip select will change)
	RTDSPIConfig.id = _RTD_CS;
	stepperSPIConfig.id = _TEST_CS; //NOTE: FOR SOME REASON INITIALIZING CS PIN TO ONE OF THE ACTUAL CS PINS CAUSES SPIE TO ENTER SLAVE MODE?
	//stepperSPIConfig.id = _ROD_POS_CS;
	
	//Initialize SPI in master mode
	spi_master_init(&SPIC);
	spi_master_init(&SPIE);
	
	spi_master_setup_device(&SPIC, &RTDSPIConfig, SPI_MODE_1, RTD_SPI_BAUD, 0); //From datasheet, RTD chip supports SPI modes 1 and 3
	spi_master_setup_device(&SPIE, &stepperSPIConfig, SPI_MODE_0, STEPPER_SPI_BAUD, 0);
	
	//Enable SPI
	spi_enable(&SPIC);
	spi_enable(&SPIE);
}

void disable_SPI(SPI_t *SPIPort)
{
	spi_disable(SPIPort);
}

void select_SPI_device(SPI_t *SPIPort, ioport_pin_t chipSelectPin)
{
	if (SPIPort == &RTD_SPI_PORT)
	{
		spi_select_device(&RTD_SPI_PORT, &RTDSPIConfig);
	}
	else if (SPIPort == &STEPPER_SPI_PORT)
	{
		stepperSPIConfig.id = chipSelectPin;
		spi_select_device(&STEPPER_SPI_PORT, &stepperSPIConfig);
	}
}

void deselect_SPI_device(SPI_t *SPIPort)
{
	if (SPIPort == &RTD_SPI_PORT)
	{
		spi_deselect_device(&RTD_SPI_PORT, &RTDSPIConfig);
	}
	else if (SPIPort == &STEPPER_SPI_PORT)
	{
		spi_deselect_device(&STEPPER_SPI_PORT, &stepperSPIConfig);
	}
}

void send_msg(SPI_t *SPIPort, ioport_pin_t cs, uint8_t *msgBytes, uint8_t len)
{	
	select_SPI_device(SPIPort, cs);
	spi_write_packet(SPIPort, msgBytes, len);
	deselect_SPI_device(SPIPort);
}

void read_msg(SPI_t *SPIPort, ioport_pin_t cs, uint8_t *regAddr, uint8_t *msgBuf)
{
	uint8_t dummyData[2];
	dummyData[0] = *regAddr;
	dummyData[1] = 0xFF;
	
	select_SPI_device(SPIPort, cs); //ASF doc says this needs to be done beforehand
	spi_write_packet(SPIPort, dummyData, 2);
	spi_read_single(SPIPort, msgBuf);
	deselect_SPI_device(SPIPort);
}

void send_and_read_msg(SPI_t *SPIPort, ioport_pin_t _cs, uint8_t *readBuf, uint8_t *sendBuf, uint8_t len)
{
	select_SPI_device(SPIPort, _cs);	
	while (len--)
	{
		spi_write_single(SPIPort, *sendBuf++);
		
		while (!spi_is_rx_full(SPIPort))
		{
			
		}
		*readBuf = spi_get(SPIPort);
		readBuf++;
		
		
		//This comes in as MSB first but buffer stores it as
	}
	deselect_SPI_device(SPIPort);
}



//while (len--) {
	//spi_write_single(spi,CONFIG_SPI_MASTER_DUMMY); //Dummy write
//
	//while (!spi_is_rx_full(spi)) {
	//}
	//
	//spi_read_single(spi, data);
	//data++;
//}
//
//return STATUS_OK;

//while (len--) {
	//spi_write_single(spi, *data++);
	//
	//while (!spi_is_rx_full(spi)) {
	//}
//}
//
//return STATUS_OK;