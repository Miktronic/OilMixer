/**
 * \file
 *
 * \brief User board configuration template
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef CONF_BOARD_H
#define CONF_BOARD_H

#define BOARD_XOSC_HZ			16000000 //Crystal connected to XTAL is 16.000MHz
#define BOARD_XOSC_STARTUP_US	XOSC_STARTUP_16384 //TODO - CHECK ACTUAL VALUE
#define BOARD_XOSC_TYPE			XOSC_TYPE_XTAL

#endif // CONF_BOARD_H
