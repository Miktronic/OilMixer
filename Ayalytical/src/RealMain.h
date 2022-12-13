/*
 * RealMain.h
 *
 * Created: 8/12/2019 3:34:12 PM
 *  Author: adam
 */ 

#ifndef REALMAIN_H_
#define REALMAIN_H_

#define FALSE	0
#define TRUE	1

#define DISCRETE_CONTROL	0
#define SPI_CONTROL			1

#define USER_INFO_ADR	0
#define USER_INFO_LEN	128

#define PID_P_ADR		150
#define PID_I_ADR		160
#define PID_D_ADR		170

#define DMA_CHANNEL		0
#define DMA_BUFF_SIZE	64

#define FAN_SENSE_DMA_CHANNEL		0
#define FAN_SENSE_DMA_BUFF_SIZE		64
#define THERM_DMA_CHANNEL			1
#define THERM_DMA_BUFF_SIZE			64
#define PUMP_SENSE_DMA_CHANNEL		2
#define PUMP_SENSE_DMA_BUFF_SIZE	64
#define HEATER_SENSE_DMA_CHANNEL	3
#define HEATER_SENSE_DMA_BUFF_SIZE	64

#define SET_BIT(Val, Bit)		(Val |= (1 << Bit))
#define CLR_BIT(Val, Bit)		(Val &= ~(1 << Bit))

typedef struct rxMsg //TODO - REMOVE THIS WHEN USB COMMS ARE IMPLEMENTED
{
	uint8_t cmd;
	uint8_t data[16];
}rxMsg;

//////////////////////////////////////////////////////////////////////////
// uC pins
//////////////////////////////////////////////////////////////////////////
#define  LED_TOP			IOPORT_CREATE_PIN(PORTK, 7) // LED TOP pin
#define  LED_TOP_STATUS		IOPORT_CREATE_PIN(PORTK, 5) // LED TOP Status

#define  LED_BOTTOM			IOPORT_CREATE_PIN(PORTK, 4) // LED TOP pin
#define  LED_BOTTOM_STATUS	IOPORT_CREATE_PIN(PORTK, 3) // LED TOP Status

#define CAMERA_TRIGGER		IOPORT_CREATE_PIN(PORTK, 2) //camera trigger pin

#define ROD_POS_SWITCH		IOPORT_CREATE_PIN(PORTB, 2) //Rod raised position switch
#define PUMP_PWM			IOPORT_CREATE_PIN(PORTF, 6) //Pump pwm enable (if using timed control)
#define HEAT_PWM			IOPORT_CREATE_PIN(PORTC, 1) //Heat pwm enable

#define SYS_CLK_TC			TCC1
#define HEAT_PWM_TC			TCD1
#define HEAT_PWM_CCCHAN		TC_CCA
#define HEAT_PWM_TC_PERIOD	500	//Set this to half the freq (needs to be twice freq to bit bang)
#define FAN_PWM_TC			TCD1
#define FAN_PWM_CCCHAN		TC_CCB
#define FAN_PWM_TC_PERIOD	500 //Set this to half the freq (needs to be twice freq to bit bang)

#define _PUMP_HEAT_FR_STDBY	IOPORT_CREATE_PIN(PORTC, 3) //Pump & heat fault reset
#define _STATUS_RED_LED		IOPORT_CREATE_PIN(PORTD, 4) //Status LEDs
#define _STATUS_GREEN_LED	IOPORT_CREATE_PIN(PORTD, 5)
#define AVCC_REF_A			IOPORT_CREATE_PIN(PORTA, 0)

#define TEC_UPDATE_TIME		10
#define PID_UPDATE_TIME		10000
#define TEMP_UPDATE_TIME	1000
#define TEC_FREQ			150000

#define DUTY_CYCLE_MIN		0
#define DUTY_CYCLE_MAX		99
#define TEMP_SP_MIN			0
#define TEMP_SP_MAX			199
#define TEMP_PID_MIN		0
#define TEMP_PID_MAX		999.99
#define MTR_AD_MIN			0
#define MTR_AD_MAX			9999
#define MTR_S_MIN			-99999
#define MTR_S_MAX			99999
#define MTR_SD_MIN			-99999
#define MTR_SD_MAX			99999
#define PUMP_ON_MIN			0
#define PUMP_ON_MAX			600000 //(10 minutes)
#define TEC_FAN_DC_MIN		0
#define TEC_FAN_DC_MAX		100
#define WDT_SET_MIN			0
#define WDT_SET_MAX			300
#define EEPROM_MIN			0
#define EEPROM_MAX			128
#define CONST_HEAT_MIN		0
#define CONST_HEAT_MAX		100

#define ASCII_OFFSET		48

#define THERMISTOR	0
#define RTD_PROBE	1

#define P_GAIN	8.5
#define I_GAIN	0.25
#define I_MAX	25		//Limit min/max to 100 times the set gain
#define I_MIN	-25
#define D_GAIN	5

//////////////////////////////////////////////////////////////////////////
// Thermistor values
//////////////////////////////////////////////////////////////////////////
//TODO - Need to update these values with actually used thermistor values (Currently using NTCLE100E3 10k)
#define THERM_BETA		3977	//Thermistor Coefficient
#define THERM_To		296		//Room temp in K
#define THERM_Ro		10000	//Resistance at To (25°C)
#define THERM_COEFF_A	0.003354016
#define THERM_COEFF_B	0.000256985
#define THERM_COEFF_C	0.000002620131

#define NUM_MOTORS	2
enum MotorNameEnum
{
	ROD_POS = 0,
	MIXER,
	//LOWER_MIXER,
	//PUMP
};

enum LED_STATES
{
	OFF =			0,
	GREEN =			1,
	RED =			2,
	ORANGE =		3,
	GREEN_BLINK =	4,
	RED_BLINK =		5,
	ORANGE_BLINK =	6,
	STATUS_BLINK =	7
};

enum TEC_STATE
{
	TEC_IDLE =		0,
	TEC_OFF =		1,
	TEC_POS =		2,
	TEC_NEG =		3,
	TEC_TRANS =		4
};

#define STATUS_BLINK_DWELL_TIME	1000
#define WATCHDOG_RESET_TIME 750
#define INVALID_PROBE_VAL -79.5 //This is the value in the thermistor LUT that will be used when a RTD sensor is hooked up

//////////////////////////////////////////////////////////////////////////
// Function prototypes
//////////////////////////////////////////////////////////////////////////
void real_main(void);
void init_IO(void);
void init_motor_structs(void);
void init_steppers_SPI_control(void);
void init_RTD_sensor(void);
void init_sys_timer(void);
void init_heater_timer(void);
void init_fan_timer(void);
void interpret_rx_msg(rxMsg msg);

double calc_temp_from_ADC_reading(void);
double calc_temp_from_RTD_reading(uint16_t reading);
double calc_sense_cur_from_ADC_reading(uint16_t reading);

void send_cmd_ack(uint8_t cmd, uint8_t *data);
void send_cmd_nack(uint8_t *data);

void clear_receivedUSBMsg(void);
void flush_USB_rx_buffer(void);
void flush_USB_tx_buffer(void);

void handle_LEDs(void);
void handle_temp_PID(void);
void handle_pump(void);
void handle_fan(void);
void calc_heater_adj(void);
void handle_TEC(void);
void reset_blink_time_LED(void);

void clear_pump_heat_fault(void);

uint8_t check_cmd_type(char *cmd);
uint8_t convert_cmd_str_to_char(char *cmd);
float convert_msg_data_to_num(char *msgData);
void convert_int_to_string(uint32_t val, char *convString);
void convert_float_to_string(float val, char *convString, uint8_t convEnd);
uint8_t convert_received_motor_to_index(uint8_t recChar);
uint8_t check_motor_index(uint8_t motorIndex);
void parse_received_USB_msg(void);
void handle_received_USB_cmd(void);

uint8_t check_received_data_limits(float checkVal, float minVal, float maxVal);

void init_DMA(void);
void fan_sense_DMA_done(uint8_t DMAStatus);
void therm_DMA_done(uint8_t DMAStatus);
void pump_sense_DMA_done(uint8_t DMAStatus);
void heater_sense_DMA_done(uint8_t DMAStatus);
double calc_buff_average(uint16_t *bufVal, uint16_t bufLen);

#endif /* REALMAIN_H_ */