/*
 * USB_Wrapper.h
 *
 * Created: 8/30/2019 2:20:39 PM
 *  Author: adam
 */ 


#ifndef USB_WRAPPER_H_
#define USB_WRAPPER_H_

#define FALSE	0
#define TRUE	1

#define NACK_MSG			0x15
#define RETURN_CHAR			0x0D //Carriage return character - Specified by Ayalytical for the end of message char

#define CMD_TYPE_GET		0
#define CMD_TYPE_SET		1
#define CMD_TYPE_UNKNOWN	2

#define NULL_CHAR			0
#define SEPERATOR_CHAR		' ' //Command is text seperated by spaces
#define END_OF_DATA_CHAR	13 //Carriage return character - specified by Ayalytical for end of message char
#define PLUS_CHAR			43 //ASCII for "+"
#define DASH_CHAR			45 //ASCII for "-"
#define DECIMAL_PNT_CHAR	46 //ASCII for "."

static const char GET_CMD_CHAR[] = "GET";
static const char SET_CMD_CHAR[] = "SET";
static const char OK_CMD_RESP[] = "OK";
static const char ERR_CMD_RESP[] = "Error";


//////////////////////////////////////////////////////////////////////////
// PC Commands
//////////////////////////////////////////////////////////////////////////
#define UNKNOWN_CMD			0
#define CMD_ROD				1
#define CMD_TEC				2
#define CMD_TEMP_SP			3
#define CMD_TEMP_SP_OFF		4
#define CMD_TEMP			5
#define CMD_TEMP_PID		6
#define CMD_MTR_CUR			7
#define CMD_MTR_AD			8
#define CMD_MTR_RATE		9
#define CMD_MTR_RATE_DUR	10
#define CMD_MTR_LIVE		11
#define CMD_MTR_STOP		12
#define CMD_MTR_MS			13
#define CMD_TEMP_IN			14
#define CMD_PUMP_ON			15
#define CMD_PUMP_OFF		16
#define CMD_PUMP_STATUS		17
#define CMD_FAN_DUTY		18
#define CMD_WDT				19
#define CMD_WDT_RESET		20
#define CMD_EEPROM			21
#define CMD_CONST_HEAT		22
#define CMD_NOISE_THRESH	23

static const char INVALID_PROBE_CHAR[] = "INVALID PROBE";

static const char UNKNOWN_CMD_CHAR[] = "UNKNOWN CMD";
static const char ROD_CMD_CHAR[] = "ROD";
static const char TEC_CMD_CHAR[] = "TEC";
static const char TEMP_SP_CMD_CHAR[] = "TEMP SP";
static const char TEMP_SP_OFF_CMD_CHAR[] = "TEMP SP OFF";
static const char TEMP_CMD_CHAR[] = "TEMP";
static const char TEMP_PID_CMD_CHAR[] = "TEMP PID";
static const char MTR_CUR_CMD_CHAR[] = "MTR-X C";
static const char MTR_AD_CMD_CHAR[] = "MTR-X AD";
static const char MTR_RATE_CMD_CHAR[] = "MTR-X S";
static const char MTR_RATE_DUR_CMD_CHAR[] = "MTR-X SD";
static const char MTR_LIVE_RATE_CMD_CHAR[] = "MTR-X";
static const char MTR_STOP_CMD_CHAR[] = "MTR-X Z";
static const char MTR_STEP_RES_CMD_CHAR[] = "MTR-X MS";
static const char TEMP_IN_CHAR[] = "TEMP IN";
static const char PUMP_ON_CHAR[] = "PUMP ON";
static const char PUMP_OFF_CHAR[] = "PUMP OFF";
static const char PUMP_STATUS_CHAR[] = "PUMP STATUS";
static const char FAN_DUTY_CHAR[] = "FAN DUTY";
static const char WDT_CHAR[] = "WDT";
static const char WDT_RESET_CHAR[] = "WDT RESET";
static const char EEPROM_CHAR[] = "EEPROM";
static const char CONST_HEAT_CHAR[] = "HEAT";

static const char *CMD_LIST[22] =
{
	ROD_CMD_CHAR,
	TEC_CMD_CHAR,
	TEMP_SP_CMD_CHAR,
	TEMP_SP_OFF_CMD_CHAR,
	TEMP_CMD_CHAR,
	TEMP_PID_CMD_CHAR,
	MTR_CUR_CMD_CHAR,
	MTR_AD_CMD_CHAR,
	MTR_RATE_CMD_CHAR,
	MTR_RATE_DUR_CMD_CHAR,
	MTR_LIVE_RATE_CMD_CHAR,
	MTR_STOP_CMD_CHAR,
	MTR_STEP_RES_CMD_CHAR,
	TEMP_IN_CHAR,
	PUMP_ON_CHAR,
	PUMP_OFF_CHAR,
	PUMP_STATUS_CHAR,
	FAN_DUTY_CHAR,
	WDT_CHAR,
	WDT_RESET_CHAR,
	EEPROM_CHAR,
	CONST_HEAT_CHAR
};

typedef struct USBMsg
{
	uint8_t validMsg;			//Valid message flag
	uint8_t motorDir;			//Motor direction (not always used)
	uint8_t motorSel;			//Motor selection (not always used)
	uint8_t cmd_type_char[3];	//Cmd type should be either GET or SET (3 chars)
	uint8_t cmd_type;			//0 = GET cmd, 1 = SET cmd, 2 = UNKNOWN
	uint8_t cmd_char[16];		//Allow for 16 char command
	uint8_t cmd;				//index value of command
	uint8_t cmd_data1[16];		//Allow for 16 char data
	uint8_t cmd_data2[16];		//Allow for 16 char data
	uint8_t cmd_data3[150];		//Allow for 150 char data, this is also used for EEProm values
	uint8_t cmd_data4[16];		//Allow for 16 char data
	float cmd_data_val1;		//Actual data value converted from char
	float cmd_data_val2;		//Actual data value converted from char
	float cmd_data_val3;		//Actual data value converted from char
	float cmd_data_val4;		//Actual data value converted from char
}USBMsg;

bool my_callback_cdc_enable(void);
void my_callback_cdc_disable(void);

uint16_t get_last_msg_length(void);
void send_cmd_response(uint8_t *TxBuff, uint8_t TxLen);
void init_USB(uint8_t *receiveFlag, uint8_t *receivedDataBufUSB[]);
uint8_t check_USB_ready(void);

#endif /* USB_WRAPPER_H_ */