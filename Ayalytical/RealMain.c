/*
 * RealMain.c
 *
 * Created: 8/12/2019 3:33:24 PM
 *  Author: adam
 */ 

//////////////////////////////////////////////////////////////////////////
// Version History - PN 10000495
//////////////////////////////////////////////////////////////////////////
/*
Version 1.0: (12/5/2019) -ACW
- Original

Version 1.1: (12/17/2019) -ACW
- Changed "GET ROD" command to return rod state in ASCII rather than 0/1
- Added missing break statement in "SET FAN DUTY" case
- Changed acceleration command to use current motor speed rather than reset speed from 0
- Added commands for getting/setting WDT timeout and resetting WDT count

Version 1.2: (12/30/2019) -ACW
- Increased "PUMP ON" command upper time limit from 10 seconds to 10 minutes per request
- Changed "PUMP ON" command to allow for indefinite pump time if given no time parameter or parameter of 0
- Added "PUMP OFF" command to turn off pump and clear pump time
- Added "PUMP STATUS" command to get current pump time
- Added "TEMP SP OFF" command to set temperature setpoint to 0 and disable heater output
- Updated "GET TEMP" command so that it will read temperature without needing to have a temperature setpoint first
- Fixed bug in "GET MTR AD" command which prevented returning of motor acceleration/deceleration information
- Fixed bug in convert_float_to_string function which was typecasting to a char instead of an int
- Updated handle_decel function so motor speed is updated to reflect current speed
- Updated set_motor_speed to allow for setting motor speed to 0 without causing issues

Version 1.3: (1/14/2020) -ACW
- Removed "MTR-X AD" command
- Removed "MTR-X Z" command
- Changed "MTR-X S" and "MTR-X SD" commands to require an acceleration parameter (last parameter) in command
- Changed message parsing to assume direction is positive if no + or - received. Previously kept last sent direction
- Changed "GET MTR-X" command to return the motors speed, acceleration time remaining, and steps remaining.
	Returns -1 for steps if being run in continuous mode
- Fixed motor command range so that "GET MTR-X" and "GET MTR-X MS" return the correct motor value for "X"
- Changed WDT command per request to be in 1s intervals allowing for times up to 300 seconds (5 minutes)
- Changed "GET EEPROM" command to return all 128 bytes of stored data
- Changed "SET EEPROM" command to require length of data and actual data in message. Setting the EEProm will start at
	the 0 position and overwrite sequentially
- Changed "UNKNOWN CMD" response to respond with only data received rather than the whole USBRxBuff array
- Changed command parsing so that invalid commands without spaces don't cause the board to get stuck in a loop
- Changed command parsing so that commands that don't contain a return character (0x0D) are ignored
- Changed "GET TEMP" command to return "INVALID PROBE" if the return temperature would be -79.5. This is the
	default value of the thermistor LUT when one isn't connected but a RTD probe is
- Changed "MTR-X C" command to only enable the motor IC if the command parameter is valid

Version 1.4: (1/21/2020) -ACW
- Fixed "MTR-X C" command to get/set the current values appropriately
- Deceleration with specified steps would return the steps remaining minus the offset at which the motor would
	begin to decelerate. Changed to return the remaining steps throughout the deceleration process
- Fixed "GET TEMP IN" command to return the ASCII equivalent rather than 0 or 1

Version 1.5: (1/31/2020) -ACW
- Updated motor speed routines to update stored speed value when set to 0
- Changed "SET TEC" command to use a state machine to gradually ramp up/down to the TEC setpoint rather than
	set the setpoint directly. TEC hardware wouldn't allow for direct setting without behaving incorrectly

Version 1.6: (2/27/2020) -ACW
- Updated PID parameters to maintain the temperature within +/- 1°C as per spec
- Added DC offset to heater to keep the heater on at a steady rate once reaching steady state. This was needed
	because the heater would take too long to re-heat after it was shut off resulting in the temperature dropping
	below +/- 1°C at higher temperatures

Version 1.7: (7/17/2020) -ACW
- Reverted PID to just use the PID parameters (not using offset table)
- PID parameters will now be stored in EEProm when they're changed and loaded on startup
- Removed INVALID PROBE response if the temperature input selection is "wrong"
- Slowed down PID calculation rate to once every 10s. Temperature reading for the loop occurs once every second now
- Added a division by 25 to total PID output to allow for larger D terms without adjusting the command limits
- Minimum I term will now be capped to 0. Maximum I term will now be capped to whatever value allows the resulting
	gain * term to give 100% heater output. The max value will vary depending on the gain value.
- Corrected PID routine to calculate the total heater output correctly

Version 1.8: (7/21/2020) -ACW
- Changed ADC initialization to match Foam DDI board for updated thermistor table
- Updated thermistor LUT for hardware changes

Version 1.9: (8/11/2020) -ACW
- Changed convert_float_to_string command to fix issue where a zero following the decimal place would be dropped
*/

#include "dma.h"
#include "ioport.h"
#include "math.h"
#include "pwm.h"
#include "stdlib.h"
#include "tc.h"

#include "ADC_Wrapper.h"
#include "NVM_wrapper.h"
#include "RealMain.h"
#include "SPI_Wrapper.h"
#include "StepperMotor.h"
#include "TEC_Controller.h"
#include "TempPIDControl.h"
#include "RTD_Wrapper.h"
#include "USB_Wrapper.h"
#include "WDT_wrapper.h"

#include "LUT.h"

#define WATCHDOG_ENABLED

//////////////////////////////////////////////////////////////////////////
// System Time Counter Variables
//////////////////////////////////////////////////////////////////////////
uint32_t sysTime_ms = 0;
uint32_t blinkTimeLED_ms = 0;
uint32_t pumpTime_ms = 0;
uint32_t tempPIDUpdate_ms = 0;
uint32_t tempUpdate_ms = 0;
uint32_t pumpDuration_ms = 0;
uint32_t TECTime_ms = 0;
volatile uint32_t watchdogTime_ms = 0;

//////////////////////////////////////////////////////////////////////////
// Status LED Variables
//////////////////////////////////////////////////////////////////////////
volatile uint8_t LEDState = OFF;
uint16_t LEDOnTime = 0;
uint16_t LEDOffTime = 0;
uint16_t LEDBlinkCount = 0;
uint16_t curLEDBlinkCount = 0;

//////////////////////////////////////////////////////////////////////////
// Enable Flags
//////////////////////////////////////////////////////////////////////////
uint8_t fanEnabled = FALSE;
uint8_t pumpEnabled = FALSE;
uint8_t heaterEnabled = FALSE;
uint8_t RTDMsgAvailable = FALSE;

//////////////////////////////////////////////////////////////////////////
// Stepper SPI Tx/Rx Buffers
//////////////////////////////////////////////////////////////////////////
uint8_t stepperDataBuffer[16];
uint8_t stepperTxBuffer[8];

//////////////////////////////////////////////////////////////////////////
// Temperature Control Variables
//////////////////////////////////////////////////////////////////////////
double tempSetPoint = 0.0; //Desired temp setpoint
double curTemp = 0.0;
uint8_t tempProbeType = THERMISTOR; //Default to reading from thermistor
PIDStruct heaterPID;
pwm_config heaterPWMConfig;
uint8_t heatDCOffsetEn = TRUE;
uint8_t heatingUp = FALSE;
uint8_t reachedTemp1st = FALSE;
uint8_t heatCoolDelay = FALSE;
uint8_t cooledOff = FALSE;
float heaterDCOffset = 0.0;
float tempSetpointDif = 0.0;

//////////////////////////////////////////////////////////////////////////
// USB Tx/Rx Buffers/Flags
//////////////////////////////////////////////////////////////////////////
uint8_t USBTxBuff[200]; //Needs to be at least 128 + msg overhead for EEProm commands
uint8_t USBRxBuff[200];
uint8_t USBDataReceived = FALSE;
volatile uint8_t USBReady = FALSE;
USBMsg receivedUSBMsg;

uint8_t TECState = TEC_OFF;
uint8_t TECPWMDutyA = 0;
uint8_t TECPWMDutyB = 0;
uint8_t curTECPWMDutyA = 0;
uint8_t curTECPWMDutyB = 0;
uint8_t fanDuty = 0;
motorConfig systemMotors[NUM_MOTORS];

uint32_t WDTResetCount = 0;
uint32_t WDTResetCountSet = 0;
float adjSteps[4] = {};

//////////////////////////////////////////////////////////////////////////
// ADC Channel DMA Buffers
//////////////////////////////////////////////////////////////////////////
static uint16_t fanSenseDMABuffer[FAN_SENSE_DMA_BUFF_SIZE / 2];
static uint16_t thermDMABuffer[THERM_DMA_BUFF_SIZE / 2];
static uint16_t pumpSenseDMABuffer[PUMP_SENSE_DMA_BUFF_SIZE / 2];
//static uint16_t heaterSenseDMABuffer[HEATER_SENSE_DMA_BUFF_SIZE / 2]; //Unused

//////////////////////////////////////////////////////////////////////////
// Rod Position Home Switch - Called when PB2 transitions high to low
//////////////////////////////////////////////////////////////////////////
ISR(PORTB_INT0_vect)
{
	uint8_t rodSwitchIn = 0;
	rodSwitchIn = ioport_get_pin_level(ROD_POS_SWITCH);
	
	//Stop motor if rod switch is hit and motor is moving up
	if ((!rodSwitchIn) && (systemMotors[ROD_POS].direction == DIR_CCW))
	{
		stop_motor(&systemMotors[ROD_POS]);
	}
	
	LEDState = RED;
}

//////////////////////////////////////////////////////////////////////////
// RTD Data Ready - Called when PB3 transitions high to low (Currently unused)
//////////////////////////////////////////////////////////////////////////
ISR(PORTC_INT0_vect)
{
	RTDMsgAvailable = TRUE;
}

//////////////////////////////////////////////////////////////////////////
// Main System Timebase - Triggers every 1ms
//////////////////////////////////////////////////////////////////////////
static void sys_clk_overflow_callback(void)
{
	//Increment time counters
	sysTime_ms++;
	blinkTimeLED_ms++;
	pumpTime_ms++;
	tempPIDUpdate_ms++;
	tempUpdate_ms++;
	watchdogTime_ms++;
	TECTime_ms++;
	
	for (uint8_t i = 0; i < NUM_MOTORS; i++)
	{
		if (systemMotors[i].enabled)
		{
			systemMotors[i].stepTime_ms++;
		}
	}
	
	tc_clear_overflow(&SYS_CLK_TC); //Reset overflow interrupt flag
}

//////////////////////////////////////////////////////////////////////////
// Heater CC Callback - Triggers on compare of CCA for bit banging HEAT_PWM output
//////////////////////////////////////////////////////////////////////////
static void heater_cc_overflow_callback(void)
{
	if (heaterEnabled)
	{
		ioport_set_pin_high(HEAT_PWM); //Enable heater output
	}
}

//////////////////////////////////////////////////////////////////////////
// Fan CC Callback - Triggers on compare of CCB for bit banging FAN_PWM output
//////////////////////////////////////////////////////////////////////////
static void fan_cc_overflow_callback(void)
{
	if (fanEnabled)
	{
		ioport_set_pin_high(FAN_PWM); //Enable fan output
	}
}

static void heater_timer_overflow_callback(void)
{
	if (heaterEnabled)
	{
		ioport_set_pin_low(HEAT_PWM); //Disable heater output
	}
	
	if (fanEnabled)
	{
		ioport_set_pin_low(FAN_PWM); //Disable fan output
	}
}

//////////////////////////////////////////////////////////////////////////
// Rod Pos Timer Callback - Handles specified steps of stepper
//////////////////////////////////////////////////////////////////////////
static void rod_position_timer_overflow_callback(void)
{
	if (systemMotors[ROD_POS].specifiedSteps) //Motor is configured for specific steps to move
	{
		if (systemMotors[ROD_POS].stepsToMove > adjSteps[ROD_POS]) //Motor still has steps to move
		{
			systemMotors[ROD_POS].stepsToMove--;
		}
		else //Start deceleration if configured, or stop motor directly
		{
			if (systemMotors[ROD_POS].enabled)
			{
				stop_motor(&systemMotors[ROD_POS]);
			}
			
			if (systemMotors[ROD_POS].stepsToMove > 0)
			{
				systemMotors[ROD_POS].stepsToMove--;
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////
// Mixer Timer Callback - Handles specified steps of stepper
//////////////////////////////////////////////////////////////////////////
static void mixer_timer_overflow_callback(void)
{
	if (systemMotors[MIXER].specifiedSteps) //Motor is configured for specific steps to move
	{
		if (systemMotors[MIXER].stepsToMove > adjSteps[MIXER]) //Motor still has steps to move
		{
			systemMotors[MIXER].stepsToMove--;
		}
		else //Start deceleration if configured, or stop motor directly
		{
			if (systemMotors[MIXER].enabled)
			{
				stop_motor(&systemMotors[MIXER]);
			}
			
			if (systemMotors[MIXER].stepsToMove > 0)
			{
				systemMotors[MIXER].stepsToMove--;
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////
// Lower Mixer Timer Callback - Handles specified steps of stepper
//////////////////////////////////////////////////////////////////////////
static void lower_mixer_timer_overflow_callback(void)
{
	if (systemMotors[LOWER_MIXER].specifiedSteps) //Motor is configured for specific steps to move
	{
		if (systemMotors[LOWER_MIXER].stepsToMove > adjSteps[LOWER_MIXER]) //Motor still has steps to move
		{
			systemMotors[LOWER_MIXER].stepsToMove--;
		}
		else //Start deceleration if configured, or stop motor directly
		{
			if (systemMotors[LOWER_MIXER].enabled)
			{
				stop_motor(&systemMotors[LOWER_MIXER]);
			}
			
			if (systemMotors[LOWER_MIXER].stepsToMove > 0)
			{
				systemMotors[LOWER_MIXER].stepsToMove--;
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////
// Pump Timer Callback - Handles specified steps of stepper
//////////////////////////////////////////////////////////////////////////
static void pump_timer_overflow_callback(void)
{
	if (systemMotors[PUMP].specifiedSteps) //Motor is configured for specific steps to move
	{
		if (systemMotors[PUMP].stepsToMove > adjSteps[PUMP]) //Motor still has steps to move
		{
			systemMotors[PUMP].stepsToMove--;
		}
		else //Start deceleration if configured, or stop motor directly
		{
			if (systemMotors[PUMP].enabled)
			{
				stop_motor(&systemMotors[PUMP]); //This doesn't take into account deceleration (will move a bit further if it is enabled)
			}
			
			if (systemMotors[PUMP].stepsToMove > 0)
			{
				systemMotors[PUMP].stepsToMove--;
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////
//This is the real main function called from generated main.c
//////////////////////////////////////////////////////////////////////////
void real_main(void)
{	
	//Set LEDs to be a red 1s ON/OFF blink during initialization
	LEDState = RED_BLINK;
	LEDOnTime = 1000;
	LEDOffTime = 1000;
	
	//Peripheral Initialization
	disable_WDT();			//Disable watchdog timer during initialization
	ioport_init();			//Initialize I/O port service
	sysclk_init();			//Initialize system clock
	pmic_init();			//Initialize programmable multilevel interrupt controller
	irq_initialize_vectors(); //Initialize interrupt vectors
	cpu_irq_enable();		//Enable interrupts
	sysclk_enable_module(SYSCLK_PORT_GEN, SYSCLK_EVSYS); //TODO - THIS IS NO LONGER USED -> REMOVE
	init_sys_timer();		//Initialize TCC1 for system timer (1ms)
	init_heater_timer();	//Initialize TCD1 for heat pwm output (CHA)
	init_fan_timer();		//Initialize TCD1 for fan pwm output (CHB)
	init_SPI();				//Initialize SPIC for RTD sensor and SPIE for stepper motors
	init_IO();				//Initialize general I/O
	init_USB(&USBDataReceived, &USBRxBuff); //Initialize USB peripheral
	while (!USBReady) //Wait unti USB is initialized
	{
		USBReady = check_USB_ready();
		handle_LEDs();
	}
	init_RTD_sensor();		//Initialize RTD temperature sensor
	init_TEC_controller();	//Initialize TEC controller
	init_ADC();				//Initialize ADC peripheral in free-running mode
	init_DMA();				//Initialize DMA peripheral
	init_NVM();				//Initialize NVM peripheral
	
	// Read PID parameters out of EEProm
	uint32_t tmpEEPromVal = 0;
	read_PID_from_NVM(PID_P_ADR, &tmpEEPromVal, 4);
	heaterPID.pGain = ((float)tmpEEPromVal / 100);
	read_PID_from_NVM(PID_I_ADR, &tmpEEPromVal, 4);
	heaterPID.iGain = ((float)tmpEEPromVal / 100);
	read_PID_from_NVM(PID_D_ADR, &tmpEEPromVal, 4);
	heaterPID.dGain = ((float)tmpEEPromVal / 100);
	heaterPID.iMax = (100 / heaterPID.iGain) * OUTPUT_SCALE;
	heaterPID.iMin = 0;
	
	LEDState = GREEN_BLINK; //Change LED to green after initialization is complete
	
	while (1)
	{
		handle_LEDs();		//Handle status LED logic
		handle_temp_PID();	//Handle temperature control PID
		handle_pump();		//Handle discrete pump control
		handle_fan();		//Handle TEC fan control
		handle_TEC();		//Handle TEC output
		
		//Reset the watchdog if enabled
		if (WDTResetCountSet != 0)
		{
			if (watchdogTime_ms >= WATCHDOG_RESET_TIME)
			{
				watchdogTime_ms = 0;
				
				//Reset the WDT every 750ms
				if (WDTResetCount != 0)
				{
					reset_WDT();
					WDTResetCount--;
				}
			}
		}
		
		//Check and respond to received USB message
		if (USBDataReceived)
		{
			parse_received_USB_msg();
			handle_received_USB_cmd();
			USBDataReceived = FALSE;
		}
		
		//Handle stepper acceleration/deceleration
		for (uint8_t i = 0; i < NUM_MOTORS; i++)
		{		
			if (systemMotors[i].enabled)
			{
				if ((systemMotors[i].accelRate != 0) && (systemMotors[i].accelTime != 0))
				{
					if (systemMotors[i].stepTime_ms >= ACCEL_TIME)
					{
						handle_accel(&systemMotors[i]); //Increase motor speed if configured with acceleration
						systemMotors[i].stepTime_ms = 0;
					}
				}
				
				if (systemMotors[i].decelRate != 0)
				{
					systemMotors[i].accelRate = 0;
					systemMotors[i].accelRate = 0;
					//Only decelerate if the motor isn't accelerating
					if (systemMotors[i].stepTime_ms >= DECEL_TIME)
					{
						handle_decel(&systemMotors[i]); //Decrease motor speed if configured with acceleration
						systemMotors[i].stepTime_ms = 0;
					}
				}
			}
		}
		
		if (!ioport_get_pin_level(_RTD_RDY))
		{
			//Not used, just sample RTD on interval
		}
	}
}

//////////////////////////////////////////////////////////////////////////
//Initializes I/O that isn't initialized as part of a peripheral
//////////////////////////////////////////////////////////////////////////
void init_IO(void)
{	
	init_motor_structs();
	init_stepper_motor_pins();
	init_steppers_SPI_control();
	
	//Setup pin interrupt on PB2 for rod switch signal
	ioport_set_pin_dir(ROD_POS_SWITCH, IOPORT_DIR_INPUT);
	PORTB.PIN2CTRL = PORT_ISC_FALLING_gc;
	PORTB.INT0MASK = PIN2_bm;
	PORTB.INTCTRL = PORT_INT0LVL_LO_gc;
	
	//Setup pin interrupt on PC3 for RTD_DRDY# signal
	ioport_set_pin_dir(_RTD_RDY, IOPORT_DIR_INPUT);
	PORTC.PIN3CTRL = PORT_ISC_FALLING_gc;
	PORTC.INT0MASK = PIN3_bm;
	PORTC.INTCTRL = PORT_INT0LVL_LO_gc;
	
	ioport_configure_pin(PUMP_PWM, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW); //Non-stepper control for pump (may not be used)
	ioport_configure_pin(HEAT_PWM, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW); //Heat pwm control
	ioport_configure_pin(_PUMP_HEAT_FR_STDBY, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH); //Fault reset (active low)
	
	//Status LEDs (active low)
	ioport_configure_pin(_STATUS_RED_LED, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(_STATUS_GREEN_LED, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	
	//Temporary fix for ADC reference use for 3.3V
	//NOTE - BOARD MOD REQUIRED FOR THIS TO WORK
	ioport_configure_pin(AVCC_REF_A, IOPORT_PULL_UP);
	
	//Initialize heater PID parameters
	heaterPID.dGain = D_GAIN;
	heaterPID.dState = 0;
	heaterPID.iGain = I_GAIN;
	heaterPID.iMax = I_MAX;
	heaterPID.iMin = I_MIN;
	heaterPID.iState = 0;
	heaterPID.pGain = P_GAIN;
}

//////////////////////////////////////////////////////////////////////////
//Initializes uC pins for each motor in the motor structs
//////////////////////////////////////////////////////////////////////////
void init_motor_structs(void)
{
	//Initialize system stepper motors
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		reset_motor_config(&systemMotors[i]);
	}
	
	//Initialize uC pins for rod pos stepper driver and pwm settings
	systemMotors[ROD_POS].motorID = ROD_POS;
	systemMotors[ROD_POS].controlPins.dir = ROD_POS_DIR;
	systemMotors[ROD_POS].controlPins.step = ROD_POS_STEP;
	systemMotors[ROD_POS].controlPins._rstPort = _ROD_POS_RST;
	systemMotors[ROD_POS].controlPins.ms1 = ROD_POS_MS1;
	systemMotors[ROD_POS].controlPins.ms0 = ROD_POS_MS0;
	systemMotors[ROD_POS].controlPins.en = ROD_POS_EN;
	systemMotors[ROD_POS].controlPins._cs = _ROD_POS_CS;
	systemMotors[ROD_POS].controlPins.diag = ROD_POS_DIAG;
	systemMotors[ROD_POS].PWMTC = PWM_TCE1;
	systemMotors[ROD_POS].PWMChan = PWM_CH_A;
	systemMotors[ROD_POS].PWMDuty = 50;
	
	//Initialize uC pins for mixer stepper driver and pwm settings
	systemMotors[MIXER].motorID = MIXER;
	systemMotors[MIXER].controlPins.dir = MIXER_DIR;
	systemMotors[MIXER].controlPins.step = MIXER_STEP;
	systemMotors[MIXER].controlPins._rstPort = _MIXER_RST;
	systemMotors[MIXER].controlPins.ms1 = MIXER_MS1;
	systemMotors[MIXER].controlPins.ms0 = MIXER_MS0;
	systemMotors[MIXER].controlPins.en = MIXER_EN;
	systemMotors[MIXER].controlPins._cs = _MIXER_CS;
	systemMotors[MIXER].controlPins.diag = MIXER_DIAG;
	systemMotors[MIXER].PWMTC = PWM_TCF0;
	systemMotors[MIXER].PWMChan = PWM_CH_D;
	systemMotors[MIXER].PWMDuty = 50;
	
	//Initialize uC pins for lower mixer stepper driver and pwm settings
	systemMotors[LOWER_MIXER].motorID = LOWER_MIXER;
	systemMotors[LOWER_MIXER].controlPins.dir = LOWER_MIXER_DIR;
	systemMotors[LOWER_MIXER].controlPins.step = LOWER_MIXER_STEP;
	systemMotors[LOWER_MIXER].controlPins._rstPort = _LOWER_MIXER_RST;
	systemMotors[LOWER_MIXER].controlPins.ms1 = LOWER_MIXER_MS1;
	systemMotors[LOWER_MIXER].controlPins.ms0 = LOWER_MIXER_MS0;
	systemMotors[LOWER_MIXER].controlPins.en = LOWER_MIXER_EN;
	systemMotors[LOWER_MIXER].controlPins._cs = _LOWER_MIXER_CS;
	systemMotors[LOWER_MIXER].controlPins.diag = LOWER_MIXER_DIAG;
	systemMotors[LOWER_MIXER].PWMTC = PWM_TCD0;
	systemMotors[LOWER_MIXER].PWMChan = PWM_CH_B;
	systemMotors[LOWER_MIXER].PWMDuty = 50;
	
	//Initialize uC pins for pump stepper driver and pwm settings
	systemMotors[PUMP].motorID = PUMP;
	systemMotors[PUMP].controlPins.dir = PUMP_DIR;
	systemMotors[PUMP].controlPins.step = PUMP_STEP;
	systemMotors[PUMP].controlPins._rstPort = _PUMP_RST;
	systemMotors[PUMP].controlPins.ms1 = PUMP_MS1;
	systemMotors[PUMP].controlPins.ms0 = PUMP_MS0;
	//systemMotors[PUMP].controlPins.en = PUMP_EN; //This pin was changed to something else, define unused pin so the ioport call is maintained
	systemMotors[PUMP].controlPins.en = IOPORT_CREATE_PIN(PORTF, 6);
	
	systemMotors[PUMP].controlPins._cs = _PUMP_CS;
	systemMotors[PUMP].controlPins.diag = PUMP_DIAG;
	systemMotors[PUMP].PWMTC = PWM_TCC0;
	systemMotors[PUMP].PWMChan = PWM_CH_A;
	systemMotors[PUMP].PWMDuty = 50;
	
	//Intialize PWM structs for motor
	for (uint8_t i = 0; i < NUM_MOTORS; i++)
	{
		init_motor_timers(&systemMotors[i]);
	}
	
	//Overflow interrupt for rod position
	tc_set_overflow_interrupt_callback(&TCE1, rod_position_timer_overflow_callback);
	tc_set_overflow_interrupt_level(&TCE1, TC_INT_LVL_MED);
	
	//Overflow interrupt for mixer
	tc_set_overflow_interrupt_callback(&TCF0, mixer_timer_overflow_callback);
	tc_set_overflow_interrupt_level(&TCF0, TC_INT_LVL_MED);
	
	//Overflow interrupt for lower mixer
	tc_set_overflow_interrupt_callback(&TCD0, lower_mixer_timer_overflow_callback);
	tc_set_overflow_interrupt_level(&TCD0, TC_INT_LVL_MED);
	
	//Overflow interrupt for pump
	tc_set_overflow_interrupt_callback(&TCC0, pump_timer_overflow_callback);
	tc_set_overflow_interrupt_level(&TCC0, TC_INT_LVL_MED);
}

//////////////////////////////////////////////////////////////////////////
//Initializes registers for SPI control of stepper motors
//////////////////////////////////////////////////////////////////////////
void init_steppers_SPI_control(void)
{
	//Initialize stepper motor register values
	for (uint8_t i = 0; i < NUM_MOTORS; i++)
	{		
		systemMotors[i].config0Reg = 0;
		SET_BIT(systemMotors[i].config0Reg, CFG0_SYR);		//Synchronous rectification (default)
		SET_BIT(systemMotors[i].config0Reg, CFG0_MXI1);		//Set phase current at 100% (default)
		SET_BIT(systemMotors[i].config0Reg, CFG0_MXI0);
		SET_BIT(systemMotors[i].config0Reg, CFG0_PFD2);		//Set decay time to 8uS (default)
		SET_BIT(systemMotors[i].config0Reg, CFG0_TBK0);		//Set blank time to 1.5uS (default)
		SET_BIT(systemMotors[i].config0Reg, CFG0_TFFRQ2);	//(default)
		SET_BIT(systemMotors[i].config0Reg, CFG0_TFFRQ1);
		
		stepperTxBuffer[0] = (systemMotors[i].config0Reg >> 8); //Send updated register info to motor driver
		stepperTxBuffer[1] = systemMotors[i].config0Reg & 0x00FF;
		send_msg(&STEPPER_SPI_PORT, systemMotors[i].controlPins._cs, stepperTxBuffer, 2);
		
		systemMotors[i].config1Reg = 0;
		SET_BIT(systemMotors[i].config1Reg, CFG1_ADDR0);	//Set register address
		SET_BIT(systemMotors[i].config1Reg, CFG1_TSC1);		//Set overcurrent fault delay to 2uS (default)
		
		stepperTxBuffer[0] = (systemMotors[i].config1Reg >> 8); //Send updated register info to motor driver
		stepperTxBuffer[1] = systemMotors[i].config1Reg & 0x00FF;
		send_msg(&STEPPER_SPI_PORT, systemMotors[i].controlPins._cs, stepperTxBuffer, 2);
		
		systemMotors[i].runReg = 0;
		SET_BIT(systemMotors[i].runReg, RUN_ADDR1);	//Set register address
		SET_BIT(systemMotors[i].runReg, RUN_DCY0);	//Set decay mode (default)
		SET_BIT(systemMotors[i].runReg, RUN_SLEW);	//Set slew rate (default)
		SET_BIT(systemMotors[i].runReg, RUN_OL0);	//Set open load current thresh to 30% (default)
		
		stepperTxBuffer[0] = (systemMotors[i].runReg >> 8); //Send updated register info to motor driver
		stepperTxBuffer[1] = systemMotors[i].runReg & 0x00FF;
		send_msg(&STEPPER_SPI_PORT, systemMotors[i].controlPins._cs, stepperTxBuffer, 2);
	}
}

//////////////////////////////////////////////////////////////////////////
//Initialize TCC1 for use as 1ms system timer
//////////////////////////////////////////////////////////////////////////
void init_sys_timer(void)
{	
	//32MHz sysclk / 64 prescale = 500kHz (2us period)
	//2us period * 500 counts = 1ms clock
	tc_enable(&SYS_CLK_TC);
	tc_set_overflow_interrupt_callback(&SYS_CLK_TC, sys_clk_overflow_callback);
	tc_set_wgm(&SYS_CLK_TC, TC_WG_NORMAL);
	tc_write_period(&SYS_CLK_TC, 500);
	tc_set_overflow_interrupt_level(&SYS_CLK_TC, TC_INT_LVL_HI);
	tc_write_clock_source(&SYS_CLK_TC, TC_CLKSEL_DIV64_gc);	
}

//////////////////////////////////////////////////////////////////////////
//Initialize TCD1 for bit banging heater output
//////////////////////////////////////////////////////////////////////////
void init_heater_timer(void)
{
	//TCD1 used for heater (1kHz)
	tc_enable(&HEAT_PWM_TC);
	tc_set_cca_interrupt_callback(&HEAT_PWM_TC, heater_cc_overflow_callback);
	tc_set_overflow_interrupt_callback(&HEAT_PWM_TC, heater_timer_overflow_callback);
	tc_set_wgm(&HEAT_PWM_TC, TC_WG_NORMAL);
	tc_write_period(&HEAT_PWM_TC, HEAT_PWM_TC_PERIOD);
	tc_write_cc(&HEAT_PWM_TC, HEAT_PWM_CCCHAN, 100); //Set to a default value
	tc_enable_cc_channels(&HEAT_PWM_TC, HEAT_PWM_CCCHAN);
	tc_set_cca_interrupt_level(&HEAT_PWM_TC, TC_INT_LVL_LO);
	tc_set_overflow_interrupt_level(&HEAT_PWM_TC, TC_INT_LVL_LO);
	tc_write_clock_source(&HEAT_PWM_TC, TC_CLKSEL_DIV64_gc);
}

//////////////////////////////////////////////////////////////////////////
//Initialize TCD1 for bit banging fan output
//////////////////////////////////////////////////////////////////////////
void init_fan_timer(void)
{
	//Fan timer shares the same timer counter (TCD1) used for heater
	//If heater timer isn't initialized, need to uncomment this section
	//tc_enable(&FAN_PWM_TC);
	//tc_set_overflow_interrupt_callback(&FAN_PWM_TC, heater_timer_overflow_callback);
	//tc_set_wgm(&FAN_PWM_TC, TC_WG_NORMAL);
	//tc_write_period(&FAN_PWM_TC, FAN_PWM_TC_PERIOD);
	//tc_set_overflow_interrupt_level(&FAN_PWM_TC, TC_INT_LVL_LO);
	//tc_write_clock_source(&FAN_PWM_TC, TC_CLKSEL_DIV64_gc);
	
	tc_set_ccb_interrupt_callback(&FAN_PWM_TC, fan_cc_overflow_callback);
	tc_write_cc(&FAN_PWM_TC, FAN_PWM_CCCHAN, 100); //Set to a default value
	tc_enable_cc_channels(&FAN_PWM_TC, FAN_PWM_CCCHAN);
	tc_set_ccb_interrupt_level(&FAN_PWM_TC, TC_INT_LVL_LO);
}

//////////////////////////////////////////////////////////////////////////
//Split up received USB message into USBMsg parts
//////////////////////////////////////////////////////////////////////////
void parse_received_USB_msg(void)
{
	//Aya standard "GET" msg format is: "GET CMD<CR>"
	//Aya standard "SET" msg format is: "SET CMD DATA DATA2<CR>
	uint8_t curChar = 0;
	uint8_t prevChar = 0;
	uint16_t msgIndex = 0;
	uint16_t indexOffset = 0;
	uint16_t msgLength = 0;
	char cmdEEPromString[] = "EEPROM";
	
	clear_receivedUSBMsg(); //Clear out receivedUSB message
	msgLength = get_last_msg_length(); //Get message length
	
	//Get command type from receivedUSBMsg (Should be either GET or SET)
	curChar = USBRxBuff[0];
	for (uint16_t i = msgIndex; i < msgLength; i++)
	{
		if ((curChar != SEPERATOR_CHAR) && (curChar != NULL_CHAR))
		{
			receivedUSBMsg.cmd_type_char[msgIndex] = curChar;
			curChar = USBRxBuff[++msgIndex];
		}
		else
		{
			break; //Loop through until get to first " " or null
		}
	}
	receivedUSBMsg.cmd_type = check_cmd_type(receivedUSBMsg.cmd_type_char); //Determine received command type
	
	//Get command from receivedUSBMsg
	curChar = USBRxBuff[++msgIndex];
	indexOffset = msgIndex; //Move starting point of loop
	for (uint16_t i = msgIndex - 1; i < msgLength; i++)
	{
		if ((curChar != END_OF_DATA_CHAR) && (curChar != NULL_CHAR) && ((curChar < 48) || (curChar > 57)))
		{
			receivedUSBMsg.cmd_char[msgIndex - indexOffset] = curChar;
			prevChar = curChar;
			curChar = USBRxBuff[++msgIndex];
		}
		else
		{
			break; //Loop through until get to <CR> or null, or a 0-9 digit (which would be data)
		}
	}
	
	receivedUSBMsg.motorDir = DIR_CW; //From Aya, assume positive if no sign given
	if (prevChar == 43)
	{
		//Remove extra space and + sign from message
		receivedUSBMsg.motorDir = DIR_CW;
		receivedUSBMsg.cmd_char[msgIndex - indexOffset - 2] = 0;
		receivedUSBMsg.cmd_char[msgIndex - indexOffset - 1] = 0;
	}
	else if (prevChar == 45)
	{
		//Remove extra space and - sign from message
		receivedUSBMsg.motorDir = DIR_CCW;
		receivedUSBMsg.cmd_char[msgIndex - indexOffset - 2] = 0;
		receivedUSBMsg.cmd_char[msgIndex - indexOffset - 1] = 0;
	}
	
	if (prevChar == SEPERATOR_CHAR)
	{
		receivedUSBMsg.cmd_char[msgIndex - indexOffset - 1] = 0; //Remove extra space from message
	}
	
	receivedUSBMsg.cmd = convert_cmd_str_to_char(receivedUSBMsg.cmd_char); //Convert received command from string to integer value for switch case
	if ((receivedUSBMsg.cmd >= 7) && (receivedUSBMsg.cmd <= 13))
	{
		receivedUSBMsg.motorSel = receivedUSBMsg.cmd_char[4]; //These commands have a motor selection character which is in position 4
	}
	
	//Any remaining characters in receivedUSBMsg is data
	indexOffset = msgIndex;
	for (uint16_t i = msgIndex - 1; i < msgLength; i++)
	{
		if ((curChar != END_OF_DATA_CHAR) && (curChar != SEPERATOR_CHAR) && (curChar != NULL_CHAR))
		{
			receivedUSBMsg.cmd_data1[msgIndex - indexOffset] = curChar;
			curChar = USBRxBuff[++msgIndex];
		}
		else
		{
			break; //Loop through until get to end or " " or null
		}
	}
	
	if (curChar != END_OF_DATA_CHAR)
	{
		curChar = USBRxBuff[++msgIndex]; //Only read if we haven't recieved an EOD already
	}
	indexOffset = msgIndex;
	for (uint16_t i = msgIndex - 1; i < msgLength; i++)
	{
		if ((curChar != END_OF_DATA_CHAR) && (curChar != SEPERATOR_CHAR) && (curChar != NULL_CHAR))
		{
			if (receivedUSBMsg.cmd != CMD_EEPROM)
			{
				if ((curChar == PLUS_CHAR) || (curChar == DASH_CHAR))
				{
					indexOffset++; //Don't add + or - sign to data value
				}
				else
				{
					receivedUSBMsg.cmd_data2[msgIndex - indexOffset] = curChar;
				}
			}
			else
			{
				receivedUSBMsg.cmd_data2[msgIndex - indexOffset] = curChar; //Parse all data if using EEProm command
			}
			
			curChar = USBRxBuff[++msgIndex];
		}
		else
		{
			break; //Loop through until get to end or " " or null
		}
	}
	
	if (curChar != END_OF_DATA_CHAR)
	{
		curChar = USBRxBuff[++msgIndex]; //Only read if we haven't received an EOD already
	}
	indexOffset = msgIndex;
	for (uint16_t i = msgIndex - 1; i < msgLength; i++)
	{
		if ((curChar != END_OF_DATA_CHAR) && (curChar != SEPERATOR_CHAR) && (curChar != NULL_CHAR))
		{
			//if (curChar != SEPERATOR_CHAR)
			//{
				//receivedUSBMsg.cmd_data3[msgIndex - indexOffset] = curChar;
			//}
			
			if ((curChar == PLUS_CHAR) || (curChar == DASH_CHAR))
			{
				indexOffset++; //Don't add + or - sign to data value
			}
			else
			{
				receivedUSBMsg.cmd_data3[msgIndex - indexOffset] = curChar;
			}
			curChar = USBRxBuff[++msgIndex];
		}
		else
		{
			break; //Loop through until get to end or " " or null
		}
	}
	
	if (curChar != END_OF_DATA_CHAR)
	{
		curChar = USBRxBuff[++msgIndex]; //Only read if we haven't received an EOD already
	}
	indexOffset = msgIndex;
	for (uint16_t i = msgIndex - 1; i < msgLength; i++)
	{
		if ((curChar != END_OF_DATA_CHAR) && (curChar != NULL_CHAR))
		{
			//if (curChar != SEPERATOR_CHAR)
			//{
				//receivedUSBMsg.cmd_data4[msgIndex - indexOffset] = curChar;
			//}
			if ((curChar == PLUS_CHAR) || (curChar == DASH_CHAR))
			{
				indexOffset++; //Don't add + or - sign to data value
			}
			else
			{
				receivedUSBMsg.cmd_data4[msgIndex - indexOffset] = curChar;
			}
			curChar = USBRxBuff[++msgIndex];
		}
		else
		{
			break; //Loop through until get to end or null
		}
	}
	
	//Convert command data from string to integers
	receivedUSBMsg.cmd_data_val1 = convert_msg_data_to_num(&receivedUSBMsg.cmd_data1);
	receivedUSBMsg.cmd_data_val2 = convert_msg_data_to_num(&receivedUSBMsg.cmd_data2);
	receivedUSBMsg.cmd_data_val3 = convert_msg_data_to_num(&receivedUSBMsg.cmd_data3);
	receivedUSBMsg.cmd_data_val4 = convert_msg_data_to_num(&receivedUSBMsg.cmd_data4);
	
	if (curChar != END_OF_DATA_CHAR)
	{
		receivedUSBMsg.validMsg = FALSE; //If last received character wasn't EOD, not valid msg
	}
	else
	{
		receivedUSBMsg.validMsg = TRUE;
	}
}

//////////////////////////////////////////////////////////////////////////
//Check command type string for type of command (Should be either GET or SET)
//////////////////////////////////////////////////////////////////////////
uint8_t check_cmd_type(char *cmd)
{
	uint8_t cmpCount = 0;	
	char *initCmdPtr = cmd;
	uint8_t *cmpCmdPtr = &GET_CMD_CHAR; //Set to check for "GET" string

	while ((*cmd++ == *cmpCmdPtr++) && (cmpCount != strlen(GET_CMD_CHAR)))
	{
		cmpCount++;
	}
	if (cmpCount == strlen(GET_CMD_CHAR))
	{
		return CMD_TYPE_GET; //Received cmd type matches "GET"
	}
	
	cmpCmdPtr = &SET_CMD_CHAR; //Set to check for "SET" string
	cmd = initCmdPtr;
	cmpCount = 0;
	while ((*cmd++ == *cmpCmdPtr++) && (cmpCount != strlen(SET_CMD_CHAR)))
	{
		cmpCount++;
	}
	if (cmpCount == strlen(SET_CMD_CHAR))
	{
		return CMD_TYPE_SET; //Received cmd type matches "SET"
	}
	
	return CMD_TYPE_UNKNOWN; //Cmd type didn't match any known commands
}

//////////////////////////////////////////////////////////////////////////
// Loops through list of valid command strings to find match
//////////////////////////////////////////////////////////////////////////
uint8_t convert_cmd_str_to_char(char *cmd)
{
	uint8_t endOfCmp = FALSE;
	uint8_t prevChar = 0;
	uint8_t cmpStrLen = 0;
	uint8_t cmpCount = 0;
	char *initCmdPtr = cmd;
	uint8_t *cmpCmdPtr = &(*CMD_LIST[0]); //Set to check for 1st command
	prevChar = *cmd;
	for (uint8_t i = 0; i < sizeof(CMD_LIST); i++)
	{		
		while (!endOfCmp)
		{
			if (*cmd == *cmpCmdPtr)
			{
				if (*cmd != 0)
				{
					cmpCount++;
				}
				prevChar = *cmd;
				cmd++;
				cmpCmdPtr++;
				
				//If next character is null then exit loop
				if (*cmd == 0)
				{
					endOfCmp = TRUE;
				}
			}
			else
			{
				if (prevChar != DASH_CHAR) //Character after dash is motor selection
				{
					if ((*cmpCmdPtr == 0) && (*cmd != 0))
					{
						//Overflowed compare string, but weren't at end of cmd string. Reset compare count
						cmpCount = 0;
					}
					endOfCmp = TRUE;
				}
				else
				{
					prevChar = *cmd;
					cmpCount++;
					cmd++;
					cmpCmdPtr++;
				}
			}
		}
		
		cmpStrLen = strlen(CMD_LIST[i]);
		if (cmpCount == cmpStrLen)
		{
			//Received cmd matches
			return (i + 1);
		}
		
		cmpCmdPtr = &(*CMD_LIST[i + 1]); //Set to check for next cmd in list
		cmd = initCmdPtr; //Reset cmd pointer
		endOfCmp = FALSE;
		cmpCount = 0;
	}
	
	return UNKNOWN_CMD;
}

//////////////////////////////////////////////////////////////////////////
// Converts ascii string to float
//////////////////////////////////////////////////////////////////////////
float convert_msg_data_to_num(char *msgData)
{
	return atof(msgData);
}

//////////////////////////////////////////////////////////////////////////
// Converts int to ascii string
//////////////////////////////////////////////////////////////////////////
void convert_int_to_string(uint32_t val, char *convString)
{
	itoa(val, convString, 10);
}

//////////////////////////////////////////////////////////////////////////
// Converts float to ascii string
//////////////////////////////////////////////////////////////////////////
void convert_float_to_string(float val, char *convString, uint8_t convEnd)
{
	char strInt[8] = {};
	char strDec[8] = {};
	uint8_t chrIndex = 0;
	uint8_t i = 0;
	float tmpVal = 0.0;
	
	do 
	{
		convString[i] = 0x00; // Flush out any old junk
		i++;
	} while (i < convEnd);
	
	// (double __val, signed char __width, unsigned char __prec, char *__s)
	convString = dtostrf(val, 1, 4, convString);			// converts the double to a string,  Min. Width including "." and "-", digits after the decimal, "[-]d.ddd"
}

//////////////////////////////////////////////////////////////////////////
// Converts ascii string to motor array index
//////////////////////////////////////////////////////////////////////////
uint8_t convert_received_motor_to_index(uint8_t recChar)
{
	switch (recChar)
	{
		case 82:
			return ROD_POS;
			break;
		case 85:
			return MIXER;
			break;
		case 76:
			return LOWER_MIXER;
			break;
		case 80:
			return PUMP;
			break;
		default:
			return 255; //No match for received motor (this will be caught in error detection later)
			break;
	}
}

//////////////////////////////////////////////////////////////////////////
// Checks to make sure selected motor is within ranges
//////////////////////////////////////////////////////////////////////////
uint8_t check_motor_index(uint8_t motorIndex)
{
	if ((motorIndex >= 0) && (motorIndex <= 3))
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

//////////////////////////////////////////////////////////////////////////
// Handles command logic of received message
//////////////////////////////////////////////////////////////////////////
void handle_received_USB_cmd(void)
{
	uint8_t chrIndex = 0;
	uint8_t validCmd = FALSE; //Assume bad command until get good one
	uint8_t motorSel = 255;
	
	flush_USB_tx_buffer(); //Clear out Tx buffer
	
	LEDOnTime = 250; //LED time for command blink codes
	LEDOffTime = 250;
	
	if (!receivedUSBMsg.validMsg)
	{
		receivedUSBMsg.cmd = 255; //This will send it into the default case
	}
	
	switch (receivedUSBMsg.cmd)
	{
		case CMD_ROD:
			if (receivedUSBMsg.cmd_type == CMD_TYPE_GET)
			{
				//Gets rod switch position input state
				//Returns 1 if switch is activated otherwise 0
				for (uint8_t i = 0; i < strlen(ROD_CMD_CHAR); i++) //Add command string to Tx buffer
				{
					USBTxBuff[chrIndex++] = ROD_CMD_CHAR[i];
				}
				USBTxBuff[chrIndex++] = SEPERATOR_CHAR;
				USBTxBuff[chrIndex++] = (uint8_t)!ioport_get_pin_level(ROD_POS_SWITCH) + ASCII_OFFSET; //Rod switch is inverted
				USBTxBuff[chrIndex++] = RETURN_CHAR;
				
				LEDBlinkCount = CMD_ROD; //Update blink count to show CMD_ROD
				validCmd = TRUE; //Final msg should look like "ROD 1<CR>"
			}
			break;
		case CMD_TEC:
			if (receivedUSBMsg.cmd_type == CMD_TYPE_GET)
			{
				//Gets TEC output duty cycle
				char dutyChar[4] = {};
				
				for (uint8_t i = 0; i < strlen(TEC_CMD_CHAR); i++) //Add command string to Tx buffer
				{
					USBTxBuff[chrIndex++] = TEC_CMD_CHAR[i];
				}
				USBTxBuff[chrIndex++] = SEPERATOR_CHAR;
				
				if (TECPWMDutyA != 0) //TEC PWM A is positive, append + sign
				{
					convert_int_to_string(TECPWMDutyA, dutyChar);
					USBTxBuff[chrIndex++] = PLUS_CHAR;
				}
				else //TEC PWM B is negative, append - sign
				{
					convert_int_to_string(TECPWMDutyB, dutyChar);
					USBTxBuff[chrIndex++] = DASH_CHAR;
				}
				for (uint8_t i = 0; i < strlen(dutyChar); i++)
				{
					USBTxBuff[chrIndex++] = dutyChar[i];
				}
				USBTxBuff[chrIndex++] = RETURN_CHAR;
				
				LEDBlinkCount = CMD_TEC; //Update blink count to show CMD_TEC
				validCmd = TRUE; //Final msg should look like "TEC +100<CR>"
			}
			else if (receivedUSBMsg.cmd_type == CMD_TYPE_SET)
			{
				//Sets TEC output duty cycle
				if (check_received_data_limits(receivedUSBMsg.cmd_data_val1, DUTY_CYCLE_MIN, DUTY_CYCLE_MAX))
				{
					if (check_received_data_limits(receivedUSBMsg.cmd_data_val2, DUTY_CYCLE_MIN, DUTY_CYCLE_MAX))
					{
						if (receivedUSBMsg.motorDir != DIR_UNKNOWN)
						{
							//Commanded duty cycle is within limits
							recover_from_OTSD(); //Send reset pulses in case TEC was in shutdown
							
							set_TEC_pwm_A_freq(TEC_FREQ);
							set_TEC_pwm_B_freq(TEC_FREQ); //This is redundant as they are both tied to same TC
							
							if (receivedUSBMsg.motorDir == 0) //Hijack motor direction to get sign of data
							{								
								TECPWMDutyA = (uint8_t)receivedUSBMsg.cmd_data_val1; //PWM A correlates to a positive TEC output
								TECPWMDutyB = 0;
								
								//Setup TEC state for gradually setting output. TEC didn't like going directly from one state to another
								if ((TECPWMDutyA != 0) && (curTECPWMDutyB == 0))
								{
									TECState = TEC_POS; //TEC was previously set to a positive output
								}
								else
								{
									TECState = TEC_OFF;
								}
							}
							else
							{
								TECPWMDutyA = 0;
								TECPWMDutyB = (uint8_t)receivedUSBMsg.cmd_data_val1; //PWM B correlates to a negative TEC output
								
								//Setup TEC state for gradually setting output. TEC didn't like going directly from one state to another
								if (curTECPWMDutyA != 0)
								{
									TECState = TEC_OFF; //TEC was previously set to a positive output, need to transition through 0 first
								}
								else
								{
									TECState = TEC_NEG; //TEC was previously off or negative output
								}
							}
							
							for (uint8_t i = 0; i < strlen(OK_CMD_RESP); i++) //Send back OK cmd response
							{
								USBTxBuff[chrIndex++] = OK_CMD_RESP[i];
							}
							USBTxBuff[chrIndex++] = RETURN_CHAR;
							
							LEDBlinkCount = CMD_TEC; //Update blink count to show CMD_TEC
							validCmd = TRUE; //Final msg should look like "OK<CR>"
						}
					}
				}
			}
			break;
		case CMD_TEMP_SP:
			if (receivedUSBMsg.cmd_type == CMD_TYPE_GET)
			{
				//Gets current temperature setpoint
				char tempChar[8] = {};
				
				convert_float_to_string(tempSetPoint, tempChar, 8);
				
				for (uint8_t i = 0; i < strlen(TEMP_SP_CMD_CHAR); i++) //Add command string to Tx buffer
				{
					USBTxBuff[chrIndex++] = TEMP_SP_CMD_CHAR[i];
				}
				USBTxBuff[chrIndex++] = SEPERATOR_CHAR;
				for (uint8_t i = 0; i < strlen(tempChar); i++) //Add current temperature setpoint to Tx buffer
				{
					USBTxBuff[chrIndex++] = tempChar[i];
				}
				USBTxBuff[chrIndex++] = RETURN_CHAR;
				
				LEDBlinkCount = CMD_TEMP_SP; //Update blink count to show CMD_TEMP_SP
				validCmd = TRUE; //Final msg should look like "TEMP SP 38.0<CR>"
			}
			else if (receivedUSBMsg.cmd_type == CMD_TYPE_SET)
			{
				//Sets current temperature setpoint
				if (check_received_data_limits(receivedUSBMsg.cmd_data_val1, TEMP_SP_MIN, TEMP_SP_MAX))
				{
					//Commanded temperature setpoint is within limits
					heaterEnabled = TRUE;
					tempSetPoint = receivedUSBMsg.cmd_data_val1;
					calc_heater_adj(); //Calculate heater DC offset from current setpoint
					
					for (uint8_t i = 0; i < strlen(OK_CMD_RESP); i++) //Send back OK cmd response
					{
						USBTxBuff[chrIndex++] = OK_CMD_RESP[i];
					}
					USBTxBuff[chrIndex++] = RETURN_CHAR;
					
					LEDBlinkCount = CMD_TEMP_SP; //Update blink count to show CMD_TEMP_SP
					validCmd = TRUE; //Final msg should look like "OK<CR>"
				}
			}
			break;
		case CMD_TEMP_SP_OFF:
			if (receivedUSBMsg.cmd_type == CMD_TYPE_SET)
			{
				tempSetPoint = 0;
				heaterEnabled = FALSE;
				
				for (uint8_t i = 0; i < strlen(OK_CMD_RESP); i++) //Send back OK cmd response
				{
					USBTxBuff[chrIndex++] = OK_CMD_RESP[i];
				}
				USBTxBuff[chrIndex++] = RETURN_CHAR;
				
				LEDBlinkCount = CMD_TEMP_SP_OFF; //Update blink count to show CMD_TEMP_SP_OFF
				validCmd = TRUE; //Final msg should look like "OK<CR>"
			}
			break;
		case CMD_TEMP:
			if (receivedUSBMsg.cmd_type == CMD_TYPE_GET)
			{
				char tempChar[8] = {};
				char tempPIDVal[4] = {};			
				
				if (!heaterEnabled)
				{
					//Temperature is only read if heater is enabled. If request is made, get current temperature
					curTemp = 0;
					if (tempProbeType == THERMISTOR)
					{
						curTemp = calc_temp_from_ADC_reading();
					}
					else
					{
						curTemp = calc_temp_from_RTD_reading(get_RTD_sensor_reading());
					}
				}
				
				convert_float_to_string(curTemp, tempChar, 8);
				convert_int_to_string((uint16_t)heaterPID.PIDVal, tempPIDVal);
				
				//Gets current temperature setpoint
				for (uint8_t i = 0; i < strlen(TEMP_CMD_CHAR); i++) //Add command string to Tx buffer
				{
					USBTxBuff[chrIndex++] = TEMP_CMD_CHAR[i];
				}
				USBTxBuff[chrIndex++] = SEPERATOR_CHAR;
				for (uint8_t i = 0; i < strlen(tempChar); i++) //Add current temperature to Tx buffer
				{
					USBTxBuff[chrIndex++] = tempChar[i];
				}
				USBTxBuff[chrIndex++] = SEPERATOR_CHAR;
				for (uint8_t i = 0; i < strlen(tempPIDVal); i++) //Add current PID output to Tx buffer
				{
					USBTxBuff[chrIndex++] = tempPIDVal[i];
				}
				USBTxBuff[chrIndex++] = RETURN_CHAR;
				
				LEDBlinkCount = CMD_TEMP; //Update blink count to show CMD_TEMP
				validCmd = TRUE; //Final msg should look like "TEMP 36.4 56.2<CR>"
			}
			break;
		case CMD_TEMP_PID:
			if (receivedUSBMsg.cmd_type == CMD_TYPE_GET)
			{
				//Gets temperature PID settings
				char pSetting[8] = {};
				char iSetting[8] = {};
				char dSetting[8] = {};
				
				convert_float_to_string(heaterPID.pGain, pSetting, 8);
				convert_float_to_string(heaterPID.iGain, iSetting, 8);
				convert_float_to_string(heaterPID.dGain, dSetting, 8);
				
				for (uint8_t i = 0; i < strlen(TEMP_PID_CMD_CHAR); i++) //Add command string to Tx buffer
				{
					USBTxBuff[chrIndex++] = TEMP_PID_CMD_CHAR[i];
				}
				USBTxBuff[chrIndex++] = SEPERATOR_CHAR;
				for (uint8_t i = 0; i < strlen(pSetting); i++) //Add PID p setting to Tx buffer
				{
					if (pSetting[i] != 0)
					{
						USBTxBuff[chrIndex++] = pSetting[i];
					}
				}
				USBTxBuff[chrIndex++] = SEPERATOR_CHAR;
				for (uint8_t i = 0; i < strlen(iSetting); i++) //Add PID i setting to Tx buffer
				{
					if (iSetting[i] != 0)
					{
						USBTxBuff[chrIndex++] = iSetting[i];
					}
				}
				USBTxBuff[chrIndex++] = SEPERATOR_CHAR;
				for (uint8_t i = 0; i < strlen(dSetting); i++) //Add PID d setting to Tx buffer
				{
					if (dSetting[i] != 0)
					{
						USBTxBuff[chrIndex++] = dSetting[i];
					}
				}
				USBTxBuff[chrIndex] = RETURN_CHAR;
				
				LEDBlinkCount = CMD_TEMP_PID; //Update blink count to show CMD_TEMP_PID
				validCmd = TRUE; //Final msg should look like "TEMP PID 8.5 0.25 5<CR>"
			}
			else if (receivedUSBMsg.cmd_type == CMD_TYPE_SET)
			{
				//Sets temperature PID settings
				if (check_received_data_limits(receivedUSBMsg.cmd_data_val1, TEMP_PID_MIN, TEMP_PID_MAX))
				{
					if (check_received_data_limits(receivedUSBMsg.cmd_data_val2, TEMP_PID_MIN, TEMP_PID_MAX))
					{
						if (check_received_data_limits(receivedUSBMsg.cmd_data_val3, TEMP_PID_MIN, TEMP_PID_MAX))
						{
							//All commanded values are within limits
							heaterPID.pGain = receivedUSBMsg.cmd_data_val1;
							heaterPID.iGain = receivedUSBMsg.cmd_data_val2;
							heaterPID.dGain = receivedUSBMsg.cmd_data_val3;
							
							heaterPID.iMax = (100 / heaterPID.iGain) * OUTPUT_SCALE; //Limit min/max of PID to 100 times the gain setpoint
							heaterPID.iMin = 0;
							
							// Write changed parameters to EEprom so they're stored
							uint32_t heatPIDP = (heaterPID.pGain * 100);
							uint32_t heatPIDI = (heaterPID.iGain * 100);
							uint32_t heatPIDD = (heaterPID.dGain * 100);
							
							write_PID_to_NVM(PID_P_ADR, &heatPIDP, 4);
							write_PID_to_NVM(PID_I_ADR, &heatPIDI, 4);
							write_PID_to_NVM(PID_D_ADR, &heatPIDD, 4);
							
							for (uint8_t i = 0; i < strlen(OK_CMD_RESP); i++) //Send back OK cmd response
							{
								USBTxBuff[chrIndex++] = OK_CMD_RESP[i];
							}
							USBTxBuff[chrIndex++] = RETURN_CHAR;
							
							LEDBlinkCount = CMD_TEMP_PID; //Update blink count to show CMD_TEMP_PID
							validCmd = TRUE; //Final msg should look like "OK<CR>"
						}
					}
				}
			}
			break;
		case CMD_MTR_CUR:
			if (receivedUSBMsg.cmd_type == CMD_TYPE_GET)
			{
				char curHoldSetting[8] = {};
				
				motorSel = convert_received_motor_to_index(receivedUSBMsg.motorSel); //Get motor selection from received msg
				if (check_motor_index(motorSel)) //Ensure motor selection is valid
				{
					convert_int_to_string((uint16_t)get_motor_hold_current_percent(&systemMotors[motorSel]), curHoldSetting);
					
					for (uint8_t i = 0; i < strlen(MTR_CUR_CMD_CHAR); i++) //Add command string to Tx buffer
					{
						if (i != 4)
						{
							USBTxBuff[chrIndex++] = MTR_CUR_CMD_CHAR[i];
						}
						else
						{
							USBTxBuff[chrIndex++] = receivedUSBMsg.motorSel; //Motor letter is variable, select this based on motor selection
						}
					}
					USBTxBuff[chrIndex++] = SEPERATOR_CHAR;
					for (uint8_t i = 0; i < strlen(curHoldSetting); i++) //Add motor hold current setting to Tx buffer
					{
						USBTxBuff[chrIndex++] = curHoldSetting[i];
					}
					USBTxBuff[chrIndex++] = RETURN_CHAR;
					
					LEDBlinkCount = CMD_MTR_CUR; //Update blink count to show CMD_MTR_CUR
					validCmd = TRUE; //Final msg should look like "MTR-R C 100<CR>"
				}
			}
			else if (receivedUSBMsg.cmd_type == CMD_TYPE_SET)
			{
				motorSel = convert_received_motor_to_index(receivedUSBMsg.motorSel); //Get motor selection from received msg
				if (check_motor_index(motorSel)) //Ensure motor selection is valid
				{
					if (set_motor_hold_current(&systemMotors[motorSel], receivedUSBMsg.cmd_data_val1))
					{
						for (uint8_t i = 0; i < strlen(OK_CMD_RESP); i++) //Send back OK cmd response
						{
							USBTxBuff[chrIndex++] = OK_CMD_RESP[i];
						}
						USBTxBuff[chrIndex++] = RETURN_CHAR;
						
						LEDBlinkCount = CMD_MTR_CUR; //Update blink count to show CMD_MTR_CUR
						validCmd = TRUE; //Final msg should look like "OK<CR>"
					}
				}
			}
			break;
		case CMD_MTR_RATE:
			if (receivedUSBMsg.cmd_type == CMD_TYPE_SET)
			{
				if (check_received_data_limits(receivedUSBMsg.cmd_data_val1, MTR_S_MIN, MTR_S_MAX)) //Ensure motor speed setting is within limits
				{
					if (check_received_data_limits(receivedUSBMsg.cmd_data_val2, MTR_AD_MIN, MTR_AD_MAX)) //Ensure motor acceleration setting is within limits
					{
						motorSel = convert_received_motor_to_index(receivedUSBMsg.motorSel); //Get motor selection from received msg
						if (check_motor_index(motorSel)) //Ensure motor selection is valid
						{
							if (receivedUSBMsg.motorDir != DIR_UNKNOWN) //Ensure a direction was specified
							{
								systemMotors[motorSel].specifiedSteps = FALSE; //This command sets a constant speed, disabled specified steps
								set_motor_direction(&systemMotors[motorSel], receivedUSBMsg.motorDir); //Update motor direction
								set_motor_accel_time(&systemMotors[motorSel], receivedUSBMsg.cmd_data_val2); //Deceleration is no longer a unique parameter, will just accelerate to set speed
								set_motor_decel_time(&systemMotors[motorSel], receivedUSBMsg.cmd_data_val2); //Still need to set it though because the deceleration property is used on slowing down
								set_motor_speed(&systemMotors[motorSel], (uint32_t)receivedUSBMsg.cmd_data_val1); //Update motor speed
								
								if ((int)receivedUSBMsg.cmd_data_val1 == 0)
								{
									stop_motor(&systemMotors[motorSel]); //Stop/decelerate the motor if the commanded speed was 0
								}
								else
								{
									start_motor(&systemMotors[motorSel]); //Ensure the motor is started if the commanded speed wasn't 0
								}
								
								for (uint8_t i = 0; i < strlen(OK_CMD_RESP); i++) //Send back OK cmd response
								{
									USBTxBuff[chrIndex++] = OK_CMD_RESP[i];
								}
								USBTxBuff[chrIndex++] = RETURN_CHAR;
								
								LEDBlinkCount = CMD_MTR_RATE; //Update blink count to show CMD_MTR_RATE
								validCmd = TRUE; //Final msg should look like "OK<CR>"
							}
						}
					}
				}
			}
			break;
		case CMD_MTR_RATE_DUR:
			if (receivedUSBMsg.cmd_type == CMD_TYPE_SET)
			{
				if (check_received_data_limits(receivedUSBMsg.cmd_data_val1, MTR_SD_MIN, MTR_SD_MAX)) //Ensure motor speed setting is within limits
				{
					if (check_received_data_limits(receivedUSBMsg.cmd_data_val2, MTR_SD_MIN, MTR_SD_MAX)) //Ensure motor steps setting is within limits
					{
						if (check_received_data_limits(receivedUSBMsg.cmd_data_val3, MTR_AD_MIN, MTR_AD_MAX)) //Ensure motor acceleration setting is within limits
						{
							motorSel = convert_received_motor_to_index(receivedUSBMsg.motorSel); //Get motor selection from received msg
							if (check_motor_index(motorSel)) //Ensure motor selection is valid
							{
								if (receivedUSBMsg.motorDir != DIR_UNKNOWN) //Ensure a direction was specified
								{
									float decelTime = 0.0;
									float decelRate = 0.0;
									adjSteps[motorSel] = 0; //Reset adjusted steps
									
									systemMotors[motorSel].specifiedSteps = TRUE; //This command sets a speed with a specified steps, enable specified steps
									set_motor_direction(&systemMotors[motorSel], receivedUSBMsg.motorDir); //Update motor direction
									set_motor_accel_time(&systemMotors[motorSel], receivedUSBMsg.cmd_data_val2); //Deceleration is no longer a unique parameter, will just accelerate to set speed
									set_motor_decel_time(&systemMotors[motorSel], receivedUSBMsg.cmd_data_val3); //Still need to set it though because the deceleration property is used on slowing down
									set_motor_speed(&systemMotors[motorSel], (uint32_t)receivedUSBMsg.cmd_data_val1); //Update motor speed
									
									//Adjust set steps to account for added deceleration phase
									if (systemMotors[motorSel].decelTime != 0)
									{
										decelTime = (systemMotors[motorSel].decelTime / 1000); //Calculate out decel time in seconds
										decelRate = receivedUSBMsg.cmd_data_val1 / decelTime; // a = v / t
										adjSteps[motorSel] = (receivedUSBMsg.cmd_data_val1 * decelTime) - (0.5 * decelRate * decelTime * decelTime); //x = vt - 1/2at^2
									}
									
									set_motor_steps(&systemMotors[motorSel], (uint32_t)receivedUSBMsg.cmd_data_val2); //Update motor steps to move
									start_motor(&systemMotors[motorSel]); //Ensure motor is started
									
									for (uint8_t i = 0; i < strlen(OK_CMD_RESP); i++) //Send back OK cmd response
									{
										USBTxBuff[chrIndex++] = OK_CMD_RESP[i];
									}
									USBTxBuff[chrIndex++] = RETURN_CHAR;
									
									LEDBlinkCount = CMD_MTR_RATE_DUR; //Update blink count to show CMD_MTR_RATE_DUR
									validCmd = TRUE; //Final msg should look like "OK<CR>"
								}
							}
						}
					}
				}
			}
			break;
		case CMD_MTR_LIVE:
			if (receivedUSBMsg.cmd_type == CMD_TYPE_GET)
			{
				//Gets current speed of motor and remaining steps (if applicable)
				char mtrSpeedChar[8] = {};
				char mtrStepsChar[8] = {};
				char mtrAccelChar[8] = {};
				
				motorSel = convert_received_motor_to_index(receivedUSBMsg.motorSel); //Get motor selection from received msg
				if (check_motor_index(motorSel)) //Ensure motor selection is valid
				{
					convert_float_to_string(systemMotors[motorSel].speed, mtrSpeedChar, 8);
					convert_float_to_string(systemMotors[motorSel].accelTime, mtrAccelChar, 8);
					if (systemMotors[motorSel].specifiedSteps)
					{
						convert_int_to_string(systemMotors[motorSel].stepsToMove, mtrStepsChar); //If motor is configured for specified steps, return the steps remaining
					}
					else
					{
						mtrStepsChar[0] = DASH_CHAR; //Motor not configured for specified steps, return -1
						mtrStepsChar[1] = 49; //Ascii 1
					}
					
					for (uint8_t i = 0; i < strlen(MTR_LIVE_RATE_CMD_CHAR); i++) //Add command string to Tx buffer
					{
						if (i != 4)
						{
							USBTxBuff[chrIndex++] = MTR_LIVE_RATE_CMD_CHAR[i];
						}
						else
						{
							USBTxBuff[chrIndex++] = receivedUSBMsg.motorSel; //Motor letter is variable, select this based on motor selection
						}
					}
					USBTxBuff[chrIndex++] = SEPERATOR_CHAR;
					if (systemMotors[motorSel].direction == DIR_CW) //Add motor direction (+/-)
					{
						USBTxBuff[chrIndex++] = 43;
					}
					else
					{
						USBTxBuff[chrIndex++] = 45;
					}
					for (uint8_t i = 0; i < strlen(mtrSpeedChar); i++) //Add motor speed to Tx buffer
					{
						USBTxBuff[chrIndex++] = mtrSpeedChar[i];
					}
					USBTxBuff[chrIndex++] = SEPERATOR_CHAR;
					for (uint8_t i = 0; i < strlen(mtrAccelChar); i++) //Add motor acceleration time to Tx buffer
					{
						USBTxBuff[chrIndex++] = mtrAccelChar[i];
					}
					USBTxBuff[chrIndex++] = SEPERATOR_CHAR;
					for (uint8_t i = 0; i < strlen(mtrStepsChar); i++) //Add motor steps remaining to Tx buffer
					{
						USBTxBuff[chrIndex++] = mtrStepsChar[i];
					}
					USBTxBuff[chrIndex++] = RETURN_CHAR;
					
					LEDBlinkCount = CMD_MTR_LIVE; //Update blink count to show CMD_MTR_LIVE
					validCmd = TRUE; //Final msg should look like "MTR-R +100 100 5000<CR>
				}
			}
			break;
		case CMD_MTR_MS:
			if (receivedUSBMsg.cmd_type == CMD_TYPE_GET)
			{
				char stepResChar[3] = {};
				
				motorSel = convert_received_motor_to_index(receivedUSBMsg.motorSel); //Get motor selection from received msg
				if (check_motor_index(motorSel)) //Ensure motor selection is valid
				{
					convert_int_to_string(systemMotors[motorSel].stepRes, stepResChar);
					
					//Gets selected motors microstepping resolution
					for (uint8_t i = 0; i < strlen(MTR_STEP_RES_CMD_CHAR); i++) //Add command string to Tx buffer
					{
						if (i != 4)
						{
							USBTxBuff[chrIndex++] = MTR_STEP_RES_CMD_CHAR[i];
						}
						else
						{
							USBTxBuff[chrIndex++] = receivedUSBMsg.motorSel; //Motor letter is variable, select this based on motor selection
						}
					}
					USBTxBuff[chrIndex++] = SEPERATOR_CHAR;
					for (uint8_t i = 0; i < strlen(stepResChar); i++) //Add microstep setting to Tx buffer
					{
						USBTxBuff[chrIndex++] = stepResChar[i];
					}
					USBTxBuff[chrIndex++] = RETURN_CHAR;
					
					LEDBlinkCount = CMD_MTR_MS; //Update blink count to show CMD_MTR_MS
					validCmd = TRUE; //Final command should look like "MTR-R MS 16<CR>"
				}
			}
			else if (receivedUSBMsg.cmd_type == CMD_TYPE_SET)
			{
				//Sets selected motors microstepping resolution
				uint8_t validSetting = FALSE;
				
				motorSel = convert_received_motor_to_index(receivedUSBMsg.motorSel); //Get motor selection from received msg
				if (check_motor_index(motorSel)) //Ensure motor selection is valid
				{
					switch ((uint8_t)receivedUSBMsg.cmd_data_val1)
					{
						case STEP_FULL:
							set_motor_step_resolution(&systemMotors[motorSel], STEP_FULL);
							validSetting = TRUE;
							break;
						case STEP_HALF:
							set_motor_step_resolution(&systemMotors[motorSel], STEP_HALF);
							validSetting = TRUE;
							break;
						case STEP_QUARTER:
							set_motor_step_resolution(&systemMotors[motorSel], STEP_QUARTER);
							validSetting = TRUE;
							break;
						case STEP_SIXTEENTH:
							set_motor_step_resolution(&systemMotors[motorSel], STEP_SIXTEENTH);
							validSetting = TRUE;
							break;
						default: //Non-supported setting
							validSetting = FALSE;
							break;
					}
					
					if (validSetting)
					{
						for (uint8_t i = 0; i < strlen(OK_CMD_RESP); i++) //Send back OK cmd response
						{
							USBTxBuff[chrIndex++] = OK_CMD_RESP[i];
						}
						USBTxBuff[chrIndex++] = RETURN_CHAR;
						
						LEDBlinkCount = CMD_MTR_MS; //Update blink count to show CMD_MTR_MS
						validCmd = TRUE; //Final msg should look like "OK<CR>"
					}
				}
			}
			break;
		case CMD_TEMP_IN:
			if (receivedUSBMsg.cmd_type == CMD_TYPE_GET)
			{
				for (uint8_t i = 0; i < strlen(TEMP_IN_CHAR); i++) //Add command string to Tx buffer
				{
					USBTxBuff[chrIndex++] = TEMP_IN_CHAR[i];
				}
				USBTxBuff[chrIndex++] = SEPERATOR_CHAR;
				USBTxBuff[chrIndex++] = tempProbeType + ASCII_OFFSET; //Add probe type to Tx buffer
				USBTxBuff[chrIndex++] = RETURN_CHAR;
				
				LEDBlinkCount = CMD_TEMP_IN; //Update blink count to show CMD_TEMP_IN
				validCmd = TRUE; //Final msg should look like "TEMP IN 1<CR>"
			}
			else if (receivedUSBMsg.cmd_type == CMD_TYPE_SET)
			{
				//Sets current temperature probe type
				if ((receivedUSBMsg.cmd_data_val1 == THERMISTOR) || (receivedUSBMsg.cmd_data_val1 == RTD_PROBE))
				{
					tempProbeType = receivedUSBMsg.cmd_data_val1;
					
					for (uint8_t i = 0; i < strlen(OK_CMD_RESP); i++) //Send back OK cmd response
					{
						USBTxBuff[chrIndex++] = OK_CMD_RESP[i];
					}
					USBTxBuff[chrIndex++] = RETURN_CHAR;
					
					LEDBlinkCount = CMD_TEMP_IN; //Update blink count to show CMD_TEMP_IN
					validCmd = TRUE; //Final msg should look like "OK<CR>"
				}
			}
			break;
		case CMD_PUMP_ON:
			if (receivedUSBMsg.cmd_type == CMD_TYPE_SET)
			{
				if (check_received_data_limits(receivedUSBMsg.cmd_data_val1, PUMP_ON_MIN, PUMP_ON_MAX)) //Ensure pump on time is within limits
				{
					pumpDuration_ms = receivedUSBMsg.cmd_data_val1;
					pumpTime_ms = 0;
					pumpEnabled = TRUE;
					ioport_set_pin_high(PUMP_PWM);
					
					for (uint8_t i = 0; i < strlen(OK_CMD_RESP); i++) //Send back OK cmd response
					{
						USBTxBuff[chrIndex++] = OK_CMD_RESP[i];
					}
					USBTxBuff[chrIndex++] = RETURN_CHAR;
					
					LEDBlinkCount = CMD_PUMP_ON; //Update blink count to show CMD_PUMP_ON
					validCmd = TRUE; //Final msg should look like "OK<CR>"
				}
			}
			break;
		case CMD_PUMP_OFF:
			if (receivedUSBMsg.cmd_type == CMD_TYPE_SET)
			{
				pumpDuration_ms = 0;
				pumpTime_ms = 0;
				pumpEnabled = FALSE;
				ioport_set_pin_low(PUMP_PWM);
				
				for (uint8_t i = 0; i < strlen(OK_CMD_RESP); i++) //Send back OK cmd response
				{
					USBTxBuff[chrIndex++] = OK_CMD_RESP[i];
				}
				USBTxBuff[chrIndex++] = RETURN_CHAR;
				
				LEDBlinkCount = CMD_PUMP_OFF; //Update blink count to show CMD_PUMP_OFF
				validCmd = TRUE; //Final msg should look like "OK<CR>"
			}
			break;
		case CMD_PUMP_STATUS:
			if (receivedUSBMsg.cmd_type == CMD_TYPE_GET)
			{
				char remainingPumpTimeChar[8] = {};
									
				if (pumpDuration_ms >= pumpTime_ms)
				{
					convert_int_to_string((pumpDuration_ms - pumpTime_ms), remainingPumpTimeChar);
				}
				
				for (uint8_t i = 0; i < strlen(PUMP_STATUS_CHAR); i++) //Add command string to Tx buffer
				{
					USBTxBuff[chrIndex++] = PUMP_STATUS_CHAR[i];
				}
				USBTxBuff[chrIndex++] = SEPERATOR_CHAR;
				for (uint8_t i = 0; i < strlen(remainingPumpTimeChar); i++) //Add remaining pump time to Tx buffer
				{
					USBTxBuff[chrIndex++] = remainingPumpTimeChar[i];
				}
				USBTxBuff[chrIndex++] = RETURN_CHAR;
				
				LEDBlinkCount = CMD_PUMP_STATUS; //Update blink count to show CMD_PUMP_STATUS
				validCmd = TRUE; //Final msg should look like "PUMP STATUS 250<CR>"
			}
			break;
		case CMD_FAN_DUTY:
			if (receivedUSBMsg.cmd_type == CMD_TYPE_GET)
			{
				char fanDutyChar[8] = {};
				
				convert_int_to_string(fanDuty, fanDutyChar);
				
				for (uint8_t i = 0; i < strlen(FAN_DUTY_CHAR); i++) //Add command string to Tx buffer
				{
					USBTxBuff[chrIndex++] = FAN_DUTY_CHAR[i];
				}
				USBTxBuff[chrIndex++] = SEPERATOR_CHAR;
				for (uint8_t i = 0; i < strlen(fanDutyChar); i++) //Add fan duty cycle to Tx buffer
				{
					USBTxBuff[chrIndex++] = fanDutyChar[i];
				}
				USBTxBuff[chrIndex++] = RETURN_CHAR;
				
				LEDBlinkCount = CMD_FAN_DUTY; //Update blink count to show CMD_FAN_DUTY
				validCmd = TRUE; //Final msg should look like "FAN DUTY 85<CR>"
			}
			else if (receivedUSBMsg.cmd_type == CMD_TYPE_SET)
			{
				if (check_received_data_limits(receivedUSBMsg.cmd_data_val1, TEC_FAN_DC_MIN, TEC_FAN_DC_MAX)) //Ensure fan duty cycle is within limits
				{
					fanDuty = receivedUSBMsg.cmd_data_val1;
					if (receivedUSBMsg.cmd_data_val1 != 0)
					{
						fanEnabled = TRUE; //Enable fan if set duty cycle is >0
					}
					else
					{
						fanEnabled = FALSE; //Disable fan if commanded to 0 duty cycle
						ioport_set_pin_low(FAN_PWM);
					}
					
					for (uint8_t i = 0; i < strlen(OK_CMD_RESP); i++) //Send back OK cmd response
					{
						USBTxBuff[chrIndex++] = OK_CMD_RESP[i];
					}
					USBTxBuff[chrIndex++] = RETURN_CHAR;
					
					LEDBlinkCount = CMD_FAN_DUTY; //Update blink count to show CMD_FAN_DUTY
					validCmd = TRUE; //Final msg should look like "OK<CR>"
				}
			}
			break;
		case CMD_WDT:
			if (receivedUSBMsg.cmd_type == CMD_TYPE_GET)
			{
				char curWDTSettingChar[5] = {};
				convert_int_to_string(WDTResetCountSet, curWDTSettingChar);
				
				for (uint8_t i = 0; i < strlen(WDT_CHAR); i++) //Add command string to Tx buffer
				{
					USBTxBuff[chrIndex++] = WDT_CHAR[i];
				}
				USBTxBuff[chrIndex++] = SEPERATOR_CHAR;
				for (uint8_t i = 0; i < strlen(curWDTSettingChar); i++) //Add current WDT setting to Tx buffer
				{
					USBTxBuff[chrIndex++] = curWDTSettingChar[i];
				}
				USBTxBuff[chrIndex++] = RETURN_CHAR;
				
				LEDBlinkCount = CMD_WDT; //Update blink count to show CMD_WDT
				validCmd = TRUE; //Final msg should look like "WDT 10<CR>"
			}
			else if (receivedUSBMsg.cmd_type == CMD_TYPE_SET)
			{
				if (check_received_data_limits(receivedUSBMsg.cmd_data_val1, WDT_SET_MIN, WDT_SET_MAX)) //Ensure WDT timeout setting is within limits
				{
					//Received setting is within WDT limits
					if (receivedUSBMsg.cmd_data_val1 == 0)
					{
						disable_WDT(); //Sending a 0 for time means disable WDT
						WDTResetCountSet = 0;
						WDTResetCount = 0;
					}
					else
					{
						set_WDT_timeout(7); //Set WDT period to be 1s
						reset_WDT();
						enable_WDT();
						watchdogTime_ms = 0;
						WDTResetCountSet = receivedUSBMsg.cmd_data_val1;
						WDTResetCount = receivedUSBMsg.cmd_data_val1;
					}
					
					for (uint8_t i = 0; i < strlen(OK_CMD_RESP); i++) //Send back OK response
					{
						USBTxBuff[chrIndex++] = OK_CMD_RESP[i];
					}
					USBTxBuff[chrIndex++] = RETURN_CHAR;
					
					LEDBlinkCount = CMD_WDT; //Update blink count to show CMD_WDT
					validCmd = TRUE; //Final msg should look like "OK<CR>"
				}
			}
			break;
		case CMD_WDT_RESET:
			if (receivedUSBMsg.cmd_type == CMD_TYPE_SET)
			{
				reset_WDT();
				watchdogTime_ms = 0;
				WDTResetCount = WDTResetCountSet; //Since this is handled in software, reset the remaining clear counts
				
				for (uint8_t i = 0; i < strlen(OK_CMD_RESP); i++) //Send back OK response
				{
					USBTxBuff[chrIndex++] = OK_CMD_RESP[i];
				}
				USBTxBuff[chrIndex++] = RETURN_CHAR;
				
				//Don't update LEDBlinkCount here since this message will be sent constantly
				validCmd = TRUE; //Final msg should look like "OK<CR>"
			}
			break;
		case CMD_EEPROM:
			if (receivedUSBMsg.cmd_type == CMD_TYPE_GET)
			{
				uint8_t dataEEProm[USER_INFO_LEN] = {};
				
				read_from_NVM(USER_INFO_ADR, dataEEProm, USER_INFO_LEN); //Read out from EEProm
				
				//Only respond with data (this command doesn't mirror command name)
				for (uint16_t i = 0; i < USER_INFO_LEN; i++)
				{
					USBTxBuff[chrIndex++] = dataEEProm[i];
				}				
				USBTxBuff[chrIndex++] = RETURN_CHAR;
				
				LEDBlinkCount = CMD_EEPROM; //Update blink count to show CMD_EEPROM
				validCmd = TRUE; //Final msg should look like "THIS IS IN EERPOM<CR>"
			}
			else if (receivedUSBMsg.cmd_type == CMD_TYPE_SET)
			{
				if (check_received_data_limits(receivedUSBMsg.cmd_data_val1, EEPROM_MIN, EEPROM_MAX)) //Ensure set data is within limits
				{
					write_to_NVM(USER_INFO_ADR, receivedUSBMsg.cmd_data2, receivedUSBMsg.cmd_data_val1); //Write data to EEProm
				}
				
				for (uint8_t i = 0; i < strlen(OK_CMD_RESP); i++) //Send back OK cmd response
				{
					USBTxBuff[chrIndex++] = OK_CMD_RESP[i];
				}
				USBTxBuff[chrIndex++] = RETURN_CHAR;
				
				LEDBlinkCount = CMD_EEPROM; //Update blink count to show CMD_EEPROM
				validCmd = TRUE; //Final msg should look like "OK<CR>"
			}
			break;
		case CMD_CONST_HEAT:
			if (receivedUSBMsg.cmd_type == CMD_TYPE_GET)
			{
				// Report back current heater output
				char curHeatChar[8] = {};
				uint16_t curHeatOut = tc_read_cc(&TCD1, TC_CCA);
				
				convert_int_to_string((100 - (curHeatOut / 5)), curHeatChar);
				
				for (uint8_t i = 0; i < strlen(CONST_HEAT_CHAR); i++) // Add command string to Tx buffer
				{
					USBTxBuff[chrIndex++] = CONST_HEAT_CHAR[i];
				}
				USBTxBuff[chrIndex++] = SEPERATOR_CHAR;
				for (uint8_t i = 0; i < strlen(curHeatChar); i++) // Add heater output to Tx buffer
				{
					USBTxBuff[chrIndex++] = curHeatChar[i];
				}
				USBTxBuff[chrIndex++] = RETURN_CHAR;
				
				LEDBlinkCount = CMD_CONST_HEAT; // Update blink count to show CMD_CONST_HEAT
				validCmd = TRUE; // Final msg should look like "HEAT 85<CR>"
					
			}
			else if (receivedUSBMsg.cmd_type == CMD_TYPE_SET)
			{				
				if (check_received_data_limits(receivedUSBMsg.cmd_data_val1, CONST_HEAT_MIN, CONST_HEAT_MAX)) // Ensure heater duty cycle is within limits
				{
					tc_write_cc(&TCD1, TC_CCA, (uint16_t)((HEAT_PWM_TC_PERIOD + 1) - (receivedUSBMsg.cmd_data_val1 * 5)));
					
					for (uint8_t i = 0; i < strlen(OK_CMD_RESP); i++) // Send back OK cmd response
					{
						USBTxBuff[chrIndex++] = OK_CMD_RESP[i];
					}
					USBTxBuff[chrIndex++] = RETURN_CHAR;
					
					LEDBlinkCount = CMD_CONST_HEAT; // Update blink count to show CMD_CONST_HEAT
					validCmd = TRUE; // Final msg should look like "OK<CR>"
				}
			}
			break;
		default:
			validCmd = FALSE; //Received command didn't match any known command
			break;
	}
	
	if (validCmd)
	{
		//Received command is valid, send back Tx buffer (will already be loaded with data)
		send_cmd_response(USBTxBuff, chrIndex);
		LEDState = STATUS_BLINK;
	}
	else
	{
		//Unrecognized command, send back error message
		flush_USB_tx_buffer(); //Clear out any data that may have been setup
		chrIndex = 0;
		for (uint8_t i = 0; i < strlen(UNKNOWN_CMD_CHAR); i++) //Add unknown command string to Tx buffer
		{
			USBTxBuff[chrIndex++] = UNKNOWN_CMD_CHAR[i];
		}
		USBTxBuff[chrIndex++] = SEPERATOR_CHAR;
		for (uint8_t i = 0; i < 32; i++) //Send back all non-null USBRxBuff data
		{
			if (USBRxBuff[i] != 0)
			{
				USBTxBuff[chrIndex++] = USBRxBuff[i];
			}
		}
		USBTxBuff[chrIndex++] = RETURN_CHAR;
		
		send_cmd_response(USBTxBuff, chrIndex); //Send back error message
		LEDState = RED_BLINK; //Change LED to show there was an error
	}
	
	flush_USB_rx_buffer(); //Clear out buffer for next rx
}

uint8_t check_received_data_limits(float checkVal, float minVal, float maxVal)
{
	if ((checkVal >= minVal) && (checkVal <= maxVal))
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

//Calculates temperature based on ADC reading from thermistor
double calc_temp_from_ADC_reading(void)
{	
	uint16_t curBuffAvg = 0;
	curBuffAvg = calc_buff_average(thermDMABuffer, (sizeof(thermDMABuffer) / 2));
	
	//Get °C from previoulsy calculated LUT
	if ((curBuffAvg >= THERM_LUT_MIN_INDEX) && (curBuffAvg <= THERM_LUT_MAX_INDEX))
	{
		//ADC reading is within bounds of LUT
		curTemp = thermSensorLUT[curBuffAvg - THERM_LUT_MIN_INDEX]; //Temp value from LUT is in deci?
		curTemp = curTemp / 10;
	}	
	
	return curTemp;
}

//Calculates temperature based on reading from RTD temperature sensor
double calc_temp_from_RTD_reading(uint16_t reading)
{
	//Reading from RTD sensor is a 15-bit ADC value
	//Get °C from previously calculated LUT
	if ((reading >= RTD_LUT_MIN_INDEX) && (reading <= RTD_LUT_MAX_INDEX))
	{
		//ADC reading is within bounds of LUT
		curTemp = RTDSensorLUT[reading - RTD_LUT_MIN_INDEX]; //Temp value from LUT is in milli°C
		curTemp = curTemp / 100;
	}
	
	return curTemp;
}


double calc_sense_cur_from_ADC_reading(uint16_t reading)
{
	//Using the data points in the datasheet, there will need to be a 2-piece equation
	//0-2Vsense (0-800mA) -> y=2.22x+0.222
	//2-4Vsense (1.5A-6A) -> y=0.444x+1.336
	//Between 0.8A - 1.5A unknown, this will have to be tested
	//All these go through a 1k/2.2k divider so the end result will need to be multiplied by (0.3125)
	
	//12-bit ADC @3.3V will result in 806uV/count (3.3V/4096)
	
	//TODO - THIS NEEDS TO BE UPDATED WHEN I HAVE HARDWARE
	
	double tempVal = reading;
	
	if (reading >= 2481)
	{
		//Use lower piece-wise formula
		tempVal = (tempVal * 2222.22) + 222;
	}
	else
	{
		//Use upper piece-wise formula
		tempVal = (tempVal * 444.44) + 1336;
	}
	
	return tempVal;
}

//Clears receivedUSB struct
void clear_receivedUSBMsg(void)
{
	for (uint8_t i = 0; i < sizeof(receivedUSBMsg.cmd_char); i++)
	{
		receivedUSBMsg.cmd_char[i] = 0;
	}
	
	for (uint8_t i = 0; i < sizeof(receivedUSBMsg.cmd_type_char); i++)
	{
		receivedUSBMsg.cmd_type_char[i] = 0;
	}
	
	for (uint8_t i = 0; i < sizeof(receivedUSBMsg.cmd_data1); i++)
	{
		receivedUSBMsg.cmd_data1[i] = 0;
	}
	
	for (uint8_t i = 0; i < sizeof(receivedUSBMsg.cmd_data2); i++)
	{
		receivedUSBMsg.cmd_data2[i] = 0;
	}
	
	for (uint8_t i = 0; i < sizeof(receivedUSBMsg.cmd_data3); i++)
	{
		receivedUSBMsg.cmd_data3[i] = 0;
	}
}

//Clears USB rx buffer
void flush_USB_rx_buffer(void)
{
	for (uint8_t i = 0; i < sizeof(USBRxBuff); i++)
	{
		USBRxBuff[i] = 0;
	}
}

//Clears USB tx buffer
void flush_USB_tx_buffer(void)
{
	for (uint8_t i = 0; i < sizeof(USBTxBuff); i++)
	{
		USBTxBuff[i] = 0;
	}
}

//Controls state of status LED (active low logic)
void handle_LEDs(void)
{
	switch (LEDState)
	{
		default:
		case OFF:
			ioport_set_pin_high(_STATUS_RED_LED);
			ioport_set_pin_high(_STATUS_GREEN_LED);
			break;
		case GREEN:
			ioport_set_pin_high(_STATUS_RED_LED);
			ioport_set_pin_low(_STATUS_GREEN_LED);
			break;
		case RED:
			ioport_set_pin_low(_STATUS_RED_LED);
			ioport_set_pin_high(_STATUS_GREEN_LED);
			break;
		case ORANGE:
			ioport_set_pin_low(_STATUS_RED_LED);
			ioport_set_pin_low(_STATUS_GREEN_LED);
			break;
		case GREEN_BLINK:		
			ioport_set_pin_high(_STATUS_RED_LED);
			if (ioport_get_pin_level(_STATUS_GREEN_LED))
			{
				//Green LED is off
				if (blinkTimeLED_ms >= LEDOffTime)
				{
					reset_blink_time_LED();
					ioport_set_pin_low(_STATUS_GREEN_LED);
				}
			}
			else
			{
				//Green LED is on
				if (blinkTimeLED_ms >= LEDOnTime)
				{
					reset_blink_time_LED();
					ioport_set_pin_high(_STATUS_GREEN_LED);
				}
			}
			break;
		case RED_BLINK:
			ioport_set_pin_high(_STATUS_GREEN_LED);
			if (ioport_get_pin_level(_STATUS_RED_LED))
			{
				//Red LED is off
				if (blinkTimeLED_ms >= LEDOffTime)
				{
					reset_blink_time_LED();
					ioport_set_pin_low(_STATUS_RED_LED);
				}
			}
			else
			{
				//Red LED is on
				if (blinkTimeLED_ms >= LEDOnTime)
				{
					reset_blink_time_LED();
					ioport_set_pin_high(_STATUS_RED_LED);
				}
			}
			break;
		case ORANGE_BLINK:
			if (ioport_get_pin_level(_STATUS_RED_LED))
			{
				//Red LED is off
				if (blinkTimeLED_ms >= LEDOffTime)
				{
					reset_blink_time_LED();
					ioport_set_pin_low(_STATUS_RED_LED);
					ioport_set_pin_low(_STATUS_GREEN_LED);
				}
			}
			else
			{
				//Red LED is on
				if (blinkTimeLED_ms >= LEDOnTime)
				{
					reset_blink_time_LED();
					ioport_set_pin_high(_STATUS_RED_LED);
					ioport_set_pin_high(_STATUS_GREEN_LED);
				}
			}
			break;
		case STATUS_BLINK:
			ioport_set_pin_high(_STATUS_RED_LED);
		
			if (LEDBlinkCount != curLEDBlinkCount)
			{
				if (ioport_get_pin_level(_STATUS_GREEN_LED))
				{
					//LED is off
					if (blinkTimeLED_ms >= LEDOffTime)
					{
						reset_blink_time_LED();
						ioport_set_pin_low(_STATUS_GREEN_LED);
					}
				}
				else
				{
					//LED is on
					if (blinkTimeLED_ms >= LEDOnTime)
					{
						reset_blink_time_LED();
						ioport_set_pin_high(_STATUS_GREEN_LED);
						curLEDBlinkCount++;
					}
				}
			}
			else
			{
				if (blinkTimeLED_ms >= STATUS_BLINK_DWELL_TIME)
				{
					reset_blink_time_LED();
					ioport_set_pin_low(_STATUS_GREEN_LED);
					curLEDBlinkCount = 0;
				}
			}
			break;
	}
}

//Reads current temperature, sends to PID loop, and controls heater
void handle_temp_PID(void)
{
	if (heaterEnabled)
	{
		if (tempUpdate_ms >= TEMP_UPDATE_TIME)
		{
			if (tempProbeType == THERMISTOR)
			{
				// Get current temperature reading from probe with 80/20 weighted average
				//curTemp = ((0.8 * curTemp) + (0.2 * calc_temp_from_ADC_reading()));
				curTemp = calc_temp_from_ADC_reading();
			}
			else
			{
				// Get current temperature reading from probe with 80/20 weighted average
				//curTemp = ((0.8 * curTemp) + (0.2 * calc_temp_from_RTD_reading(get_RTD_sensor_reading())));
				curTemp = calc_temp_from_RTD_reading(get_RTD_sensor_reading());
			}
		}
		
		if (tempPIDUpdate_ms >= PID_UPDATE_TIME)
		{			
			if (tempProbeType == THERMISTOR)
			{
				compute_temp_PID_duty(&heaterPID, (tempSetPoint - curTemp), (float)PID_UPDATE_TIME / 1000);
				tc_write_cc(&TCD1, TC_CCA, (uint16_t)((HEAT_PWM_TC_PERIOD + 1) - (heaterPID.PIDVal * 5)));
			}
			else
			{
				compute_temp_PID_duty(&heaterPID, (tempSetPoint - curTemp), (float)PID_UPDATE_TIME / 1000);
				tc_write_cc(&TCD1, TC_CCA, (uint16_t)((HEAT_PWM_TC_PERIOD + 1) - (heaterPID.PIDVal * 5)));
			}
			
			tempPIDUpdate_ms = 0;
		}
	}
}

//Discrete control of pump
void handle_pump(void)
{
	if (pumpEnabled)
	{
		if (pumpDuration_ms != 0) //0 set for duration is always on
		{
			if (pumpTime_ms >= pumpDuration_ms)
			{
				ioport_set_pin_low(PUMP_PWM); //This pin is used for pump step now
			}
		}
	}
}

//TEC Fan
void handle_fan(void)
{
	if (fanEnabled)
	{
		tc_write_cc(&FAN_PWM_TC, FAN_PWM_CCCHAN, (uint16_t)((FAN_PWM_TC_PERIOD + 1) - (fanDuty * 5)));
	}
}

//Calculate heater DC offset, determined empirically with the current setup
void calc_heater_adj(void)
{
	heaterDCOffset = ((0.0035 * tempSetPoint * tempSetPoint) + (0.0383 * tempSetPoint) + 5.679);
	//0.0027, 0.0383, 5.679
}

//Handles gradual transition of TEC
void handle_TEC(void)
{
	if (TECTime_ms >= TEC_UPDATE_TIME)
	{		
		switch (TECState)
		{
			case TEC_IDLE:
				//Do nothing here
				break;
			case TEC_OFF:
				//Transition to 0 for both A/B
				if (curTECPWMDutyA != 0)
				{
					curTECPWMDutyA--;
					set_TEC_pwm_A_duty(curTECPWMDutyA);
					//if (curTECPWMDutyA != TECPWMDutyA)
					//{
						////Setpoint is positive
						//TECState = TEC_POS;
					//}
					if (curTECPWMDutyA == 0)
					{
						if (curTECPWMDutyB != TECPWMDutyB)
						{
							TECState = TEC_NEG;
						}
						else
						{
							TECState = TEC_IDLE;
						}
					}
				}
				
				if (curTECPWMDutyB != 0)
				{
					curTECPWMDutyB--;
					set_TEC_pwm_B_duty(curTECPWMDutyB);
					//if (curTECPWMDutyB != TECPWMDutyB)
					//{
						////Setpoint is negative
						//TECState = TEC_NEG;
					//}
					if (curTECPWMDutyB == 0)
					{
						if (curTECPWMDutyA != TECPWMDutyA)
						{
							TECState = TEC_POS;
						}
						else
						{
							TECState = TEC_IDLE;
						}
					}
				}
				break;
			case TEC_POS: //Transition to +
				if (curTECPWMDutyA != TECPWMDutyA)
				{
					if (curTECPWMDutyA < TECPWMDutyA)
					{
						curTECPWMDutyA++;
					}
					else if (curTECPWMDutyA > TECPWMDutyA)
					{
						curTECPWMDutyA--;
					}
					set_TEC_pwm_A_duty(curTECPWMDutyA);
				}
				else
				{
					TECState = TEC_IDLE;
				}
				break;
			case TEC_NEG: //Transition to -
				if (curTECPWMDutyB != TECPWMDutyB)
				{
					if (curTECPWMDutyB < TECPWMDutyB)
					{
						curTECPWMDutyB++;
					}
					else if (curTECPWMDutyB > TECPWMDutyB)
					{
						curTECPWMDutyB--;
					}
					set_TEC_pwm_B_duty(curTECPWMDutyB);
				}
				else
				{
					TECState = TEC_IDLE;
				}
				break;
			default:
				TECState = TEC_IDLE;
				break;
		}
		
		TECTime_ms = 0;
	}
}

void reset_blink_time_LED(void)
{
	blinkTimeLED_ms = 0;
}

void clear_pump_heat_fault(void)
{
	//ioport_set_pin_low(_PUMP_HEAT_FR_STDBY);
	//delay_ms(50);
	//ioport_set_pin_high(_PUMP_HEAT_FR_STDBY);
}

double calc_buff_average(uint16_t *bufVal, uint16_t bufLen)
{
	double avgVal = 0;
	for (uint16_t i = 0; i < bufLen; i++)
	{
		avgVal += *bufVal;
		bufVal++;
	}
	avgVal = avgVal / bufLen;
	return avgVal;
}

void init_DMA(void)
{	
	//DMA setup for fan current sense
	struct dma_channel_config fanSenseDMAConfig;
	memset(&fanSenseDMAConfig, 0, sizeof(fanSenseDMAConfig));
	dma_channel_set_burst_length(&fanSenseDMAConfig, DMA_CH_BURSTLEN_2BYTE_gc);
	dma_channel_set_transfer_count(&fanSenseDMAConfig, FAN_SENSE_DMA_BUFF_SIZE);
	dma_channel_set_src_reload_mode(&fanSenseDMAConfig, DMA_CH_SRCRELOAD_BURST_gc);
	dma_channel_set_dest_reload_mode(&fanSenseDMAConfig, DMA_CH_DESTRELOAD_TRANSACTION_gc);
	dma_channel_set_src_dir_mode(&fanSenseDMAConfig, DMA_CH_SRCDIR_INC_gc);
	dma_channel_set_dest_dir_mode(&fanSenseDMAConfig, DMA_CH_DESTDIR_INC_gc);
	dma_channel_set_source_address(&fanSenseDMAConfig, (uint16_t)(uintptr_t)ADCA.CH0RES); //Get data from ADC CH0
	dma_channel_set_destination_address(&fanSenseDMAConfig, (uint16_t)(uintptr_t)fanSenseDMABuffer);
	dma_channel_set_trigger_source(&fanSenseDMAConfig, DMA_CH_TRIGSRC_ADCA_CH0_gc);
	dma_channel_set_repeats(&fanSenseDMAConfig, 0); //Setting repeats to 0 equals infinite repeats
	dma_enable(); //Resets entire DMA peripheral (only call once)
	dma_channel_write_config(FAN_SENSE_DMA_CHANNEL, &fanSenseDMAConfig);
	dma_channel_enable(FAN_SENSE_DMA_CHANNEL);
	
	//DMA setup for thermistor
	struct dma_channel_config thermDMAConfig;
	memset(&thermDMAConfig, 0, sizeof(thermDMAConfig));
	dma_channel_set_burst_length(&thermDMAConfig, DMA_CH_BURSTLEN_2BYTE_gc);
	dma_channel_set_transfer_count(&thermDMAConfig, THERM_DMA_BUFF_SIZE);
	dma_channel_set_src_reload_mode(&thermDMAConfig, DMA_CH_SRCRELOAD_BURST_gc);
	dma_channel_set_dest_reload_mode(&thermDMAConfig, DMA_CH_DESTRELOAD_TRANSACTION_gc);
	dma_channel_set_src_dir_mode(&thermDMAConfig, DMA_CH_SRCDIR_INC_gc);
	dma_channel_set_dest_dir_mode(&thermDMAConfig, DMA_CH_DESTDIR_INC_gc);
	dma_channel_set_source_address(&thermDMAConfig, (uint16_t)(uintptr_t)&ADCA.CH1RES); //Get data from ADC CH1
	dma_channel_set_destination_address(&thermDMAConfig, (uint16_t)(uintptr_t)thermDMABuffer);
	dma_channel_set_trigger_source(&thermDMAConfig, DMA_CH_TRIGSRC_ADCA_CH1_gc);
	dma_channel_set_repeats(&thermDMAConfig, 0); //Setting repeats to 0 equals infinite repeats
	dma_channel_write_config(THERM_DMA_CHANNEL, &thermDMAConfig);
	dma_channel_enable(THERM_DMA_CHANNEL);
	
	//DMA setup for pump current sense
	struct dma_channel_config pumpSenseDMAConfig;
	memset(&pumpSenseDMAConfig, 0, sizeof(pumpSenseDMAConfig));
	dma_channel_set_burst_length(&pumpSenseDMAConfig, DMA_CH_BURSTLEN_2BYTE_gc);
	dma_channel_set_transfer_count(&pumpSenseDMAConfig, PUMP_SENSE_DMA_BUFF_SIZE);
	dma_channel_set_src_reload_mode(&pumpSenseDMAConfig, DMA_CH_SRCRELOAD_BURST_gc);
	dma_channel_set_dest_reload_mode(&pumpSenseDMAConfig, DMA_CH_DESTRELOAD_TRANSACTION_gc);
	dma_channel_set_src_dir_mode(&pumpSenseDMAConfig, DMA_CH_SRCDIR_INC_gc);
	dma_channel_set_dest_dir_mode(&pumpSenseDMAConfig, DMA_CH_DESTDIR_INC_gc);
	dma_channel_set_source_address(&pumpSenseDMAConfig, (uint16_t)(uintptr_t)&ADCA.CH2RES); //Get data from ADC CH2
	dma_channel_set_destination_address(&pumpSenseDMAConfig, (uint16_t)(uintptr_t)pumpSenseDMABuffer);
	dma_channel_set_trigger_source(&pumpSenseDMAConfig, DMA_CH_TRIGSRC_ADCA_CH2_gc);
	dma_channel_set_repeats(&pumpSenseDMAConfig, 0); //Setting repeats to 0 equals infinite repeats
	dma_channel_write_config(PUMP_SENSE_DMA_CHANNEL, &pumpSenseDMAConfig);
	dma_channel_enable(PUMP_SENSE_DMA_CHANNEL);
	
	//DMA setup for heater current sense //Enabling this DMA causes board to reset
	//struct dma_channel_config heaterSenseDMAConfig;
	//memset(&heaterSenseDMAConfig, 0, sizeof(heaterSenseDMAConfig));
	//dma_channel_set_burst_length(&heaterSenseDMAConfig, DMA_CH_BURSTLEN_2BYTE_gc);
	//dma_channel_set_transfer_count(&heaterSenseDMAConfig, HEATER_SENSE_DMA_BUFF_SIZE);
	//dma_channel_set_src_reload_mode(&heaterSenseDMAConfig, DMA_CH_SRCRELOAD_BURST_gc);
	//dma_channel_set_dest_reload_mode(&heaterSenseDMAConfig, DMA_CH_DESTRELOAD_TRANSACTION_gc);
	//dma_channel_set_src_dir_mode(&heaterSenseDMAConfig, DMA_CH_SRCDIR_INC_gc);
	//dma_channel_set_dest_dir_mode(&heaterSenseDMAConfig, DMA_CH_DESTDIR_INC_gc);
	//dma_channel_set_source_address(&heaterSenseDMAConfig, (uint16_t)(uintptr_t)&ADCA.CH3RES); //Get data from ADC CH3
	//dma_channel_set_trigger_source(&heaterSenseDMAConfig, (uint16_t)(uintptr_t)heaterSenseDMABuffer);
	//dma_channel_set_trigger_source(&heaterSenseDMAConfig, DMA_CH_TRIGSRC_ADCA_CH3_gc);
	//dma_channel_set_repeats(&heaterSenseDMAConfig, 0); //Setting repeats to 0 equals infinite repeats
	//dma_channel_write_config(HEATER_SENSE_DMA_CHANNEL, &heaterSenseDMAConfig);
	//dma_channel_enable(HEATER_SENSE_DMA_CHANNEL);
}