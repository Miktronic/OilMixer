/*
 * StepperMotor.c
 *
 * Created: 8/12/2019 3:01:44 PM
 *  Author: adam
 */ 

#include <avr/io.h>
#include "ioport.h"

#include "StepperMotor.h"
#include "SPI_Wrapper.h"
#include "RealMain.h"


//NOTE: From Ayalytical:
//30,700ppm will move the mixer at 1000RPM
//Gear ratio = 0.1535

uint8_t dataBufSPI[8];

//Configures stepper driver for specified step resolution
void set_motor_step_resolution(motorConfig *motor, motorStepRes stepRes)
{
	motor->stepRes = stepRes;
	switch (motor->stepRes)
	{
		default: //Default to full steps
		case STEP_FULL:
			ioport_set_pin_low(motor->controlPins.ms0);
			ioport_set_pin_low(motor->controlPins.ms1);
			break;
		case STEP_HALF:
			ioport_set_pin_high(motor->controlPins.ms0);
			ioport_set_pin_low(motor->controlPins.ms1);
			break;
		case STEP_QUARTER:
			ioport_set_pin_low(motor->controlPins.ms0);
			ioport_set_pin_high(motor->controlPins.ms1);
			break;
		case STEP_SIXTEENTH:
			ioport_set_pin_high(motor->controlPins.ms0);
			ioport_set_pin_high(motor->controlPins.ms1);
			break;
	}
}

//Configures stepper driver PWM setting for specified speed
void set_motor_speed(motorConfig *motor, uint32_t speedPPS)
{
	//Received command from control PC is in pulses per sec (can set PWM freq directly)
	if (motor->accelTime != 0)
	{
		//Calculate acceleration rate to setpoint speed from current speed
		motor->accelRate = ((float)((float)speedPPS - motor->speed) / motor->accelTime);
		motor->PWMEndFreq = speedPPS;
	}
	else
	{
		motor->accelRate = 0;
		motor->PWMEndFreq = 0;
		if (speedPPS != 0)
		{
			motor->speed = speedPPS;
		}
	}
	
	motor->PWMFreq = ((double)motor->speed);
}

//Sets the amount of steps to move when specified
void set_motor_steps(motorConfig *motor, uint32_t steps)
{
	motor->stepsToMove = steps;
}

//Configures stepper driver for specified direction
void set_motor_direction(motorConfig *motor, uint8_t direction)
{
	motor->direction = direction;
	if (motor->direction == DIR_CW)
	{
		ioport_set_pin_high(motor->controlPins.dir);
	}
	else
	{
		ioport_set_pin_low(motor->controlPins.dir);
	}
}

//Sets motor acceleration value
void set_motor_accel_time(motorConfig *motor, uint32_t accelTime)
{
	motor->accelTime = accelTime;
	motor->accelRate = 0;
	set_motor_speed(motor, motor->speed); //Call set_motor_speed to update acceleration rate
}

//Sets motor deceleration value
void set_motor_decel_time(motorConfig *motor, uint32_t decelTime)
{
	motor->decelTime = decelTime; //Deceleration rate will be calculated on calling stop_motor
	motor->decelRate = 0;
}

//Individual motor stepping
void step_motor(motorConfig *motor)
{
	ioport_set_pin_high(motor->controlPins.en);
	ioport_set_pin_high(motor->controlPins.step);
	motor->stepsToMove--;
	ioport_set_pin_low(motor->controlPins.step);
}

//Handles motor acceleration. This will be called every ACCEL_TIME from main loop
void handle_accel(motorConfig *motor)
{
	if (motor->accelTime != 0)
	{
		if (motor->PWMEndFreq != motor->PWMFreq)
		{
			motor->PWMFreq += (motor->accelRate * ACCEL_TIME);
			motor->speed = motor->PWMFreq;
			if ((motor->PWMFreq > motor->PWMEndFreq) && (motor->accelRate >= 0))
			{
				motor->PWMFreq = motor->PWMEndFreq;
			}
		}
		
		pwm_set_frequency(&motor->PWMConfig, (uint16_t)motor->PWMFreq);
		pwm_start(&motor->PWMConfig, motor->PWMDuty);
		motor->accelTime -= ACCEL_TIME;
	}
	else
	{
		motor->accelRate = 0;
	}
}

//Handles motor deceleration. This will be called every DECEL_TIME from main loop
void handle_decel(motorConfig *motor)
{
	if (motor->decelTime != 0)
	{
		if (motor->PWMFreq != 0)
		{
			motor->PWMFreq -= (motor->decelRate * DECEL_TIME);
			motor->speed = motor->PWMFreq;
			if (motor->PWMFreq < 0)
			{
				motor->PWMFreq = 0;
			}
		}
		
		pwm_set_frequency(&motor->PWMConfig, (uint16_t)motor->PWMFreq);
		pwm_start(&motor->PWMConfig, motor->PWMDuty);
		motor->decelTime -= DECEL_TIME;
	}
	else
	{
		motor->decelRate = 0;
		stop_motor(motor);
	}
}

//Initializes TC's used for controlling PWM at 0Hz
void init_motor_timers(motorConfig *motor)
{
	pwm_init(&motor->PWMConfig, motor->PWMTC, motor->PWMChan, 0);	
}

//Initializes uC pins for stepper motor discrete control
void init_stepper_motor_pins(void)
{	
	//Initialize all rod position control pins		
	ioport_configure_pin(ROD_POS_DIR, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH); //Default to CW rotation
	ioport_configure_pin(ROD_POS_STEP, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(_ROD_POS_RST, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH); //Needs to be high for operation
	ioport_configure_pin(ROD_POS_MS1, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW); //Default to full step resolution
	ioport_configure_pin(ROD_POS_MS0, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(ROD_POS_EN, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW); //Default to disabled
	ioport_configure_pin(ROD_POS_DIAG, IOPORT_DIR_INPUT);
	
	//Initialize all mixer control pins
	ioport_configure_pin(MIXER_DIR, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH); //Default to CW rotation
	ioport_configure_pin(MIXER_STEP, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(_MIXER_RST, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH); //Needs to be high for operation
	ioport_configure_pin(MIXER_MS1, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW); //Default to full step resolution
	ioport_configure_pin(MIXER_MS0, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(MIXER_EN, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW); //Default to disabled
	ioport_configure_pin(MIXER_DIAG, IOPORT_DIR_INPUT);
	
	//Initialize all lower control pins
	ioport_configure_pin(LOWER_MIXER_DIR, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH); //Default to CW rotation
	ioport_configure_pin(LOWER_MIXER_STEP, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(_LOWER_MIXER_RST, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH); //Needs to be high for operation
	ioport_configure_pin(LOWER_MIXER_MS1, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW); //Default to full step resolution
	ioport_configure_pin(LOWER_MIXER_MS0, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(LOWER_MIXER_EN, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW); //Default to disabled
	ioport_configure_pin(LOWER_MIXER_DIAG, IOPORT_DIR_INPUT);
	
	//Initialize all pump control pins
	ioport_configure_pin(PUMP_DIR, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH); //Default to CW rotation
	ioport_configure_pin(PUMP_STEP, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(_PUMP_RST, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH); //Needs to be high for operation
	ioport_configure_pin(PUMP_MS1, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW); //Default to full step resolution
	ioport_configure_pin(PUMP_MS0, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	
	//This pin is now used for mixer step
	//ioport_configure_pin(PUMP_EN, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW); //Default to disabled
	ioport_configure_pin(PUMP_DIAG, IOPORT_DIR_INPUT);
}

//Gets current holding current of motor in percent of Ismax
uint8_t get_motor_hold_current_percent(motorConfig *motor)
{
	uint8_t holdCurSetting = 0;
	uint8_t MXIVal = 0;
	
	MXIVal = ((motor->config0Reg & 0x0600) >> 9);
	
	switch (MXIVal)
	{
		case 0:
			holdCurSetting = 25;
			break;
		case 1:
			holdCurSetting = 50;
			break;
		case 2:
			holdCurSetting = 75;
			break;
		case 3:
			holdCurSetting = 100;
			break;
		default:
			holdCurSetting = 0;
			break;
	}
	
	if (!motor->enabled)
	{
		holdCurSetting = 0; //Motor disabled means a holding current of 0
	}
	
	return holdCurSetting;
}

//Sets the holding current of the motor
uint8_t set_motor_hold_current(motorConfig *motor, uint8_t holdCur)
{
	uint8_t validSetting = TRUE;
	uint8_t TxBuff[8];
	
	switch (holdCur)
	{
		case 0:
			//Holding current of 0% -> disable motor driver
			disable_motor(motor);
			break;
		case 25:
			//Holding current of 25% of Ismax
			CLR_BIT(motor->config0Reg, CFG0_MXI0);
			CLR_BIT(motor->config0Reg, CFG0_MXI1);
			break;
		case 50:
			//Holding current of 50% of Ismax
			SET_BIT(motor->config0Reg, CFG0_MXI0);
			CLR_BIT(motor->config0Reg, CFG0_MXI1);
			break;
		case 75:
			//Holding current of 75% of Ismax
			CLR_BIT(motor->config0Reg, CFG0_MXI0);
			SET_BIT(motor->config0Reg, CFG0_MXI1);
			break;
		case 100:
			//Holding current of 100% of Ismax (default at startup)
			SET_BIT(motor->config0Reg, CFG0_MXI0);
			SET_BIT(motor->config0Reg, CFG0_MXI1);
			break;
		default:
			//Unknown setting, don't change current setting
			validSetting = FALSE;
			break;
	}
	
	if (validSetting && (holdCur != 0))
	{
		enable_motor(motor); //Enable motor driver chip if valid
		
		//Send updated register info to motor driver
		TxBuff[0] = (motor->config0Reg >> 8);
		TxBuff[1] = motor->config0Reg & 0x00FF;
		send_msg(&STEPPER_SPI_PORT, motor->controlPins._cs, TxBuff, 2);
	}
	
	return validSetting;
}

//Enables stepper driver with current motor configuration, starts pwm's if applicable
void start_motor(motorConfig *motor)
{
	motor->enabled = TRUE;
	if ((motor->motorID == ROD_POS) && (!ioport_get_pin_level(ROD_POS_SWITCH)) && (motor->direction == DIR_CCW))
	{
		//Don't start rod position stepper if the position switch is hit and it is moving up
	}
	else
	{
		ioport_set_pin_high(motor->controlPins.en);
		if ((motor->PWMFreq == 0) && (motor->PWMEndFreq == 0))
		{
			stop_motor(motor);
		}
		else
		{
			//Can set using HAL directly
			pwm_set_frequency_32bit(&motor->PWMConfig, motor->PWMFreq);
			pwm_start(&motor->PWMConfig, motor->PWMDuty);
		}
	}
}

//Disables pwm output, motor still remains enabled
void stop_motor(motorConfig *motor)
{
	if (motor->decelTime != 0)
	{
		//Calculate deceleration rate based on current speed
		motor->decelRate = (double)motor->PWMFreq / motor->decelTime;
	}
	else
	{
		motor->enabled = FALSE;
		motor->speed = 0;
		pwm_set_duty_cycle_percent(&motor->PWMConfig, 0);
		pwm_set_frequency(&motor->PWMConfig, 0);
		ioport_set_pin_low(motor->controlPins.step);
	}
}

//Enables motor driver
void enable_motor(motorConfig *motor)
{
	motor->enabled = TRUE;
	ioport_set_pin_high(motor->controlPins.en);
}

//Fully disables motor
void disable_motor(motorConfig *motor)
{
	motor->enabled = FALSE;
	ioport_set_pin_low(motor->controlPins.en);
}

//Sends reset home position to signal to pwm driver chip
void reset_faults(motorConfig *motor)
{
	//Pulses of more than reset shutdown width will force sleep mode (10us)	
	switch (motor->motorID)
	{
		case ROD_POS:
			ioport_set_pin_low(_ROD_POS_RST);
			ioport_set_pin_high(_ROD_POS_RST);
			break;
		case MIXER:
			ioport_set_pin_low(_MIXER_RST);
			ioport_set_pin_high(_MIXER_RST);
			break;
		case LOWER_MIXER:
			ioport_set_pin_low(_LOWER_MIXER_RST);
			ioport_set_pin_high(_LOWER_MIXER_RST);
			break;
		case PUMP:
			ioport_set_pin_low(_PUMP_RST);
			ioport_set_pin_high(_PUMP_RST);
			break;
	}
}

//Resets motorConfig struct to initial state
void reset_motor_config(motorConfig *motor)
{
	motor->controlType = DISCRETE_CONTROL;
	motor->direction = DIR_CW;
	motor->enabled = FALSE;
	motor->speed = 0;
	motor->accelTime = 0;
	motor->accelRate = 0;
	motor->stepsToMove = 0;
	motor->stepsToMove = 0;
	motor->stepRes = STEP_FULL;
	motor->PWMFreq = 0;
	motor->PWMEndFreq = 0;
	motor->PWMDuty = 50;
}

//Same as pwm_set_frequency in HAL but with uint32_t input
void pwm_set_frequency_32bit(struct pwm_config *config, uint32_t freq_hz)
{
	uint32_t cpu_hz = sysclk_get_cpu_hz();
	uint16_t smallest_div;
	uint16_t dividor;

	/* Avoid division by zero. */
	Assert(freq_hz != 0);

	/* Calculate the smallest divider for the requested frequency
	   related to the CPU frequency */
	smallest_div = cpu_hz / freq_hz / 0xFFFF;
	if (smallest_div < 1) {
		dividor = 1;
		config->clk_sel = PWM_CLK_DIV1;
	} else if (smallest_div < 2) {
		dividor = 2;
		config->clk_sel = PWM_CLK_DIV2;
	} else if (smallest_div < 4) {
		dividor = 4;
		config->clk_sel = PWM_CLK_DIV4;
	} else if (smallest_div < 8) {
		dividor = 8;
		config->clk_sel = PWM_CLK_DIV8;
	} else if (smallest_div < 64) {
		dividor = 64;
		config->clk_sel = PWM_CLK_DIV64;
	} else if (smallest_div < 256) {
		dividor = 256;
		config->clk_sel = PWM_CLK_DIV256;
	} else {
		dividor = 1024;
		config->clk_sel = PWM_CLK_DIV1024;
	}

	/* Calculate the period from the just found divider */
	config->period = cpu_hz / dividor / freq_hz;

	/* Make sure our period is at least 100 ticks so we are able to provide
	   a full range (0-100% duty cycle */
	if (config->period < 100) {
		/* The period is too short. */
		config->clk_sel = PWM_CLK_OFF;
		config->period = 0;
		Assert(false);
	}
}
