/*
 * StepperMotor.h
 *
 * Created: 8/12/2019 3:02:10 PM
 *  Author: adam
 */ 

#include <stdint.h>
#include "pwm.h"

#ifndef STEPPERMOTOR_H_
#define STEPPERMOTOR_H_

#define DIR_CW		0
#define DIR_CCW		1
#define DIR_UNKNOWN 2

#define STEPS_PER_REV	200		//# of steps per full revolution (at full step)
#define GEAR_RATIO		0.1535
#define ACCEL_TIME	10
#define DECEL_TIME	10
#define STEP_TIME	100

//////////////////////////////////////////////////////////////////////////
// Pin definitions
//////////////////////////////////////////////////////////////////////////
#define ROD_POS_DIR			IOPORT_CREATE_PIN(PORTH, 2) //Rod position step control pins
//#define ROD_POS_STEP		IOPORT_CREATE_PIN(PORTD, 1) //Moved to PE4
#define ROD_POS_STEP		IOPORT_CREATE_PIN(PORTE, 4)
#define _ROD_POS_RST		IOPORT_CREATE_PIN(PORTH, 5)
#define ROD_POS_MS1			IOPORT_CREATE_PIN(PORTH, 1)
#define ROD_POS_MS0			IOPORT_CREATE_PIN(PORTH, 0)
#define ROD_POS_EN			IOPORT_CREATE_PIN(PORTH, 3)
#define ROD_POS_DIAG		IOPORT_CREATE_PIN(PORTH, 7)

#define MIXER_DIR			IOPORT_CREATE_PIN(PORTJ, 2) //Mixer step control pins
//#define MIXER_STEP			IOPORT_CREATE_PIN(PORTD, 2) //Moved to PF3
#define MIXER_STEP			IOPORT_CREATE_PIN(PORTF, 3)
#define _MIXER_RST			IOPORT_CREATE_PIN(PORTJ, 5)
#define MIXER_MS1			IOPORT_CREATE_PIN(PORTJ, 1)
#define MIXER_MS0			IOPORT_CREATE_PIN(PORTJ, 0)
#define MIXER_EN			IOPORT_CREATE_PIN(PORTJ, 3)
#define MIXER_DIAG			IOPORT_CREATE_PIN(PORTJ, 7)

//#define LOWER_MIXER_DIR		IOPORT_CREATE_PIN(PORTK, 2) //Lower mixer step control pins
//#define LOWER_MIXER_STEP	IOPORT_CREATE_PIN(PORTD, 3) //Moved to pin D1 only because D3 wasn't operational
//#define LOWER_MIXER_STEP	IOPORT_CREATE_PIN(PORTD, 1)
//#define _LOWER_MIXER_RST	IOPORT_CREATE_PIN(PORTK, 5)
//#define LOWER_MIXER_MS1		IOPORT_CREATE_PIN(PORTK, 1)
//#define LOWER_MIXER_MS0		IOPORT_CREATE_PIN(PORTK, 0)
//#define LOWER_MIXER_EN		IOPORT_CREATE_PIN(PORTK, 3)
//#define LOWER_MIXER_DIAG	IOPORT_CREATE_PIN(PORTK, 7)

//#define PUMP_DIR			IOPORT_CREATE_PIN(PORTF, 2) //Pump step control pins (if using stepper motor control)
//#define PUMP_STEP			IOPORT_CREATE_PIN(PORTD, 0) //Moved to PC0
//#define PUMP_STEP			IOPORT_CREATE_PIN(PORTC, 0)
//#define _PUMP_RST			IOPORT_CREATE_PIN(PORTF, 5)
//#define PUMP_MS1			IOPORT_CREATE_PIN(PORTF, 1)
//#define PUMP_MS0			IOPORT_CREATE_PIN(PORTF, 0)
//#define PUMP_EN				IOPORT_CREATE_PIN(PORTF, 3) //This pin is now used for mixer step
//#define PUMP_DIAG			IOPORT_CREATE_PIN(PORTF, 7)

typedef enum motorStepRes
{
	STEP_FULL = 1,
	STEP_HALF = 2,
	STEP_QUARTER = 4,
	STEP_SIXTEENTH = 16
} motorStepRes;

enum CONFIG0_REG
{
	CFG0_PWM =		0,	//PWM config setting
	CFG0_TFFRQ0 =	1,	//Off time/Freq setting
	CFG0_TFFRQ1 =	2,
	CFG0_TFFRQ2 =	3,
	CFG0_TBK0 =		4,	//Blank time setting
	CFG0_TBK1 =		5,
	CFG0_PFD0 =		6,	//Fast decay period setting
	CFG0_PFD1 =		7,
	CFG0_PFD2 =		8,
	CFG0_MXI0 =		9,	//Max phase current setting
	CFG0_MXI1 =		10,
	CFG0_MS0 =		11,	//Microstepping setting
	CFG0_MS1 =		12,
	CFG0_SYR =		13, //Synchronous rect setting
	CFG0_ADDR0 =	14, //Register address
	CFG0_ADDR1 =	15
};

enum CONFIG1_REG
{
	CFG1_DIAG0 =	0,	//Diagnostics select setting
	CFG1_DIAG1 =	1,
	CFG1_CD0 =		2,	//PWM count difference
	CFG1_CD1 =		3,
	CFG1_CD2 =		4,
	CFG1_CD3 =		5,
	CFG1_CD4 =		6,
	CFG1_CD5 =		7,
	CFG1_CD6 =		8,
	CFG1_CD7 =		9,
	//Bit 10 unused
	CFG1_TSC0 =		11, //Overcurrent fault delay setting
	CFG1_TSC1 =		12,
	CFG1_OSC =		13, //Clock source setting
	CFG1_ADDR0 =	14,
	CFG1_ADDR1 =	15	
};

enum RUN_REG
{
	RUN_SC0 =		0,	//Step change number
	RUN_SC1 =		1,
	RUN_SC2 =		2,
	RUN_SC3 =		3,
	RUN_SC4 =		4,
	RUN_SC5 =		5,
	RUN_DCY0 =		6,	//Decay mode setting
	RUN_DCY1 =		7,
	RUN_BRK =		8,	//Brake enable setting
	RUN_SLEW =		9,	//Slew rate control setting
	RUN_HLR =		10,	//Recirculation path setting
	RUN_OL0 =		11,	//Open load current setting
	RUN_OL1 =		12,
	RUN_EN =		13,	//Bridge enable setting
	RUN_ADDR0 =		14, //Register address
	RUN_ADDR1 =		15
};

enum TABLE_LOAD_REG
{
	PT0_OFFSET =		0, //Phase table
	PT1_OFFSET =		1,
	PT2_OFFSET =		2,
	PT3_OFFSET =		3,
	PT4_OFFSET =		4,
	PT5_OFFSET =		5,
	PTP_OFFSET =		6,
	//7-11 unused
	STS0_OFFSET =		12,	//Stall detection setting
	STS1_OFFSET =		13,
	TBLL_ADDR0_OFFSET =	14,
	TBLL_ADDR1_OFFSET =	15
};

enum FAULT0_REG
{
	FAULT0_FLAG =	15,
	FAULT0_TEMP1 =	14,
	FAULT0_TEMP0 =	13,
	FAULT0_OV =		12,
	FAULT0_UV =		11,
	FAULT0_STALL =	10,
	FAULT0_OLB =	9,
	FAULT0_OLA =	8,
	FAULT0_BML =	7,
	FAULT0_BMH =	6,
	FAULT0_BPL =	5,
	FAULT0_BPH =	4,
	FAULT0_AML =	3,
	FAULT0_AMH =	2,
	FAULT0_APL =	1,
	FAULT0_APH =	0
};

enum FAULT1_REG
{
	FAULT1_FLAG =	15,
	FAULT1_TEMP1 =	14,
	FAULT1_TEMP0 =	13,
	FAULT1_OV =		12,
	FAULT1_UV =		11,
	FAULT1_STALL =	10,
	FAULT1_OLB =	9,
	FAULT1_OLA =	8,
	//Bits 7 and 6 are unused
	FAULT1_SA5 =	5,
	FAULT1_SA4 =	4,
	FAULT1_SA3 =	3,
	FAULT1_SA2 =	2,
	FAULT1_SA1 =	1,
	FAULT1_SA0 =	0
};

typedef struct ctrlPins
{
	ioport_pin_t dir;
	ioport_pin_t step;
	ioport_pin_t _rstPort;
	ioport_pin_t ms1;
	ioport_pin_t ms0;
	ioport_pin_t en;
	ioport_pin_t _cs;
	ioport_pin_t diag;
}ctrlPins;

typedef struct motorConfig
{
	uint8_t motorID;			//Motor ID (i.e. rod pos, pump, etc)
	uint8_t controlType;		//Discrete or SPI control
	uint8_t direction;			//Motor direction
	uint8_t enabled;			//Flag for motor enabled
	uint32_t stepTime_ms;		//Current step time count
	float speed;				//Current set speed of motor in RPM (0 - 1000 in spec)
	float accelTime;			//Time in ms for acceleration
	float accelRate;			//Rate of acceleration (in pwm freq/ms)
	float decelTime;			//Time in ms for deceleration
	float decelRate;			//Rate of deceleration (in pwm freq/ms)
	uint16_t stepsToMove;		//Current steps to move
	uint8_t specifiedSteps;		//Specified number of steps to move (not freerunning)
	motorStepRes stepRes;		//Step resolution
	pwm_config PWMConfig;		//PWM configuration
	uint8_t PWMTC;				//PWM timer counter
	uint8_t PWMChan;			//PWM timer channel
	float PWMFreq;			//PWM frequency (Speed * GEAR_RATIO * STEPS_PER_REV * stepRes) //uint32_t
	uint32_t PWMEndFreq;		//Final PWM frequency
	uint8_t PWMDuty;			//PWM duty cycle
	uint16_t config0Reg;		//Config0 control register
	uint16_t config1Reg;		//Config1 control register
	uint16_t runReg;			//Run control register
	uint8_t curStepAngle;		//Current step angle for SPI control
	uint16_t tableLoadReg;		//Table load control register
	uint16_t fault0Reg;			//Fault 0 register
	uint16_t fault1Reg;			//Fault 1 register
	ctrlPins controlPins;		//Motor control pins	
} motorConfig;

//////////////////////////////////////////////////////////////////////////
// Function prototypes
//////////////////////////////////////////////////////////////////////////
void set_motor_step_resolution(motorConfig *motor, motorStepRes stepRes);
void set_motor_speed(motorConfig *motor, uint32_t speedPPS);
void set_motor_steps(motorConfig *motor, uint32_t steps);
void set_motor_direction(motorConfig *motor, uint8_t direction);
void set_motor_accel_time(motorConfig *motor, uint32_t accelTime);
void set_motor_decel_time(motorConfig *motor, uint32_t decelTime);
void step_motor(motorConfig *motor);
void handle_accel(motorConfig *motor);
void handle_decel(motorConfig *motor);
void init_motor_timers(motorConfig *motor);
void init_stepper_motor_pins(void);
uint8_t get_motor_hold_current_percent(motorConfig *motor);
uint8_t set_motor_hold_current(motorConfig *motor, uint8_t holdCur);
void start_motor(motorConfig *motor);
void stop_motor(motorConfig *motor);
void enable_motor(motorConfig *motor);
void disable_motor(motorConfig *motor);
void reset_faults(motorConfig *motor);
void reset_motor_config(motorConfig *motor);
void pwm_set_frequency_32bit(struct pwm_config *config, uint32_t freq_hz);

#endif /* STEPPERMOTOR_H_ */