/*
 * TEC_Controller.h
 *
 * Created: 10/16/2019 12:19:08 PM
 *  Author: adam
 */ 


#ifndef TEC_CONTROLLER_H_
#define TEC_CONTROLLER_H_

//////////////////////////////////////////////////////////////////////////
// TEC control pins
//////////////////////////////////////////////////////////////////////////
#define _TEC_RST_A			IOPORT_CREATE_PIN(PORTA, 4)
#define _TEC_RST_B			IOPORT_CREATE_PIN(PORTA, 5)
#define _TEC_OTW			IOPORT_CREATE_PIN(PORTB, 0)
#define _TEC_FAULT			IOPORT_CREATE_PIN(PORTB, 1)
#define _FAN_FR_STBY		IOPORT_CREATE_PIN(PORTE, 3)

#define TEC_PWM_A			IOPORT_CREATE_PIN(PORTE, 0)
#define TEC_PWM_B			IOPORT_CREATE_PIN(PORTE, 1)
#define FAN_PWM				IOPORT_CREATE_PIN(PORTE, 2)
#define TEC_FAN_TC				PWM_TCE0
#define TEC_FAN_CHAN			PWM_CH_C
#define TEC_PWM_A_TC			PWM_TCE0
#define TEC_PWM_A_CHAN			PWM_CH_A
#define TEC_PWM_B_TC			PWM_TCE0
#define TEC_PWM_B_CHAN			PWM_CH_B

//////////////////////////////////////////////////////////////////////////
// Function prototypes
//////////////////////////////////////////////////////////////////////////
void init_TEC_controller(void);
void disable_TEC_controller(void);
void set_TEC_pwm_A_duty(uint8_t duty);
void enable_TEC_A(void);
void disable_TEC_A(void);
void set_TEC_pwm_A_freq(uint32_t freq);
void stop_TEC_pwm_A(void);
void set_TEC_pwm_B_duty(uint8_t duty);
void set_TEC_pwm_B_freq(uint32_t freq);
void enable_TEC_B(void);
void disable_TEC_B(void);
void stop_TEC_pwm_B(void);
void set_TEC_fan_duty(uint8_t duty);
void set_TEC_fan_freq(uint16_t freq);
uint8_t get_TEC_fault_status(void);
uint8_t get_TEC_OTW_status(void);
void reset_TEC_A(void);
void reset_TEC_B(void);
void recover_from_OTSD(void);
void clear_fan_fault(void);
void enter_fan_standby(void);
void pwm_set_TEC_frequency_32bit(struct pwm_config *config, uint32_t freq_hz);

#endif /* TEC_CONTROLLER_H_ */