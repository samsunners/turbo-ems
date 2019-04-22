#ifndef AUX_H
#define AUX_H

#include "board_avr2560.h"

void initialiseVVT();
void vvtControl();
void initialiseFan();
void fanControl();

#define VVT_PIN_LOW()    *vvt_pin_port &= ~(vvt_pin_mask)
#define VVT_PIN_HIGH()   *vvt_pin_port |= (vvt_pin_mask)
#define FAN_PIN_LOW()    *fan_pin_port &= ~(fan_pin_mask)
#define FAN_PIN_HIGH()   *fan_pin_port |= (fan_pin_mask)

volatile uint8_t *vvt_pin_port;
volatile uint8_t vvt_pin_mask;
volatile uint8_t *fan_pin_port;
volatile uint8_t fan_pin_mask;

volatile bool vvt_pwm_state;
unsigned int vvt_pwm_max_count; //Used for variable PWM frequency
volatile unsigned int vvt_pwm_cur_value;
long vvt_pwm_target_value;
static inline void vvtInterrupt();

#endif
