#ifndef AVR2560_H
#define AVR2560_H

#include <avr/interrupt.h>
#include <avr/io.h>

/*
***********************************************************************************************************
* General
*/
#define EEPROM_LIB_H <EEPROM.h>
void initBoard();
uint16_t freeRam();

#define millis() (ms_counter) //Replaces the standard millis() function with this macro. It is both faster and more accurate. See timers.ino for its counter increment.
volatile unsigned long timer5_overflow_count = 0; //Increments every time counter 5 overflows. Used for the fast version of micros
static inline unsigned long micros_safe(); //A version of micros that is interrupt safe


#define SCHEDULE_TIMER_COUNTER TCNT3
#define SCHEDULE_TIMER_COMPARE OCR3A

#define MAX_TIMER_PERIOD 262140UL //The longest period of time (in uS) that the timer can permit (IN this case it is 65535 * 4, as each timer tick is 4uS)
#define MAX_TIMER_PERIOD_SLOW 1048560UL //The longest period of time (in uS) that the timer can permit (IN this case it is 65535 * 16, as each timer tick is 16uS)
#define uS_TO_TIMER_COMPARE(uS1) (uS1 >> 2) //Converts a given number of uS into the required number of timer ticks until that time has passed
#define uS_TO_TIMER_COMPARE_SLOW(uS1) (uS1 >> 4)

#define ENABLE_VVT_TIMER()    TIMSK1 |= (1 << OCIE1B)
#define DISABLE_VVT_TIMER()   TIMSK1 &= ~(1 << OCIE1B)

#define VVT_TIMER_COMPARE     OCR1B
#define VVT_TIMER_COUNTER     TCNT1

static inline uint32_t __attribute__((always_inline)) micros_safe()
{
    uint32_t newMicros;
    noInterrupts();
    newMicros = (((timer5_overflow_count << 16) + TCNT5) * 4);
    interrupts();
    return newMicros;
}

#endif //AVR2560_H
