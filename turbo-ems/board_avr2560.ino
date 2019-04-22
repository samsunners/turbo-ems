#include "globals.h"
#include "auxiliaries.h"


void initBoard()
{
    // Auxilliaries
    //
    //PWM used by the Boost and VVT outputs
    TCCR1B = 0x00;          //Disbale Timer1 while we set it up
    TCNT1  = 0;             //Reset Timer Count
    TIFR1  = 0x00;          //Timer1 INT Flag Reg: Clear Timer Overflow Flag
    TCCR1A = 0x00;          //Timer1 Control Reg A: Wave Gen Mode normal (Simply counts up from 0 to 65535 (16-bit int)
    TCCR1B = (1 << CS12);   //Timer1 Control Reg B: Timer Prescaler set to 256. 1 tick = 16uS. Refer to http://www.instructables.com/files/orig/F3T/TIKL/H3WSA4V7/F3TTIKLH3WSA4V7.jpg

    vvt_pwm_max_count = 1000000L / (16 * configPage6.vvtFreq * 2); //Converts the frequency in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle

    /*
    ***********************************************************************************************************
    * Timers
    */
    //Configure Timer2 for our low-freq interrupt code.
    TCCR2B = 0x00;          //Disbale Timer2 while we set it up
    TCNT2  = 131;           //Preload timer2 with 131 cycles, leaving 125 till overflow. As the timer runs at 125Khz, this causes overflow to occur at 1Khz = 1ms
    TIFR2  = 0x00;          //Timer2 INT Flag Reg: Clear Timer Overflow Flag
    TIMSK2 = 0x01;          //Timer2 Set Overflow Interrupt enabled.
    TCCR2A = 0x00;          //Timer2 Control Reg A: Wave Gen Mode normal
    /* Now configure the prescaler to CPU clock divided by 128 = 125Khz */
    TCCR2B |= (1<<CS22)  | (1<<CS20); // Set bits
    TCCR2B &= ~(1<<CS21);             // Clear bit

    //Much help in this from http://arduinomega.blogspot.com.au/2011/05/timer2-and-overflow-interrupt-lets-get.html
    //Fuel Schedules, which uses timer 3
    TCCR3B = 0x00;          //Disable Timer3 while we set it up
    TCNT3  = 0;             //Reset Timer Count
    TIFR3  = 0x00;          //Timer3 INT Flag Reg: Clear Timer Overflow Flag
    TCCR3A = 0x00;          //Timer3 Control Reg A: Wave Gen Mode normal
    TCCR3B = (1 << CS12);   //Timer3 Control Reg B: Timer Prescaler set to 256. Refer to http://www.instructables.com/files/orig/F3T/TIKL/H3WSA4V7/F3TTIKLH3WSA4V7.jpg

    //Ignition Schedules, which uses timer 5. This is also used by the fast version of micros. If the speed of this timer is changed from 4uS ticks, that MUST be changed as well. See globals.h and timers.ino
    TCCR5B = 0x00;          //Disable Timer5 while we set it up
    TCNT5  = 0;             //Reset Timer Count
    TCCR5A = 0x00;          //Timer5 Control Reg A: Wave Gen Mode normal
    TCCR5B = (1 << CS11) | (1 << CS10); //Timer5 Control Reg B: Timer Prescaler set to 64. Refer to http://www.instructables.com/files/orig/F3T/TIKL/H3WSA4V7/F3TTIKLH3WSA4V7.jpg
    TIFR5 |= (1 << OCF5A) | (1<<OCF5B) | (1<<OCF5C); //Clear the compare flag bits
    TIMSK5 |= (1 << TOIE5); //Enable the timer5 overflow interrupt (See timers.ino for ISR)
    TIMSK0 &= ~_BV(TOIE0); // disable timer0 overflow interrupt (used by arduino default micros)
}

uint16_t freeRam()
{
    extern int __heap_start, *__brkval;
    int currentVal;
    uint16_t v;

    if(__brkval == 0) { currentVal = (int) &__heap_start; }
    else { currentVal = (int) __brkval; }

    //Old version:
    //return (uint16_t) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
    return (uint16_t) &v - currentVal; //cppcheck-suppress misra-c2012-11.4
}

//This is used by the fast version of micros. We just need to increment the timer overflow counter
ISR(TIMER5_OVF_vect)
{
    ++timer5_overflow_count;
}
