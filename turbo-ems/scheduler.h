/*
This scheduler is designed to maintain 2 schedules for use by the fuel and ignition systems.
It functions by waiting for the overflow vectors from each of the timers in use to overflow, which triggers an interrupt

//Technical
Currently I am prescaling the 16-bit timers to 256 for injection and 64 for ignition. This means that the counter increments every 16us (injection) / 4uS (ignition) and will overflow every 1048576uS
Max Period = (Prescale)*(1/Frequency)*(2^17)
(See http://playground.arduino.cc/code/timer1)
This means that the precision of the scheduler is 16uS (+/- 8uS of target) for fuel and 4uS (+/- 2uS) for ignition

/Features
This differs from most other schedulers in that its calls are non-recurring (IE You schedule an event at a certain time and once it has occurred, it will not reoccur unless you explicitely ask for it)
Each timer can have only 1 callback associated with it at any given time. If you call the setCallback function a 2nd time, the original schedule will be overwritten and not occur

Timer identification
The Arduino timer3 is used for schedule 1
The Arduino timer4 is used for schedule 2
Both of these are 16-bit timers (ie count to 65536)
See page 136 of the processors datasheet: http://www.atmel.com/Images/doc2549.pdf

256 prescale gives tick every 16uS
256 prescale gives overflow every 1048576uS (This means maximum wait time is 1.0485 seconds)

*/
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "config.h"

#define SCHEDULE_TIMER_ENABLE() TIMSK3 |= (1 << OCIE3A);
#define SCHEDULE_TIMER_DISABLE() TIMSK3 &= ~(1 << OCIE3A);

void open_injector1()
{
	*inj1_pin_port |= (inj1_pin_mask);
	BIT_SET(currentStatus.status1, BIT_STATUS1_INJ1);
}

void close_injector1()
{
	*inj1_pin_port &= ~(inj1_pin_mask); 
	BIT_CLEAR(currentStatus.status1, BIT_STATUS1_INJ1);
}
void open_injector2()
{
	*inj2_pin_port |= (inj2_pin_mask);
	BIT_SET(currentStatus.status1, BIT_STATUS1_INJ2);
}

void close_injector2()
{
	*inj2_pin_port &= ~(inj2_pin_mask);
	BIT_CLEAR(currentStatus.status1, BIT_STATUS1_INJ2);
}

void open_injector3()
{
	*inj3_pin_port |= (inj3_pin_mask);
	BIT_SET(currentStatus.status1, BIT_STATUS1_INJ3);
}

void close_injector3()
{
	*inj3_pin_port &= ~(inj3_pin_mask);
	BIT_CLEAR(currentStatus.status1, BIT_STATUS1_INJ3);
}

void open_injector4()
{
	*inj4_pin_port |= (inj4_pin_mask);
	BIT_SET(currentStatus.status1, BIT_STATUS1_INJ4);
}

void close_injector4(){
	*inj4_pin_port &= ~(inj4_pin_mask);
	BIT_CLEAR(currentStatus.status1, BIT_STATUS1_INJ4);
}

void charge_coil1(){
	*ign1_pin_port |= (ign1_pin_mask);
}

void fire_coil1(){
	*ign1_pin_port &= ~(ign1_pin_mask);
}

void charge_coil2(){
	*ign2_pin_port |= (ign2_pin_mask);
}

void fire_coil2(){
	*ign2_pin_port &= ~(ign2_pin_mask);
}

void charge_coil3(){
	*ign3_pin_port |= (ign3_pin_mask);
}

void fire_coil3(){
	*ign3_pin_port &= ~(ign3_pin_mask);
}

void charge_coil4(){
	*ign4_pin_port |= (ign4_pin_mask);
}

void fire_coil4(){
	*ign4_pin_port &= ~(ign4_pin_mask);
}

void initialise_schedule();

enum SCHEDULE_STATE {OFF, PENDING, STAGED, RUNNING}; //The 3 statuses that a schedule can have

#define NUM_SCHEDULES 8
#define FUEL1 0
#define FUEL2 1
#define FUEL3 2
#define FUEL4 3
#define IGN1 4
#define IGN2 5
#define IGN3 6
#define IGN4 7

struct Schedule {
	void (*start_call)();
	void (*end_call)();
    volatile uint8_t current_state;
    volatile uint8_t schedules_set; //A counter of how many times the schedule has been set
    volatile uint32_t start_time;
    volatile uint32_t end_time;
    volatile uint32_t next_start_time;
    volatile uint32_t next_end_time;
    volatile bool has_next_time;
};

Schedule fuel1_schedule;
Schedule fuel2_schedule;
Schedule fuel3_schedule;
Schedule fuel4_schedule;
Schedule ign1_schedule;
Schedule ign2_schedule;
Schedule ign3_schedule;
Schedule ign4_schedule;

Schedule *schedule[NUM_SCHEDULES];

#endif // SCHEDULER_H
