/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

#include "globals.h"
#include "scheduler.h"


void initialise_schedule()
{
	schedule[FUEL1]=&fuel1_schedule;
	schedule[FUEL1]->start_call=&open_injector1;
	schedule[FUEL1]->end_call=&close_injector1;

	schedule[FUEL2]=&fuel2_schedule;
	schedule[FUEL2]->start_call=&open_injector2;
	schedule[FUEL2]->end_call=&close_injector2;

	schedule[FUEL3]=&fuel3_schedule;
	schedule[FUEL3]->start_call=&open_injector3;
	schedule[FUEL3]->end_call=&close_injector3;

	schedule[FUEL4]=&fuel4_schedule;
	schedule[FUEL4]->start_call=&open_injector4;
	schedule[FUEL4]->end_call=&close_injector4;

	schedule[IGN1]=&ign1_schedule;
	schedule[IGN1]->start_call=&charge_coil1;
	schedule[IGN1]->end_call=&fire_coil1;

	schedule[IGN2]=&ign2_schedule;
	schedule[IGN2]->start_call=&charge_coil2;
	schedule[IGN2]->end_call=&fire_coil2;

	schedule[IGN3]=&ign3_schedule;
	schedule[IGN3]->start_call=&charge_coil3;
	schedule[IGN3]->end_call=&fire_coil3;

	schedule[IGN4]=&ign4_schedule;
	schedule[IGN4]->start_call=&charge_coil4;
	schedule[IGN4]->end_call=&fire_coil4;

	uint8_t i=0;
	for(i=0;i<NUM_SCHEDULES;++i)
	{
		schedule[i]->current_state=OFF;
		schedule[i]->schedules_set=0;		
	}

    SCHEDULE_TIMER_DISABLE();
}

void schedule_halt()
{
	noInterrupts();
	uint8_t i;
	for(i=0;i<NUM_SCHEDULES;++i)
	{
		schedule[i]->end_call();
		schedule[i]->current_state = OFF;
	}
	SCHEDULE_TIMER_DISABLE();
	interrupts();
}

void set_schedule(uint8_t schedule_num, int32_t timeout, int32_t duration)
{
    if( (timeout < 10) )return;
	uint32_t curr_t=micros_safe();

    SCHEDULE_TIMER_DISABLE();
    if(schedule[schedule_num]->current_state == RUNNING) //Check that we're not already part way through a schedule
    {
        schedule[schedule_num]->next_start_time = curr_t + timeout ;
        schedule[schedule_num]->next_end_time = schedule[schedule_num]->next_start_time + duration;
        schedule[schedule_num]->has_next_time = true;
    }
    else
    {
        schedule[schedule_num]->start_time = curr_t + timeout;
        schedule[schedule_num]->end_time = schedule[schedule_num]->start_time + duration;
        schedule[schedule_num]->current_state = PENDING; //Turn this schedule on
        schedule[schedule_num]->schedules_set++; //Increment the number of times this schedule has been set
    }

	SCHEDULE_TIMER_COMPARE=1;
    SCHEDULE_TIMER_ENABLE();
	TCNT3=0;
}

ISR(TIMER3_COMPA_vect, ISR_NOBLOCK){
	SCHEDULE_TIMER_DISABLE();
	uint32_t curr_t=micros_safe();
	uint32_t next_t=(~(uint32_t)0);
	uint8_t i;
	for(i=0;i<NUM_SCHEDULES;++i)
	{
		if(schedule[i]->current_state==PENDING)
		{
			if( curr_t >= schedule[i]->start_time )
			{
				schedule[i]->start_call();
				schedule[i]->current_state=RUNNING;
			}
		}
		if(schedule[i]->current_state==RUNNING)
		{
			if( curr_t >= schedule[i]->end_time )
			{
				schedule[i]->end_call();
				schedule[i]->current_state=OFF;
				if(schedule[i]->has_next_time == true)
				{
					schedule[i]->start_time = schedule[i]->next_start_time;
					schedule[i]->end_time = schedule[i]->next_end_time;
					schedule[i]->has_next_time = false;
					schedule[i]->current_state = PENDING;
				}
			}
		}

		
		if(schedule[i]->current_state==PENDING)
		{
			if(schedule[i]->start_time < next_t)next_t=schedule[i]->start_time;
		}
		if(schedule[i]->current_state==RUNNING)
		{
			if(schedule[i]->end_time < next_t)next_t=schedule[i]->end_time;
		}
	}

	if ( next_t > curr_t )next_t-=curr_t;//next_t now stores the delay until next schedule
	else{ next_t=0; };

	if(next_t<MAX_TIMER_PERIOD_SLOW)
	{
		SCHEDULE_TIMER_COMPARE=uS_TO_TIMER_COMPARE_SLOW(next_t);
	}
	else{
		SCHEDULE_TIMER_COMPARE=uS_TO_TIMER_COMPARE_SLOW(MAX_TIMER_PERIOD_SLOW);
	}

	TCNT3=0;
	SCHEDULE_TIMER_ENABLE();
}
