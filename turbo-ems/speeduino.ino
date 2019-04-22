/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,la
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/
#include <stdint.h> //https://developer.mbed.org/handbook/C-Data-Types

#include "globals.h"
#include "speeduino.h"
#include "table.h"
#include "scheduler.h"
#include "comms.h"
#include "maths.h"
#include "corrections.h"
#include "decoders.h"
#include "auxiliaries.h"
#include "sensors.h"
#include "storage.h"
#include "angles.h"
#include "init.h"
#include "config.h"
#include "board_avr2560.h"

void setup()
{
    pinMode(PIN_LED_BLUE,OUTPUT);
    initialiseAll();
    tpic8101_init()!=true;
}

void loop()
{
    mainLoopCount++;

    if (Serial.available() > 0) {
        command();
    }

    previousLoopTime = currentLoopTime;
    currentLoopTime = micros_safe();

    if ( ((currentLoopTime - crank.tim[crank.last]) < (uint32_t)MAX_STALL_TIME) || (crank.tim[crank.last] > currentLoopTime) )
    {
        update_RPM();
        currentStatus.longRPM = crank.rpm; //Long RPM is included here
        currentStatus.RPM = currentStatus.longRPM;
        FUEL_PUMP_ON();
        currentStatus.fuelPumpOn = true; //Not sure if this is needed.
    } else
    {
		schedule_halt();
        crank.tooth=-1;
        cam.tooth = -1;
        currentStatus.RPM = 0;
        currentStatus.PW1 = 0;
        currentStatus.VE = 0;
        crank.tim[crank.last] = 0;
        //toothLastMinusOneToothTime = 0;
        currentStatus.hasSync = false;
        currentStatus.runSecs = 0; //Reset the counter for number of seconds running.
        secCounter = 0; //Reset our seconds counter.
        MAPcurRev = 0;
        MAPcount = 0;
        currentStatus.rpmDOT = 0;
        AFRnextCycle = 0;
        ignitionCount = 0;
        ignitionOn = false;
        fuelOn = false;
        FUEL_PUMP_OFF();
        BIT_CLEAR(currentStatus.engine, BIT_ENGINE_CRANK); //Clear cranking bit (Can otherwise get stuck 'on' even with 0 rpm)
        BIT_CLEAR(currentStatus.engine, BIT_ENGINE_WARMUP); //Same as above except for WUE
        BIT_CLEAR(currentStatus.engine, BIT_ENGINE_RUN); //Same as above except for RUNNING status
        BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ASE); //Same as above except for ASE status
//        VVT_PIN_LOW();
//        DISABLE_VVT_TIMER();
    }

    if (currentStatus.hasSync && (currentStatus.RPM > 0))
    {
        ignitionOn = true;
        fuelOn = true;
        //Check whether running or cranking
        if(currentStatus.RPM > currentStatus.crankRPM)
        {
            BIT_SET(currentStatus.engine, BIT_ENGINE_RUN); //Sets the engine running bit
            BIT_CLEAR(currentStatus.engine, BIT_ENGINE_CRANK);
        }
        else
        {
            //Sets the engine cranking bit, clears the engine running bit
            BIT_SET(currentStatus.engine, BIT_ENGINE_CRANK);
            BIT_CLEAR(currentStatus.engine, BIT_ENGINE_RUN);
            currentStatus.runSecs = 0; //We're cranking (hopefully), so reset the engine run time to prompt ASE.
        }
    } else {
        ignitionOn=false;
        fuelOn=false;
    }

    update_crank_angle();

    if (fuelOn)
    {
        getPW();
        unsigned long pwLimit = au_to_time(CONFIG_PULSE_WIDTH_LIMIT); //The pulsewidth limit in degrees
        if (currentStatus.PW1 > pwLimit) {
            currentStatus.PW1 = pwLimit;
        };
        correctionsFuel();

        if(currentStatus.PW1 > inj_opentime_uS)
        {
            au_t temp_angle;

            temp_angle=CONFIG_INJ1_START_ANGLE - crank.angle;
			//TODO: The if(temp_angle<32767) prevents a missed schedule due to temp_angle rollover. We should find a more elegant solution.
			if(temp_angle<32767)set_schedule(FUEL1, au_to_time(temp_angle), currentStatus.PW1);

            temp_angle=CONFIG_INJ2_START_ANGLE - crank.angle;
            if(temp_angle<32767)set_schedule(FUEL2, au_to_time(temp_angle), currentStatus.PW1);

            temp_angle=CONFIG_INJ3_START_ANGLE - crank.angle;
            if(temp_angle<32767)set_schedule(FUEL3, au_to_time(temp_angle), currentStatus.PW1);

            temp_angle=CONFIG_INJ4_START_ANGLE - crank.angle;
            if(temp_angle<32767)set_schedule(FUEL4, au_to_time(temp_angle), currentStatus.PW1);
        }
    }//fuelon


    if(ignitionOn)
    {
        getAdvance();
        au_t temp_angle;

        temp_angle=CONFIG_CYLINDER1_TDC - DEGREES_TO_AU(currentStatus.advance*2) - crank.angle;
        if(temp_angle<32767)set_schedule(IGN1, au_to_time(temp_angle)-CONFIG_DWELL,CONFIG_DWELL);

        temp_angle=CONFIG_CYLINDER2_TDC - DEGREES_TO_AU(currentStatus.advance*2) - crank.angle;
        if(temp_angle<32767)set_schedule(IGN2, au_to_time(temp_angle)-CONFIG_DWELL,CONFIG_DWELL);

        temp_angle=CONFIG_CYLINDER3_TDC - DEGREES_TO_AU(currentStatus.advance*2) - crank.angle;
        if(temp_angle<32767)set_schedule(IGN3, au_to_time(temp_angle)-CONFIG_DWELL,CONFIG_DWELL);

        temp_angle=CONFIG_CYLINDER4_TDC - DEGREES_TO_AU(currentStatus.advance*2) - crank.angle;
        if(temp_angle<32767)set_schedule(IGN4, au_to_time(temp_angle)-CONFIG_DWELL,CONFIG_DWELL);
    } //ignition on
} //loop


inline void __attribute__((always_inline)) getPW()
{
	getVE();
    uint32_t temp=req_fuel_uS;
    temp*=currentStatus.VE;
    temp*=currentStatus.MAP;
    temp/=currentStatus.baro;
    temp*=currentStatus.corrections;
    temp/=10000;
    currentStatus.PW1 = temp;
}


inline void __attribute__((always_inline)) getVE()
{
    currentStatus.fuelLoad = currentStatus.MAP;
    currentStatus.VE = get3DTableValue(&fuelTable, currentStatus.fuelLoad, currentStatus.RPM); //Perform lookup into fuel map for RPM vs MAP value
}

inline void __attribute__((always_inline)) getAdvance()
{
    int8_t tempAdvance = 0;
    currentStatus.ignLoad = currentStatus.MAP;
    tempAdvance = get3DTableValue(&ignitionTable, currentStatus.ignLoad, currentStatus.RPM) - OFFSET_IGNITION; //As above, but for ignition advance
    currentStatus.advance = correctionsIgn(tempAdvance);
}
