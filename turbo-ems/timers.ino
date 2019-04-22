/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/*
Timers are used for having actions performed repeatedly at a fixed interval (Eg every 100ms)
They should not be confused with Schedulers, which are for performing an action once at a given point of time in the future

Timers are typically low resolution (Compared to Schedulers), with maximum frequency currently being approximately every 10ms
*/
#include "globals.h"
#include "sensors.h"
#include "scheduler.h"
#include "auxiliaries.h"
#include <avr/wdt.h>

//Timer2 Overflow Interrupt Vector, called when the timer overflows ever 1 ms.
ISR(TIMER2_OVF_vect, ISR_NOBLOCK) //This MUST be no block.
{
    static uint16_t ms_count = 0;
    ms_count++;

    readMAP();
    readTPS();
    readO2();

    if (ms_count % 100 == 1)
    {
//        vvtControl();

        static uint16_t lastRPM_100ms=0;
        currentStatus.rpmDOT = (currentStatus.RPM - lastRPM_100ms) * 10; //This is the RPM per second that the engine has accelerated/decelleratedin the last loop
        lastRPM_100ms = currentStatus.RPM; //Record the current RPM for next calc
    }


    if (ms_count % 250 == 2)
    {
        if(eepromWritesPending == true) {
            writeAllConfig();    //Check for any outstanding EEPROM writes.
        }
        readCLT();
        readBat();
        readIAT();
        //Serial.print(AU_TO_DEGREES(cam.angle));Serial.print(",");Serial.print(AU_TO_DEGREES(crank.angle));Serial.print(",");Serial.println(AU_TO_DEGREES(cam.angle-crank.angle));
        //Serial.print(cam.error_skipped_tooth);Serial.print(",");Serial.println(cam.error_extra_tooth);
        //Serial.print(crank.error_skipped_tooth);Serial.print(",");Serial.println(crank.error_extra_tooth);
//		Serial.println(AU_TO_DEGREES(time_to_au(crank.gap[crank.last])));
    }

    if (ms_count == 1000)//every second
    {
        ms_count = 0;
        currentStatus.crankRPM = ((unsigned int)configPage4.crankRPM * 10);

        if (BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN))
        {
            if (currentStatus.runSecs <= 254)currentStatus.runSecs++;
        }

        currentStatus.loopsPerSecond = mainLoopCount;
        mainLoopCount = 0;

        currentStatus.secl++;//secl is simply a counter that increments every second and is used to track whether the system has unexpectedly reset

        if (configPage6.fanEnable == 1)
        {
            fanControl();            // Fucntion to turn the cooling fan on/off
        }
    }

    TCNT2 = 131;//Reset Timer2 to trigger in another ~1ms
}
