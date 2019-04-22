/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/
#include "config.h"
#include "globals.h"
#include "auxiliaries.h"
#include "src/PID_v1/PID_v1.h"

void initialiseFan()
{
    digitalWrite(PIN_FAN, CONFIG_FAN_OFF);         //Initiallise program with the fan in the off state
    currentStatus.fanOn = false;

    fan_pin_port = portOutputRegister(digitalPinToPort(PIN_FAN));
    fan_pin_mask = digitalPinToBitMask(PIN_FAN);
}

void fanControl()
{
    if ( currentStatus.coolant >= CONFIG_FAN_ON_TEMP )
    {
        digitalWrite(PIN_FAN,CONFIG_FAN_ON);
    }
    else if( currentStatus.coolant <= CONFIG_FAN_OFF_TEMP )
    {
        digitalWrite(PIN_FAN,CONFIG_FAN_OFF);
    }
}

void initialiseVVT()
{
    vvt_pin_port = portOutputRegister(digitalPinToPort(PIN_VVT));
    vvt_pin_mask = digitalPinToBitMask(PIN_VVT);
    ENABLE_VVT_TIMER(); //Turn on the B compare unit (ie turn on the interrupt)
}

void vvtControl()
{
    if( configPage6.vvtEnabled == 1 )
    {
        byte vvtDuty = get3DTableValue(&vvtTable, currentStatus.TPS, currentStatus.RPM);

        //VVT table can be used for controlling on/off switching. If this is turned on, then disregard any interpolation or non-binary values
        if( (configPage6.VVTasOnOff == true) && (vvtDuty < 100) ) {
            vvtDuty = 0;
        }

        if(vvtDuty == 0)
        {
            //Make sure solenoid is off (0% duty)
            VVT_PIN_LOW();
            DISABLE_VVT_TIMER();
        }
        else if (vvtDuty >= 100)
        {
            //Make sure solenoid is on (100% duty)
            VVT_PIN_HIGH();
            DISABLE_VVT_TIMER();
        }
        else
        {
            vvt_pwm_target_value = percentage(vvtDuty, vvt_pwm_max_count);
            ENABLE_VVT_TIMER();
        }
    }
    else {
        DISABLE_VVT_TIMER();    // Disable timer channel
    }
}

//The interrupt to control the VVT PWM
ISR(TIMER1_COMPB_vect)
{
    if (vvt_pwm_state == true)
    {
        VVT_PIN_LOW();  // Switch pin to low
        VVT_TIMER_COMPARE = VVT_TIMER_COUNTER + (vvt_pwm_max_count - vvt_pwm_cur_value);
        vvt_pwm_state = false;
    }
    else
    {
        VVT_PIN_HIGH();  // Switch pin high
        VVT_TIMER_COMPARE = VVT_TIMER_COUNTER + vvt_pwm_target_value;
        vvt_pwm_cur_value = vvt_pwm_target_value;
        vvt_pwm_state = true;
    }
}
