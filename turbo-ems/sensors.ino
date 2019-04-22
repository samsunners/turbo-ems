/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/
#include "sensors.h"
#include "angles.h"
#include "globals.h"
#include "maths.h"
#include "storage.h"
#include "comms.h"
#include "config.h"

void initialiseADC()
{
    //This sets the ADC (Analog to Digitial Converter) to run at 1Mhz, greatly reducing analog read times (MAP/TPS) when using the standard analogRead() function
    //1Mhz is the fastest speed permitted by the CPU without affecting accuracy
    //Please see chapter 11 of 'Practical Arduino' (http://books.google.com.au/books?id=HsTxON1L6D4C&printsec=frontcover#v=onepage&q&f=false) for more detail
    BIT_SET(ADCSRA,ADPS2);
    BIT_CLEAR(ADCSRA,ADPS1);
    BIT_CLEAR(ADCSRA,ADPS0);
    MAPcurRev = 0;
    MAPcount = 0;
    MAPrunningValue = 0;

    //Sanity checks to ensure none of the filter values are set above 240 (Which would include the 255 value which is the default on a new arduino)
    //If an invalid value is detected, it's reset to the default the value and burned to EEPROM.
    //Each sensor has it's own default value
    if(configPage4.ADCFILTER_TPS > 240) {
        configPage4.ADCFILTER_TPS = 50;
        writeConfig(ignSetPage);
    }
    if(configPage4.ADCFILTER_CLT > 240) {
        configPage4.ADCFILTER_CLT = 180;
        writeConfig(ignSetPage);
    }
    if(configPage4.ADCFILTER_IAT > 240) {
        configPage4.ADCFILTER_IAT = 180;
        writeConfig(ignSetPage);
    }
    if(configPage4.ADCFILTER_O2  > 240) {
        configPage4.ADCFILTER_O2 = 100;
        writeConfig(ignSetPage);
    }
    if(configPage4.ADCFILTER_BAT > 240) {
        configPage4.ADCFILTER_BAT = 128;
        writeConfig(ignSetPage);
    }
    if(configPage4.ADCFILTER_MAP > 240) {
        configPage4.ADCFILTER_MAP = 20;
        writeConfig(ignSetPage);
    }
    if(configPage4.ADCFILTER_BARO > 240) {
        configPage4.ADCFILTER_BARO = 64;
        writeConfig(ignSetPage);
    }

}

static inline void readMAP()
{
    unsigned int tempReading = analogRead(PIN_MAP);

    //Error checking
    if( (tempReading >= CONFIG_VALID_MAP_MAX) || (tempReading <= CONFIG_VALID_MAP_MIN) ) {
        mapErrorCount += 1;
    }
    else {
        mapErrorCount = 0;
    }

    currentStatus.mapADC = tempReading;

    currentStatus.MAP = fastMap10Bit(currentStatus.mapADC, CONFIG_MAP_MIN, CONFIG_MAP_MAX); //Get the current MAP value
    if(currentStatus.MAP < 0) {
        currentStatus.MAP = 0;    //Sanity check
    }
}

void readTPS()
{
    unsigned int tempReading=analogRead(PIN_TPS);
    currentStatus.tpsADC=map(tempReading, CONFIG_TPS_MIN, CONFIG_TPS_MAX, 0, 255);
    currentStatus.TPSlast = currentStatus.TPS;
    currentStatus.TPSlast_time = currentStatus.TPS_time;

    if( (tempReading >= CONFIG_TPS_VALID_MAX) || (tempReading <= CONFIG_TPS_VALID_MIN) ) {
        tpsErrorCount += 1;
        currentStatus.TPS=0;
        currentStatus.TPS_time=0;
        return;
    }

    tpsErrorCount = 0;
    currentStatus.TPS = map(tempReading, CONFIG_TPS_MIN, CONFIG_TPS_MAX, 0, 100); //Take the raw TPS ADC value and convert it into a TPS% based on the calibrated values
    currentStatus.TPS_time = micros_safe();
}

void readCLT()//coolant temp
{	//for 470ohm bias resistor t(celcius)=30.2v-7.8
    currentStatus.cltADC = analogRead(PIN_CLT);
	int16_t temp = 960-currentStatus.cltADC;
	temp*=100;
	temp/=678;
    currentStatus.coolant = temp;
}

void readIAT()//intake air temp
{
    unsigned int tempReading;
    tempReading = fastMap1023toX(analogRead(PIN_IAT), 511); //Get the current raw IAT value
    currentStatus.iatADC = ADC_FILTER(tempReading, configPage4.ADCFILTER_IAT, currentStatus.iatADC);
    currentStatus.IAT = currentStatus.iatADC;//TODO: Calibrate
}

void readO2()
{
    unsigned int tempReading;
    tempReading = analogRead(PIN_O2);
    tempReading = fastMap1023toX(analogRead(PIN_O2), 511); //Get the current O2 value.
    currentStatus.O2ADC = ADC_FILTER(tempReading, configPage4.ADCFILTER_O2, currentStatus.O2ADC);
    currentStatus.O2 = currentStatus.O2ADC;//TODO: Calibrate
}

void readBat()
{
    currentStatus.battery10 = fastMap1023toX(analogRead(PIN_BAT), CONFIG_BAT_MAX);
}
