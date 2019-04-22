#ifndef SENSORS_H
#define SENSORS_H

#include "Arduino.h"

// The following are alpha values for the ADC filters.
// Their values are from 0 to 255 with 0 being no filtering and 255 being maximum
/*
#define ADCFILTER_TPS  128
#define ADCFILTER_CLT  180
#define ADCFILTER_IAT  180
#define ADCFILTER_O2   128
#define ADCFILTER_BAT  128
#define ADCFILTER_MAP   20 //This is only used on Instantaneous MAP readings and is intentionally very weak to allow for faster response
#define ADCFILTER_BARO  64
*/

#define BARO_MIN      87
#define BARO_MAX      108

#define KNOCK_MODE_DIGITAL  1
#define KNOCK_MODE_ANALOG   2

volatile byte knockCounter = 0;
volatile uint16_t knockAngle;

unsigned long MAPrunningValue; //Used for tracking either the total of all MAP readings in this cycle (Event average) or the lowest value detected in this cycle (event minimum)
unsigned int MAPcount; //Number of samples taken in the current MAP cycle
uint32_t MAPcurRev; //Tracks which revolution we're sampling on

//These variables are used for tracking the number of running sensors values that appear to be errors. Once a threshold is reached, the sensor reading will go to default value and assume the sensor is faulty
byte mapErrorCount = 0;
byte tpsErrorCount = 0;
byte iatErrorCount = 0;
byte cltErrorCount = 0;

/*
 * Simple low pass IIR filter macro for the analog inputs
 * This is effectively implementing the smooth filter from http://playground.arduino.cc/Main/Smooth
 * But removes the use of floats and uses 8 bits of fixed precision.
 */
#define ADC_FILTER(input, alpha, prior) (((long)input * (256 - alpha) + ((long)prior * alpha))) >> 8
//These functions all do checks on a pin to determine if it is already in use by another (higher importance) function
#define pinIsInjector(pin)  ( (pin == PIN_INJ1) || (pin == PIN_INJ2) || (pin == PIN_INJ3) || (pin == PIN_INJ4) )
#define pinIsIgnition(pin)  ( (pin == PIN_IGN1) || (pin == PIN_IGN2) || (pin == PIN_IGN3) || (pin == PIN_IGN4) )
#define pinIsSensor(pin)    ( (pin == PIN_CLT) || (pin == PIN_IAT) || (pin == PIN_MAP) || (pin == PIN_TPS) || (pin == PIN_O2) || (pin == PIN_BAT) )
#define pinIsUsed(pin)      ( pinIsInjector(pin) || pinIsIgnition(pin) || pinIsSensor(pin) )

static inline void readMAP() __attribute__((always_inline));
void initialiseADC();
void readTPS();
void readCLT();
void readIAT();
void readO2();
void readBat();

#endif // SENSORS_H
