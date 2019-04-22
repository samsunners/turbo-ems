#include "globals.h"
#include "init.h"
#include "storage.h"
#include "speeduino.h"
#include "utils.h"
#include "scheduler.h"
#include "auxiliaries.h"
#include "sensors.h"
#include "decoders.h"
#include "corrections.h"
#include "table.h"
#include "board_avr2560.h"

void initialiseAll()
{
    fire_coil1();
    fire_coil2();
    fire_coil3();
    fire_coil4();
    close_injector1();
    close_injector2();
    close_injector3();
    close_injector4();

    Serial.begin(115200);

    table3D_setSize(&fuelTable, 16);
    table3D_setSize(&ignitionTable, 16);
    table3D_setSize(&afrTable, 16);
    table3D_setSize(&stagingTable, 8);
    table3D_setSize(&boostTable, 8);
    table3D_setSize(&vvtTable, 8);

    loadConfig();

    initBoard(); //This calls the current individual boards init function. See the board_xxx.ino files for these.

    //Repoint the 2D table structs to the config pages that were just loaded
    taeTable.valueSize = SIZE_BYTE; //Set this table to use byte values
    taeTable.xSize = 4;
    taeTable.values = configPage4.taeValues;
    taeTable.axisX = configPage4.taeBins;
    WUETable.valueSize = SIZE_BYTE; //Set this table to use byte values
    WUETable.xSize = 10;
    WUETable.values = configPage2.wueValues;
    WUETable.axisX = configPage4.wueBins;
    crankingEnrichTable.valueSize = SIZE_BYTE;
    crankingEnrichTable.xSize = 4;
    crankingEnrichTable.values = configPage10.crankingEnrichValues;
    crankingEnrichTable.axisX = configPage10.crankingEnrichBins;

    injectorVCorrectionTable.valueSize = SIZE_BYTE;
    injectorVCorrectionTable.xSize = 6;
    injectorVCorrectionTable.values = configPage6.injVoltageCorrectionValues;
    injectorVCorrectionTable.axisX = configPage6.voltageCorrectionBins;
    IATDensityCorrectionTable.valueSize = SIZE_BYTE;
    IATDensityCorrectionTable.xSize = 9;
    IATDensityCorrectionTable.values = configPage6.airDenRates;
    IATDensityCorrectionTable.axisX = configPage6.airDenBins;
    IATRetardTable.valueSize = SIZE_BYTE;
    IATRetardTable.xSize = 6;
    IATRetardTable.values = configPage4.iatRetValues;
    IATRetardTable.axisX = configPage4.iatRetBins;

    //Setup the calibration tables
    setPinMapping();

    initialise_schedule();
    initialiseFan();
    initialiseVVT();
    initialiseCorrections();
    initialiseADC();

    //Lookup the current MAP reading for barometric pressure
    readMAP();

    if ((currentStatus.MAP >= BARO_MIN) && (currentStatus.MAP <= BARO_MAX)) //Check if engine isn't running
    {
        currentStatus.baro = currentStatus.MAP;
        storeLastBaro(currentStatus.baro);
    }
    else
    {
        //Attempt to use the last known good baro reading from EEPROM
        if ((readLastBaro() >= BARO_MIN) && (readLastBaro() <= BARO_MAX)) //Make sure it's not invalid (Possible on first run etc)
        {
            currentStatus.baro = readLastBaro();    //last baro correction
        }
        else {
            currentStatus.baro = 100;    //Final fall back position.
        }
    }


    req_fuel_uS = configPage2.reqFuel * 100; //Convert to uS and an int. This is the only variable to be used in calculations
    inj_opentime_uS = configPage2.injOpen * 100; //Injector open time. Comes through as ms*10 (Eg 15.5ms = 155).
    currentStatus.RPM = 0;
    currentStatus.hasSync = false;
    currentStatus.runSecs = 0;
    currentStatus.secl = 0;
    currentStatus.startRevolutions = 0;
    currentStatus.syncLossCounter = 0;
    currentStatus.crankRPM = ((unsigned int)configPage4.crankRPM * 10); //Crank RPM limit (Saves us calculating this over and over again. It's updated once per second in timers.ino)
    currentStatus.fuelPumpOn = false;

    initialiseTriggers();

    previousLoopTime = 0;
    currentLoopTime = micros_safe();
    mainLoopCount = 0;

    initialisationComplete = true;
    digitalWrite(PIN_LED_GREEN, HIGH);
}

void setPinMapping()
{
    //Finally, set the relevant pin modes for outputs
    pinMode(PIN_IGN1, OUTPUT);
    pinMode(PIN_IGN2, OUTPUT);
    pinMode(PIN_IGN3, OUTPUT);
    pinMode(PIN_IGN4, OUTPUT);
    pinMode(PIN_INJ1, OUTPUT);
    pinMode(PIN_INJ2, OUTPUT);
    pinMode(PIN_INJ3, OUTPUT);
    pinMode(PIN_INJ4, OUTPUT);
    pinMode(PIN_FUEL, OUTPUT);
    pinMode(PIN_FAN, OUTPUT);
    pinMode(PIN_VVT, OUTPUT);

    inj1_pin_port = portOutputRegister(digitalPinToPort(PIN_INJ1));
    inj1_pin_mask = digitalPinToBitMask(PIN_INJ1);
    inj2_pin_port = portOutputRegister(digitalPinToPort(PIN_INJ2));
    inj2_pin_mask = digitalPinToBitMask(PIN_INJ2);
    inj3_pin_port = portOutputRegister(digitalPinToPort(PIN_INJ3));
    inj3_pin_mask = digitalPinToBitMask(PIN_INJ3);
    inj4_pin_port = portOutputRegister(digitalPinToPort(PIN_INJ4));
    inj4_pin_mask = digitalPinToBitMask(PIN_INJ4);

    ign1_pin_port = portOutputRegister(digitalPinToPort(PIN_IGN1));
    ign1_pin_mask = digitalPinToBitMask(PIN_IGN1);
    ign2_pin_port = portOutputRegister(digitalPinToPort(PIN_IGN2));
    ign2_pin_mask = digitalPinToBitMask(PIN_IGN2);
    ign3_pin_port = portOutputRegister(digitalPinToPort(PIN_IGN3));
    ign3_pin_mask = digitalPinToBitMask(PIN_IGN3);
    ign4_pin_port = portOutputRegister(digitalPinToPort(PIN_IGN4));
    ign4_pin_mask = digitalPinToBitMask(PIN_IGN4);

    pump_pin_port = portOutputRegister(digitalPinToPort(PIN_FUEL));
    pump_pin_mask = digitalPinToBitMask(PIN_FUEL);

    pinMode(PIN_CRANK, INPUT);
    pinMode(PIN_CAM, INPUT);

    //These must come after the above pinMode statements
    triggerPri_pin_port = portInputRegister(digitalPinToPort(PIN_CRANK));
    triggerPri_pin_mask = digitalPinToBitMask(PIN_CRANK);
    triggerSec_pin_port = portInputRegister(digitalPinToPort(PIN_CAM));
    triggerSec_pin_mask = digitalPinToBitMask(PIN_CAM);
}

void initialiseTriggers()
{
    byte triggerInterrupt = 0; // By default, use the first interrupt
    byte triggerInterrupt2 = 1;

    switch (PIN_CRANK) {
    //Arduino Mega 2560 mapping
    case 2:
        triggerInterrupt = 0;
        break;
    case 3:
        triggerInterrupt = 1;
        break;
    case 18:
        triggerInterrupt = 5;
        break;
    case 19:
        triggerInterrupt = 4;
        break;
    case 20:
        triggerInterrupt = 3;
        break;
    case 21:
        triggerInterrupt = 2;
        break;
    default:
        triggerInterrupt = 0;
        break; //This should NEVER happen
    }

    switch (PIN_CAM) {
    //Arduino Mega 2560 mapping
    case 2:
        triggerInterrupt2 = 0;
        break;
    case 3:
        triggerInterrupt2 = 1;
        break;
    case 18:
        triggerInterrupt2 = 5;
        break;
    case 19:
        triggerInterrupt2 = 4;
        break;
    case 20:
        triggerInterrupt2 = 3;
        break;
    case 21:
        triggerInterrupt2 = 2;
        break;
    default:
        triggerInterrupt2 = 0;
        break; //This should NEVER happen
    }

    pinMode(PIN_CRANK, INPUT);
    pinMode(PIN_CAM, INPUT);

    detachInterrupt(triggerInterrupt);
    detachInterrupt(triggerInterrupt2);

    attachInterrupt(triggerInterrupt, triggerCrank, CONFIG_CRANK_TRIGGER_EDGE);
    attachInterrupt(triggerInterrupt2, triggerCam, CONFIG_CAM_TRIGGER_EDGE);
}
