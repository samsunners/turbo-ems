/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/*
The corrections functions in this file affect the fuel pulsewidth (Either increasing or decreasing)
based on factors other than the VE lookup.

These factors include temperature (Warmup Enrichment and After Start Enrichment), Acceleration/Decelleration,
Flood clear mode etc.
*/
//************************************************************************************************************

#include "globals.h"
#include "corrections.h"
#include "maths.h"
#include "sensors.h"
#include "src/PID_v1/PID_v1.h"

long PID_O2, PID_output, PID_AFRTarget;
PID egoPID(&PID_O2, &PID_output, &PID_AFRTarget, configPage6.egoKP, configPage6.egoKI, configPage6.egoKD, REVERSE); //This is the PID object if that algorithm is used. Needs to be global as it maintains state outside of each function call

void initialiseCorrections()
{
    egoPID.SetMode(AUTOMATIC); //Turn O2 PID on
    currentStatus.egoCorrection = 100; //Default value of no adjustment must be set to avoid randomness on first correction cycle after startup
    AFRnextCycle = 0;
    currentStatus.knockActive = false;
}

/*
correctionsTotal() calls all the other corrections functions and combines their results.
This is the only function that should be called from anywhere outside the file
*/
inline void correctionsFuel()
{
    unsigned long product = 100;

//    currentStatus.wueCorrection = correctionWUE();
//    product*=currentStatus.wueCorrection;
//    product/=100;

    product*=correctionASE();
    product/=100;

    product*=correctionCranking();
    product/=100;

//    currentStatus.TAEamount = correctionAccel();
//    product*=currentStatus.TAEamount;
//    product/=100;

//    currentStatus.egoCorrection = correctionAFRClosedLoop();
//    product*=currentStatus.egoCorrection;
//    product/=100;

//    currentStatus.iatCorrection = correctionIATDensity();
//    product*=currentStatus.iatCorrection;
//    product/=100;

    product*=correctionFloodClear();
    product/=100;

    correctionDFCO();
    if ( bitRead(currentStatus.status1, BIT_STATUS1_DFCO) == 1 ) {
        product = 0;
    }

    if(product > 200) {
        product = 200;//This is the maximum allowable increase
    }

    currentStatus.corrections=product;
}

/*
Warm Up Enrichment (WUE)
Uses a 2D enrichment table (WUETable) where the X axis is engine temp and the Y axis is the amount of extra fuel to add
*/
static inline byte correctionWUE()
{
    byte WUEValue;
    //Possibly reduce the frequency this runs at (Costs about 50 loops per second)
    if (currentStatus.coolant > (WUETable.axisX[9] - CALIBRATION_TEMPERATURE_OFFSET))
    {
        //This prevents us doing the 2D lookup if we're already up to temp
        BIT_CLEAR(currentStatus.engine, BIT_ENGINE_WARMUP);
        WUEValue = WUETable.values[9]; //Set the current value to be whatever the final value on the curve is.
    }
    else
    {
        BIT_SET(currentStatus.engine, BIT_ENGINE_WARMUP);
        WUEValue = table2D_getValue(&WUETable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
    }

    return WUEValue;
}

/*
Cranking Enrichment
Additional fuel % to be added when the engine is cranking
*/
static inline byte correctionCranking()
{
    if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) )
    {
        return CONFIG_CRANKING_ENRICHMENT;
    };
    return 100;
}

/*
After Start Enrichment
This is a short period (Usually <20 seconds) immediately after the engine first fires (But not when cranking)
where an additional amount of fuel is added (Over and above the WUE amount)
*/
static inline byte correctionASE()
{
    if ( (currentStatus.runSecs < CONFIG_ASE_SECONDS) && !(BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) )
    {
        BIT_SET(currentStatus.engine, BIT_ENGINE_ASE); //Mark ASE as active.
        return CONFIG_ASE_CORRECTION;
    }

    BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ASE); //Mark ASE as inactive.
    return 100;
}

static inline int16_t correctionAccel()
{

}

static inline byte correctionFloodClear()
{
    if( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) && (currentStatus.TPS >= CONFIG_FLOOD_CLEAR_TPS) )return 0;
    return 100;
}

static inline byte correctionIATDensity()
{
    byte IATValue = 100;
    if ( (currentStatus.IAT + CALIBRATION_TEMPERATURE_OFFSET) > (IATDensityCorrectionTable.axisX[8])) {
        IATValue = IATDensityCorrectionTable.values[IATDensityCorrectionTable.xSize-1];    //This prevents us doing the 2D lookup if the intake temp is above maximum
    }
    else {
        IATValue = table2D_getValue(&IATDensityCorrectionTable, currentStatus.IAT + CALIBRATION_TEMPERATURE_OFFSET);    //currentStatus.IAT is the actual temperature, values in IATDensityCorrectionTable.axisX are temp+offset
    }

    return IATValue;
}

static inline void correctionDFCO()
{
    if ( 	(bitRead(currentStatus.status1, BIT_STATUS1_DFCO) == 0) && 
			(currentStatus.RPM > CONFIG_DFCO_RPM_THRESH) && 
			(currentStatus.TPS < CONFIG_DFCO_TPS_THRESH) ) 
	{
		bitSet(currentStatus.status1, BIT_STATUS1_DFCO);
    }
    else if(	(bitRead(currentStatus.status1, BIT_STATUS1_DFCO) == 1) &&
				( (currentStatus.RPM < CONFIG_DFCO_RPM_THRESH-CONFIG_DFCO_RPM_HYS) || (currentStatus.TPS > CONFIG_DFCO_TPS_THRESH)) )
	{
		bitClear(currentStatus.status1, BIT_STATUS1_DFCO);
    }
}

/*
Lookup the AFR target table and perform either a simple or PID adjustment based on this

Simple (Best suited to narrowband sensors):
If the O2 sensor reports that the mixture is lean/rich compared to the desired AFR target, it will make a 1% adjustment
It then waits <egoDelta> number of ignition events and compares O2 against the target table again. If it is still lean/rich then the adjustment is increased to 2%
This continues until either:
  a) the O2 reading flips from lean to rich, at which point the adjustment cycle starts again at 1% or
  b) the adjustment amount increases to <egoLimit> at which point it stays at this level until the O2 state (rich/lean) changes

PID (Best suited to wideband sensors):

*/

static inline byte correctionAFRClosedLoop()
{
        currentStatus.afrTarget = currentStatus.O2; //Catch all incase the below doesn't run. This prevents the Include AFR option from doing crazy things if the AFR target conditions aren't met. This value is changed again below if all conditions are met.

        //Determine whether the Y axis of the AFR target table tshould be MAP (Speed-Density) or TPS (Alpha-N)
        //Note that this should only run after the sensor warmup delay necause it is used within the Include AFR option
        if(currentStatus.runSecs > configPage6.ego_sdelay) {
            currentStatus.afrTarget = get3DTableValue(&afrTable, currentStatus.fuelLoad, currentStatus.RPM);    //Perform the target lookup
        }

        //Check all other requirements for closed loop adjustments
        if( (currentStatus.coolant > (int)(configPage6.egoTemp - CALIBRATION_TEMPERATURE_OFFSET)) && (currentStatus.RPM > (unsigned int)(configPage6.egoRPM * 100)) && (currentStatus.TPS < configPage6.egoTPSMax) && (currentStatus.O2 < configPage6.ego_max) && (currentStatus.O2 > configPage6.ego_min) && (currentStatus.runSecs > configPage6.ego_sdelay) )
        {
//            AFRValue = currentStatus.egoCorrection; //Need to record this here, just to make sure the correction stays 'on' even if the nextCycle count isn't ready

            if(ignitionCount >= AFRnextCycle)
            {
                AFRnextCycle = ignitionCount + configPage6.egoCount; //Set the target ignition event for the next calculation

                    //*************************************************************************************************************************************
                    //PID algorithm
                    egoPID.SetOutputLimits((long)(-configPage6.egoLimit), (long)(configPage6.egoLimit)); //Set the limits again, just incase the user has changed them since the last loop. Note that these are sent to the PID library as (Eg:) -15 and +15
                    egoPID.SetTunings(configPage6.egoKP, configPage6.egoKI, configPage6.egoKD); //Set the PID values again, just incase the user has changed them since the last loop
                    PID_O2 = (long)(currentStatus.O2);
                    PID_AFRTarget = (long)(currentStatus.afrTarget);

                    egoPID.Compute();
                    //currentStatus.egoCorrection = 100 + PID_output;
//                    AFRValue = 100 + PID_output;
            } //Ignition count check
        } //Multi variable check
//    return AFRValue; //Catch all (Includes when AFR target = current AFR
}

//******************************** IGNITION ADVANCE CORRECTIONS ********************************

int8_t correctionsIgn(int8_t advance)
{
//    advance = correctionIATretard(advance);
//    advance = correctionKnock(advance);
//    advance = correctionFixedTiming(advance);
    advance = correctionCrankingFixedTiming(advance); //This overrrides the regular fixed timing, must come last

    return advance;
}

static inline int8_t correctionFixedTiming(int8_t advance)
{
    if (configPage2.fixAngEnable == 1) {
        return configPage4.FixAng;    //Check whether the user has set a fixed timing angle
    };
    return advance;
}

static inline int8_t correctionCrankingFixedTiming(int8_t advance)
{
    if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) ) {
        return CONFIG_CRANKING_FIXED_TIMING;    //Use the fixed cranking ignition angle
    };
    return advance;
}

static inline int8_t correctionIATretard(int8_t advance)
{
	//TODO: Finish
    int16_t tempAdvance=advance-table2D_getValue(&IATRetardTable, currentStatus.IAT);
    if(tempAdvance<0)tempAdvance=0;//we probably never need to retard ignition below 0 degrees
    return (int8_t)tempAdvance;
}

static inline int8_t correctionKnock(int8_t advance)
{
    int16_t tempAdvance = advance;
    //TODO: Finish this.
    return (int8_t)tempAdvance;
}
