#ifndef GLOBALS_H
#define GLOBALS_H
#include <Arduino.h>
#include "table.h"
#include "board_avr2560.h"

//Handy bitsetting macros
#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
#define BIT_CHECK(var,pos) !!((var) & (1<<(pos)))

#define MS_IN_MINUTE 60000
#define US_IN_MINUTE 60000000

//Define the load algorithm
#define LOAD_SOURCE_MAP         0
#define LOAD_SOURCE_TPS         1
#define LOAD_SOURCE_IMAPEMAP    2

//Define bit positions within engine virable
#define BIT_ENGINE_RUN      0   // Engine running
#define BIT_ENGINE_CRANK    1   // Engine cranking
#define BIT_ENGINE_ASE      2   // after start enrichment (ASE)
#define BIT_ENGINE_WARMUP   3   // Engine in warmup
#define BIT_ENGINE_ACC      4   // in acceleration mode (TPS accel)
#define BIT_ENGINE_DCC      5   // in deceleration mode
#define BIT_ENGINE_MAPACC   6   // MAP acceleration mode
#define BIT_ENGINE_MAPDCC   7   // MAP decelleration mode

//Define masks for Status1
#define BIT_STATUS1_INJ1           0  //inj1
#define BIT_STATUS1_INJ2           1  //inj2
#define BIT_STATUS1_INJ3           2  //inj3
#define BIT_STATUS1_INJ4           3  //inj4
#define BIT_STATUS1_DFCO           4  //Decelleration fuel cutoff
#define BIT_STATUS1_BOOSTCUT       5  //Fuel component of MAP based boost cut out
#define BIT_STATUS1_TOOTHLOG1READY 6  //Used to flag if tooth log 1 is ready
#define BIT_STATUS1_TOOTHLOG2READY 7  //Used to flag if tooth log 2 is ready (Log is not currently used)

//Define masks for spark variable
#define BIT_SPARK_HLAUNCH         0  //Hard Launch indicator
#define BIT_SPARK_SLAUNCH         1  //Soft Launch indicator
#define BIT_SPARK_HRDLIM          2  //Hard limiter indicator
#define BIT_SPARK_SFTLIM          3  //Soft limiter indicator
#define BIT_SPARK_BOOSTCUT        4  //Spark component of MAP based boost cut out
#define BIT_SPARK_ERROR           5  // Error is detected
#define BIT_SPARK_IDLE            6  // idle on
#define BIT_SPARK_SYNC            7  // Whether engine has sync or not

#define BIT_SPARK2_FLATSH         0  //Flat shift hard cut
#define BIT_SPARK2_FLATSS         1  //Flat shift soft cut
#define BIT_SPARK2_UNUSED3        2
#define BIT_SPARK2_UNUSED4        3
#define BIT_SPARK2_UNUSED5        4
#define BIT_SPARK2_UNUSED6        5
#define BIT_SPARK2_UNUSED7        6
#define BIT_SPARK2_UNUSED8        7

#define BIT_STATUS3_RESET_PREVENT 0 //Indicates whether reset prevention is enabled
#define BIT_STATUS3_NITROUS       1
#define BIT_STATUS3_UNUSED2       2
#define BIT_STATUS3_UNUSED3       3
#define BIT_STATUS3_UNUSED4       4
#define BIT_STATUS3_NSQUIRTS1     5
#define BIT_STATUS3_NSQUIRTS2     6
#define BIT_STATUS3_NSQUIRTS3     7

#define TOOTH_LOG_SIZE      64
#define TOOTH_LOG_BUFFER    128 //256

#define COMPOSITE_LOG_PRI   0
#define COMPOSITE_LOG_SEC   1
#define COMPOSITE_LOG_TRIG  2
#define COMPOSITE_LOG_SYNC  3

#define SEC_TRIGGER_SINGLE  0
#define SEC_TRIGGER_4_1     1

#define BOOST_MODE_SIMPLE   0
#define BOOST_MODE_FULL     1

#define HARD_CUT_FULL       0
#define HARD_CUT_ROLLING    1

#define SIZE_BYTE           8
#define SIZE_INT            16

#define EVEN_FIRE           0
#define ODD_FIRE            1

#define EGO_ALGORITHM_SIMPLE  0
#define EGO_ALGORITHM_PID     2

#define STAGING_MODE_TABLE  0
#define STAGING_MODE_AUTO   1

#define KNOCK_MODE_OFF      0
#define KNOCK_MODE_DIGITAL  1
#define KNOCK_MODE_ANALOG   2

#define RESET_CONTROL_DISABLED             0
#define RESET_CONTROL_PREVENT_WHEN_RUNNING 1
#define RESET_CONTROL_PREVENT_ALWAYS       2
#define RESET_CONTROL_SERIAL_COMMAND       3

#define OPEN_LOOP_BOOST     0
#define CLOSED_LOOP_BOOST   1

#define MAX_RPM 18000 //This is the maximum rpm that the ECU will attempt to run at. It is NOT related to the rev limiter, but is instead dictates how fast certain operations will be allowed to run. Lower number gives better performance
#define engineSquirtsPerCycle 2 //Would be 1 for a 2 stroke

//Table sizes
#define OFFSET_FUELTRIM 127 //The fuel trim tables are offset by 128 to allow for -128 to +128 values
#define OFFSET_IGNITION 40 //Ignition values from the main spark table are offset 40 degrees downards to allow for negative spark timing

#define CALIBRATION_TEMPERATURE_OFFSET 40

#define SERIAL_BUFFER_THRESHOLD 32 // When the serial buffer is filled to greater than this threshold value, the serial processing operations will be performed more urgently in order to avoid it overflowing. Serial buffer is 64 bytes long, so the threshold is set at half this as a reasonable figure

#define FUEL_PUMP_ON() *pump_pin_port |= (pump_pin_mask)
#define FUEL_PUMP_OFF() *pump_pin_port &= ~(pump_pin_mask)

const char TSfirmwareVersion[] PROGMEM = "Speeduino";

const byte data_structure_version = 2; //This identifies the data structure when reading / writing.
//const byte page_size = 64;
//const int16_t npage_size[11] PROGMEM = {0,288,128,288,128,288,128,240,192,192,192};
#define NUM_PAGES     11
const uint16_t npage_size[NUM_PAGES] PROGMEM = {0,128,288,288,128,288,128,240,192,192,192};
//const byte page11_size = 128;
#define MAP_PAGE_SIZE 288

struct table3D fuelTable; //16x16 fuel map
struct table3D ignitionTable; //16x16 ignition map
struct table3D afrTable; //16x16 afr target map
struct table3D stagingTable; //8x8 fuel staging table
struct table3D boostTable; //8x8 boost map
struct table3D vvtTable; //8x8 vvt map
struct table3D trim1Table; //6x6 Fuel trim 1 map
struct table3D trim2Table; //6x6 Fuel trim 2 map
struct table3D trim3Table; //6x6 Fuel trim 3 map
struct table3D trim4Table; //6x6 Fuel trim 4 map
struct table2D taeTable; //4 bin TPS Acceleration Enrichment map (2D)
struct table2D WUETable; //10 bin Warm Up Enrichment map (2D)
struct table2D crankingEnrichTable; //4 bin cranking Enrichment map (2D)
struct table2D dwellVCorrectionTable; //6 bin dwell voltage correction (2D)
struct table2D injectorVCorrectionTable; //6 bin injector voltage correction (2D)
struct table2D IATDensityCorrectionTable; //9 bin inlet air temperature density correction (2D)
struct table2D IATRetardTable; //6 bin ignition adjustment based on inlet air temperature  (2D)
struct table2D rotarySplitTable; //8 bin ignition split curve for rotary leading/trailing  (2D)
struct table2D flexFuelTable;  //6 bin flex fuel correction table for fuel adjustments (2D)
struct table2D flexAdvTable;   //6 bin flex fuel correction table for timing advance (2D)
struct table2D flexBoostTable; //6 bin flex fuel correction table for boost adjustments (2D)
struct table2D knockWindowStartTable;
struct table2D knockWindowDurationTable;

int ignition1EndAngle = 0;
int ignition2EndAngle = 0;
int ignition3EndAngle = 0;
int ignition4EndAngle = 0;

//These are for the direct port manipulation of the injectors, coils and aux outputs
volatile uint8_t *inj1_pin_port;
volatile uint8_t inj1_pin_mask;
volatile uint8_t *inj2_pin_port;
volatile uint8_t inj2_pin_mask;
volatile uint8_t *inj3_pin_port;
volatile uint8_t inj3_pin_mask;
volatile uint8_t *inj4_pin_port;
volatile uint8_t inj4_pin_mask;

volatile uint8_t *ign1_pin_port;
volatile uint8_t ign1_pin_mask;
volatile uint8_t *ign2_pin_port;
volatile uint8_t ign2_pin_mask;
volatile uint8_t *ign3_pin_port;
volatile uint8_t ign3_pin_mask;
volatile uint8_t *ign4_pin_port;
volatile uint8_t ign4_pin_mask;

volatile uint8_t *tach_pin_port;
volatile uint8_t tach_pin_mask;
volatile uint8_t *pump_pin_port;
volatile uint8_t pump_pin_mask;

volatile uint8_t *triggerPri_pin_port;
volatile uint8_t triggerPri_pin_mask;
volatile uint8_t *triggerSec_pin_port;
volatile uint8_t triggerSec_pin_mask;

//These are variables used across multiple files
bool initialisationComplete = false; //Tracks whether the setup() function has run completely
byte fpPrimeTime = 0; //The time (in seconds, based on currentStatus.secl) that the fuel pump started priming
volatile uint16_t mainLoopCount;
volatile unsigned long ms_counter = 0; //A counter that increments once per ms
uint16_t fixedCrankingOverride = 0;
volatile bool fpPrimed = false; //Tracks whether or not the fuel pump priming has been completed yet

volatile byte LOOP_TIMER;

//The status struct contains the current values for all 'live' variables
//In current version this is 64 bytes
struct statuses {
    volatile bool hasSync;
    uint16_t RPM;
    long longRPM;
    int mapADC;
    int baroADC;
    long MAP; //Has to be a long for PID calcs (Boost control)
    int16_t EMAP;
    int16_t EMAPADC;
    byte baro; //Barometric pressure is simply the inital MAP reading, taken before the engine is running. Alternatively, can be taken from an external sensor
    byte TPS; //The current TPS reading (0% - 100%)
    byte TPSlast; //The previous TPS reading
    unsigned long TPS_time; //The time the TPS sample was taken
    unsigned long TPSlast_time; //The time the previous TPS sample was taken
    byte tpsADC; //0-255 byte representation of the TPS
    byte tpsDOT;
    volatile int rpmDOT;
    byte VE;
    byte O2;
    byte O2_2;
    int coolant;
    int cltADC;
    int IAT;
    int iatADC;
    int batADC;
    int O2ADC;
    int O2_2ADC;
    int dwell;
    byte dwellCorrection; //The amount of correction being applied to the dwell time.
    byte battery10; //The current BRV in volts (multiplied by 10. Eg 12.5V = 125)
    int8_t advance; //Signed 8 bit as advance can now go negative (ATDC)
    byte corrections;
    int16_t TAEamount; //The amount of accleration enrichment currently being applied
    byte egoCorrection; //The amount of closed loop AFR enrichment currently being applied
    byte wueCorrection; //The amount of warmup enrichment currently being applied
    byte batCorrection; //The amount of battery voltage enrichment currently being applied
    byte iatCorrection; //The amount of inlet air temperature adjustment currently being applied
    byte launchCorrection; //The amount of correction being applied if launch control is active
    byte flexCorrection; //Amount of correction being applied to compensate for ethanol content
    int8_t flexIgnCorrection; //Amount of additional advance being applied based on flex. Note the type as this allows for negative values
    byte afrTarget;
    byte idleDuty;
    bool idleUpActive;
    bool fanOn; //Whether or not the fan is turned on
    volatile byte ethanolPct; //Ethanol reading (if enabled). 0 = No ethanol, 100 = pure ethanol. Eg E85 = 85.
    unsigned long TAEEndTime; //The target end time used whenever TAE is turned on
    volatile byte status1;
    volatile byte spark;
    volatile byte spark2;
    byte engine;
    unsigned int PW1; //In uS
    unsigned int PW2; //In uS
    unsigned int PW3; //In uS
    unsigned int PW4; //In uS
    unsigned int PW5; //In uS
    unsigned int PW6; //In uS
    unsigned int PW7; //In uS
    unsigned int PW8; //In uS
    volatile byte runSecs; //Counter of seconds since cranking commenced (overflows at 255 obviously)
    volatile byte secl; //Continous
    volatile unsigned int loopsPerSecond;
    bool launchingSoft; //True when in launch control soft limit mode
    bool launchingHard; //True when in launch control hard limit mode
    uint16_t freeRAM;
    unsigned int clutchEngagedRPM;
    bool flatShiftingHard;
    volatile uint32_t startRevolutions; //A counter for how many revolutions have been completed since sync was achieved.
    uint16_t boostTarget;
    byte testOutputs;
    bool testActive;
    uint16_t boostDuty; //Percentage value * 100 to give 2 points of precision
    byte idleLoad; //Either the current steps or current duty cycle for the idle control.
    uint16_t canin[16];   //16bit raw value of selected canin data for channel 0-15
    uint8_t current_caninchannel = 0; //start off at channel 0
    uint16_t crankRPM = 400; //The actual cranking RPM limit. Saves us multiplying it everytime from the config page
    volatile byte status3;
    int16_t flexBoostCorrection; //Amount of boost added based on flex
    byte nitrous_status;
    byte nSquirts;
    byte nChannels; //Number of fuel and ignition channels
    int16_t fuelLoad;
    int16_t ignLoad;
    bool fuelPumpOn; //The current status of the fuel pump
    byte syncLossCounter;
    byte knockRetard;
    bool knockActive;
    bool toothLogEnabled;
    bool compositeLogEnabled;

    //Helpful bitwise operations:
    //Useful reference: http://playground.arduino.cc/Code/BitMath
    // y = (x >> n) & 1;    // n=0..15.  stores nth bit of x in y.  y becomes 0 or 1.
    // x &= ~(1 << n);      // forces nth bit of x to be 0.  all other bits left alone.
    // x |= (1 << n);       // forces nth bit of x to be 1.  all other bits left alone.

};
struct statuses currentStatus; //The global status object

struct config2 {

    byte unused2_1;
    byte unused2_2;
    byte asePct;  //Afterstart enrichment (%)
    byte aseCount; //Afterstart enrichment cycles. This is the number of ignition cycles that the afterstart enrichment % lasts for
    byte wueValues[10]; //Warm up enrichment array (10 bytes)
    byte crankingPct; //Cranking enrichment
    byte pinMapping; // The board / ping mapping to be used
    byte tachoPin : 6; //Custom pin setting for tacho output
    byte tachoDiv : 2; //Whether to change the tacho speed
    byte tachoDuration; //The duration of the tacho pulse in mS
    byte unused2_18;
    byte tpsThresh;
    byte taeTime;

    //Display config bits
    byte displayType : 3; //21
    byte display1 : 3;
    byte display2 : 2;

    byte display3 : 3;    //22
    byte display4 : 2;
    byte display5 : 3;

    byte displayB1 : 4;   //23
    byte displayB2 : 4;

    byte reqFuel;       //24
    byte divider;
    byte injTiming : 1;
    byte multiplyMAP : 1;
    byte includeAFR : 1;
    byte hardCutType : 1;
    byte ignAlgorithm : 3;
    byte indInjAng : 1;
    byte injOpen; //Injector opening time (ms * 10)
    uint16_t inj1Ang;
    uint16_t inj2Ang;
    uint16_t inj3Ang;
    uint16_t inj4Ang;

    //config1 in ini
    byte mapSample : 2;
    byte strokes : 1;
    byte injType : 1;
    byte nCylinders : 4; //Number of cylinders

    //config2 in ini
    byte fuelAlgorithm : 3;
    byte fixAngEnable : 1; //Whether fixed/locked timing is enabled
    byte nInjectors : 4; //Number of injectors


    //config3 in ini
    byte engineType : 1;
    byte flexEnabled : 1;
    byte unused2_38c : 1; //"Speed Density", "Alpha-N"
    byte baroCorr : 1;
    byte injLayout : 2;
    byte perToothIgn : 1;
    byte dfcoEnabled : 1; //Whether or not DFCO is turned on

    byte primePulse;
    byte dutyLim;
    byte flexFreqLow; //Lowest valid frequency reading from the flex sensor
    byte flexFreqHigh; //Highest valid frequency reading from the flex sensor

    byte boostMaxDuty;
    byte tpsMin;
    byte tpsMax;
    int8_t mapMin; //Must be signed
    uint16_t mapMax;
    byte fpPrime; //Time (In seconds) that the fuel pump should be primed for on power up
    byte stoich;
    uint16_t oddfire2; //The ATDC angle of channel 2 for oddfire
    uint16_t oddfire3; //The ATDC angle of channel 3 for oddfire
    uint16_t oddfire4; //The ATDC angle of channel 4 for oddfire

    byte idleUpPin : 6;
    byte idleUpPolarity : 1;
    byte idleUpEnabled : 1;

    byte idleUpAdder;
    byte taeTaperMin;
    byte taeTaperMax;

    byte iacCLminDuty;
    byte iacCLmaxDuty;
    byte boostMinDuty;

    int8_t baroMin; //Must be signed
    uint16_t baroMax;

    int8_t EMAPMin; //Must be signed
    uint16_t EMAPMax;

    byte fanWhenOff : 1;      // Only run fan when engine is running
    byte fanUnused : 7;

    byte unused1_70[57];
};

//Page 4 of the config - See the ini file for further reference
//This mostly covers off variables that are required for ignition
struct config4 {

    int16_t triggerAngle;
    int8_t FixAng; //Negative values allowed
    byte CrankAng;
    byte TrigAngMul; //Multiplier for non evenly divisible tooth counts.

    byte TrigEdge : 1;
    byte TrigSpeed : 1;
    byte IgInv : 1;
    byte TrigPattern : 5;

    byte TrigEdgeSec : 1;
    byte fuelPumpPin : 6;
    byte useResync : 1;

    byte sparkDur; //Spark duration in ms * 10
    byte trigPatternSec; //Mode for Missing tooth secondary trigger.  Either single tooth cam wheel or 4-1
    uint8_t bootloaderCaps; //Capabilities of the bootloader over stock. e.g., 0=Stock, 1=Reset protection, etc.

    byte resetControl : 2; //Which method of reset control to use (0=None, 1=Prevent When Running, 2=Prevent Always, 3=Serial Command)
    byte resetControlPin : 6;

    byte StgCycles=1; //The number of initial cycles before the ignition should fire when first cranking

    byte boostType : 1; //Open or closed loop boost control
    byte useDwellLim : 1; //Whether the dwell limiter is off or on
    byte sparkMode : 3; //Spark output mode (Eg Wasted spark, single channel or Wasted COP)
    byte triggerFilter : 2; //The mode of trigger filter being used (0=Off, 1=Light (Not currently used), 2=Normal, 3=Aggressive)
    byte ignCranklock : 1; //Whether or not the ignition timing during cranking is locked to a CAS pulse. Only currently valid for Basic distributor and 4G63.

    byte dwellCrank; //Dwell time whilst cranking
    byte dwellRun; //Dwell time whilst running
    byte triggerTeeth=36; //The full count of teeth on the trigger wheel if there were no gaps
    byte triggerMissingTeeth=2; //The size of the tooth gap (ie number of missing teeth)
    byte crankRPM; //RPM below which the engine is considered to be cranking
    byte floodClear; //TPS value that triggers flood clear mode (No fuel whilst cranking)
    byte SoftRevLim; //Soft rev limit (RPM/100)
    byte SoftLimRetard; //Amount soft limit retards (degrees)
    byte SoftLimMax; //Time the soft limit can run
    byte HardRevLim; //Hard rev limit (RPM/100)
    byte taeBins[4]; //TPS based acceleration enrichment bins (%/s)
    byte taeValues[4]; //TPS based acceleration enrichment rates (% to add)
    byte wueBins[10]; //Warmup Enrichment bins (Values are in configTable1)
    byte dwellLimit;
    byte dwellCorrectionValues[6]; //Correction table for dwell vs battery voltage
    byte iatRetBins[6]; // Inlet Air Temp timing retard curve bins
    byte iatRetValues[6]; // Inlet Air Temp timing retard curve values
    byte dfcoRPM; //RPM at which DFCO turns off/on at
    byte dfcoHyster; //Hysteris RPM for DFCO
    byte dfcoTPSThresh; //TPS must be below this figure for DFCO to engage

    byte ignBypassEnabled : 1; //Whether or not the ignition bypass is enabled
    byte ignBypassPin : 6; //Pin the ignition bypass is activated on
    byte ignBypassHiLo : 1; //Whether this should be active high or low.

    byte ADCFILTER_TPS;
    byte ADCFILTER_CLT;
    byte ADCFILTER_IAT;
    byte ADCFILTER_O2;
    byte ADCFILTER_BAT;
    byte ADCFILTER_MAP; //This is only used on Instantaneous MAP readings and is intentionally very weak to allow for faster response
    byte ADCFILTER_BARO;

    byte unused2_64[57];
};

//Page 6 of the config - See the ini file for further reference
//This mostly covers off variables that are required for AFR targets and closed loop
struct config6 {

    byte egoAlgorithm : 2;
    byte egoType : 2;
    byte boostEnabled : 1;
    byte vvtEnabled : 1;
    byte boostCutType : 2;

    byte egoKP;
    byte egoKI;
    byte egoKD;
    byte egoTemp; //The temperature above which closed loop functions
    byte egoCount; //The number of ignition cylces per step
    byte unused6_6;
    byte egoLimit; //Maximum amount the closed loop will vary the fueling
    byte ego_min; //AFR must be above this for closed loop to function
    byte ego_max; //AFR must be below this for closed loop to function
    byte ego_sdelay; //Time in seconds after engine starts that closed loop becomes available
    byte egoRPM; //RPM must be above this for closed loop to function
    byte egoTPSMax; //TPS must be below this for closed loop to function
    byte vvtPin : 6;
    byte useExtBaro : 1;
    byte boostMode : 1; //Simple of full boost control
    byte boostPin : 6;
    byte VVTasOnOff : 1; //Whether or not to use the VVT table as an on/off map
    byte useEMAP : 1;
    byte voltageCorrectionBins[6]; //X axis bins for voltage correction tables
    byte injVoltageCorrectionValues[6]; //Correction table for injector PW vs battery voltage
    byte airDenBins[9];
    byte airDenRates[9];
    byte boostFreq; //Frequency of the boost PWM valve
    byte vvtFreq; //Frequency of the vvt PWM valve
    byte idleFreq;

    byte launchPin : 6;
    byte launchEnabled : 1;
    byte launchHiLo : 1;

    byte lnchSoftLim;
    int8_t lnchRetard; //Allow for negative advance value (ATDC)
    byte lnchHardLim;
    byte lnchFuelAdd;

    //PID values for idle needed to go here as out of room in the idle page
    byte idleKP;
    byte idleKI;
    byte idleKD;

    byte boostLimit; //Is divided by 2, allowing kPa values up to 511
    byte boostKP;
    byte boostKI;
    byte boostKD;

    byte lnchPullRes : 2;
    byte fuelTrimEnabled : 1;
    byte flatSEnable : 1;
    byte baroPin : 4;
    byte flatSSoftWin;
    byte flatSRetard;
    byte flatSArm;

    byte iacCLValues[10]; //Closed loop target RPM value
    byte iacOLStepVal[10]; //Open loop step values for stepper motors
    byte iacOLPWMVal[10]; //Open loop duty values for PMWM valves
    byte iacBins[10]; //Temperature Bins for the above 3 curves
    byte iacCrankSteps[4]; //Steps to use when cranking (Stepper motor)
    byte iacCrankDuty[4]; //Duty cycle to use on PWM valves when cranking
    byte iacCrankBins[4]; //Temperature Bins for the above 2 curves

    byte iacAlgorithm : 3; //Valid values are: "None", "On/Off", "PWM", "PWM Closed Loop", "Stepper", "Stepper Closed Loop"
    byte iacStepTime : 3; //How long to pulse the stepper for to ensure the step completes (ms)
    byte iacChannels : 1; //How many outputs to use in PWM mode (0 = 1 channel, 1 = 2 channels)
    byte iacPWMdir : 1; //Direction of the PWM valve. 0 = Normal = Higher RPM with more duty. 1 = Reverse = Lower RPM with more duty

    byte iacFastTemp; //Fast idle temp when using a simple on/off valve

    byte iacStepHome; //When using a stepper motor, the number of steps to be taken on startup to home the motor
    byte iacStepHyster; //Hysteresis temperature (*10). Eg 2.2C = 22

    byte fanInv : 1;        // Fan output inversion bit
    byte fanEnable : 1;     // Fan enable bit. 0=Off, 1=On/Off
    byte fanPin : 6;
    byte fanSP;             // Cooling fan start temperature
    byte fanHyster;         // Fan hysteresis
    byte fanFreq;           // Fan PWM frequency
    byte fanPWMBins[4];     //Temperature Bins for the PWM fan control
};

//Page 9 of the config mostly deals with CANBUS control
//See ini file for further info (Config Page 10 in the ini)
struct config9 {
    byte enable_secondarySerial:1;            //enable secondary serial
    byte intcan_available:1;                     //enable internal can module
    byte enable_intcan:1;
    byte caninput_sel[16];                    //bit status on/Can/analog_local/digtal_local if input is enabled
    uint16_t caninput_source_can_address[16];        //u16 [15] array holding can address of input
    uint8_t caninput_source_start_byte[16];     //u08 [15] array holds the start byte number(value of 0-7)
    uint16_t caninput_source_num_bytes;     //u16 bit status of the number of bytes length 1 or 2
    byte unused10_67;
    byte unused10_68;
    byte enable_candata_out : 1;
    byte canoutput_sel[8];
    uint16_t canoutput_param_group[8];
    uint8_t canoutput_param_start_byte[8];
    byte canoutput_param_num_bytes[8];

    byte unused10_110;
    byte unused10_111;
    byte unused10_112;
    byte unused10_113;
    byte speeduino_tsCanId:4;         //speeduino TS canid (0-14)
    uint16_t true_address;            //speeduino 11bit can address
    uint16_t realtime_base_address;   //speeduino 11 bit realtime base address
    uint16_t obd_address;             //speeduino OBD diagnostic address
    uint8_t Auxinpina[16];            //analog  pin number when internal aux in use
    uint8_t Auxinpinb[16];            // digital pin number when internal aux in use

    byte iacStepperInv : 1;  //stepper direction of travel to allow reversing. 0=normal, 1=inverted.
    byte iacCoolTime : 3; // how long to wait for the stepper to cool between steps

    byte unused10_154;
    byte unused10_155;
    byte unused10_156;
    byte unused10_157;
    byte unused10_158;
    byte unused10_159;
    byte unused10_160;
    byte unused10_161;
    byte unused10_162;
    byte unused10_163;
    byte unused10_164;
    byte unused10_165;
    byte unused10_166;
    byte unused10_167;
    byte unused10_168;
    byte unused10_169;
    byte unused10_170;
    byte unused10_171;
    byte unused10_172;
    byte unused10_173;
    byte unused10_174;
    byte unused10_175;
    byte unused10_176;
    byte unused10_177;
    byte unused10_178;
    byte unused10_179;
    byte unused10_180;
    byte unused10_181;
    byte unused10_182;
    byte unused10_183;
    byte unused10_184;
    byte unused10_185;
    byte unused10_186;
    byte unused10_187;
    byte unused10_188;
    byte unused10_189;
    byte unused10_190;
    byte unused10_191;
};

/*
Page 10 - No specific purpose. Created initially for the cranking enrich curve
192 bytes long
See ini file for further info (Config Page 11 in the ini)
*/
struct config10 {
    byte crankingEnrichBins[4];
    byte crankingEnrichValues[4];

    byte rotaryType : 2;
    byte stagingEnabled : 1;
    byte stagingMode : 1;
    byte EMAPPin : 4;

    byte rotarySplitValues[8];
    byte rotarySplitBins[8];

    uint16_t boostSens;
    byte boostIntv;
    uint16_t stagedInjSizePri;
    uint16_t stagedInjSizeSec;
    byte lnchCtrlTPS;

    uint8_t flexBoostBins[6];
    int16_t flexBoostAdj[6];  //kPa to be added to the boost target @ current ethanol (negative values allowed)
    uint8_t flexFuelBins[6];
    uint8_t flexFuelAdj[6];   //Fuel % @ current ethanol (typically 100% @ 0%, 163% @ 100%)
    uint8_t flexAdvBins[6];
    uint8_t  flexAdvAdj[6];    //Additional advance (in degrees) @ current ethanol (typically 0 @ 0%, 10-20 @ 100%). NOTE: THIS IS A SIGNED VALUE!
    //And another three corn rows die.

    byte n2o_enable : 2;
    byte n2o_arming_pin : 6;
    byte n2o_minCLT;
    byte n2o_maxMAP;
    byte n2o_minTPS;
    byte n2o_maxAFR;

    byte n2o_stage1_pin : 6;
    byte n2o_pin_polarity : 1;
    byte n2o_stage1_unused : 1;
    byte n2o_stage1_minRPM;
    byte n2o_stage1_maxRPM;
    byte n2o_stage1_adderMin;
    byte n2o_stage1_adderMax;
    byte n2o_stage1_retard;

    byte n2o_stage2_pin : 6;
    byte n2o_stage2_unused : 2;
    byte n2o_stage2_minRPM;
    byte n2o_stage2_maxRPM;
    byte n2o_stage2_adderMin;
    byte n2o_stage2_adderMax;
    byte n2o_stage2_retard;

    byte knock_mode : 2;
    byte knock_pin : 6;

    byte knock_trigger : 1;
    byte knock_pullup : 1;
    byte knock_limiterDisable : 1;
    byte knock_unused : 2;
    byte knock_count : 3;

    byte knock_threshold;
    byte knock_maxMAP;
    byte knock_maxRPM;
    byte knock_window_rpms[6];
    byte knock_window_angle[6];
    byte knock_window_dur[6];

    byte knock_maxRetard;
    byte knock_firstStep;
    byte knock_stepSize;
    byte knock_stepTime;

    byte knock_duration; //Time after knock retard starts that it should start recovering
    byte knock_recoveryStepTime;
    byte knock_recoveryStep;

    byte unused11_122_191[70];
};

// global variables // from speeduino.ino
extern struct statuses currentStatus; // from speeduino.ino
extern struct table3D fuelTable; //16x16 fuel map
extern struct table3D ignitionTable; //16x16 ignition map
extern struct table3D afrTable; //16x16 afr target map
extern struct table3D stagingTable; //8x8 afr target map
extern struct table2D taeTable; //4 bin TPS Acceleration Enrichment map (2D)
extern struct table2D WUETable; //10 bin Warm Up Enrichment map (2D)
extern struct table2D crankingEnrichTable; //4 bin cranking Enrichment map (2D)
extern struct config2 configPage2;
extern struct config4 configPage4;
extern struct config6 configPage6;
extern struct config9 configPage9;
extern struct config10 configPage10;
extern unsigned long currentLoopTime; //The time the current loop started (uS)
extern unsigned long previousLoopTime; //The time the previous loop started (uS)
volatile uint16_t ignitionCount; //The count of ignition events that have taken place since the engine started
#endif // GLOBALS_H
