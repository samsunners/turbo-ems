//1-3-4-2 firing order
#define CONFIG_CYLINDER1_TDC 0
#define CONFIG_CYLINDER2_TDC 49152//540 degrees
#define CONFIG_CYLINDER3_TDC 16384//180 degrees
#define CONFIG_CYLINDER4_TDC 32768//360 degrees

#define CONFIG_INJ1_START_ANGLE 31858
#define CONFIG_INJ2_START_ANGLE 15474
#define CONFIG_INJ3_START_ANGLE 48242
#define CONFIG_INJ4_START_ANGLE 64626

#define PIN_O2 A3
#define PIN_CLT A4
#define PIN_CRANKING A8
#define PIN_IAT A6

#define PIN_IGN1 37
#define PIN_IGN2 36
#define PIN_IGN3 35
#define PIN_IGN4 34
#define PIN_INJ1 33
#define PIN_INJ2 32
#define PIN_INJ3 31
#define PIN_INJ4 30

#define CONFIG_DWELL 3000 //dwell time in microseconds

//throttle position sensor stuff
#define PIN_TPS A5
#define CONFIG_TPS_MIN 110 //5v full scale = 1024
#define CONFIG_TPS_MAX 780
#define CONFIG_TPS_VALID_MIN 50 //Error threshholds
#define CONFIG_TPS_VALID_MAX 900


#define PIN_MAP A7 //actually MAF at the moment
#define CONFIG_MAP_MIN 0
#define CONFIG_MAP_MAX 250 //250kPa = 5v for our sensor
#define CONFIG_VALID_MAP_MAX 1013 //The largest ADC value that is valid for the MAP sensor
#define CONFIG_VALID_MAP_MIN 10 //The smallest ADC value that is valid for the MAP sensor



#define PIN_FUEL PIN_PJ2
#define CONFIG_FUEL_CUT_RPM 5000 //red line at which to cut fuel
#define CONFIG_FUEL_PRIME_TIME 2000000//time (us) to prime fuel pump

#define PIN_VVT 29

#define PIN_CRANK 3
#define CONFIG_CRANK_TRIGGER_EDGE RISING
#define PIN_CAM 2
#define CONFIG_CAM_TRIGGER_EDGE RISING

#define CONFIG_CRANKING_FIXED_TIMING 0 //spark timing during cranking
#define CONFIG_CRANKING_ENRICHMENT 180 //20% extra fuel during cranking

#define PIN_LED_BLUE 38
#define PIN_LED_GREEN PIN_PD6

#define PIN_FAN 14
#define CONFIG_FAN_ON HIGH
#define CONFIG_FAN_OFF LOW
#define CONFIG_FAN_ON_TEMP 100 //celsius
#define CONFIG_FAN_OFF_TEMP 90

#define PIN_BAT A2
#define CONFIG_BAT_MAX 282 //full scale voltage


#define CONFIG_PULSE_WIDTH_LIMIT 49152U //injector pulse width limit in au

#define CONFIG_DFCO_TPS_THRESH	2
#define CONFIG_DFCO_RPM_THRESH	2000
#define CONFIG_DFCO_RPM_HYS		200

#define CONFIG_ASE_SECONDS		10 //how many seconds should ASE stay on
#define CONFIG_ASE_CORRECTION	120

#define CONFIG_FLOOD_CLEAR_TPS 90
