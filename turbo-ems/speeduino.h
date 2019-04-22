#ifndef SPEEDUINO_H
#define SPEEDUINO_H

uint16_t PW();
inline void __attribute__((always_inline)) getVE();
inline void __attribute__((always_inline)) getAdvance();

struct config2 configPage2;
struct config4 configPage4; //Done
struct config6 configPage6;
struct config9 configPage9;
struct config10 configPage10;

uint16_t req_fuel_uS, inj_opentime_uS;

bool ignitionOn = false; //The current state of the ignition system
bool fuelOn = false; //The current state of the ignition system

uint32_t counter;
uint32_t currentLoopTime; //The time the current loop started (uS)
uint32_t previousLoopTime; //The time the previous loop started (uS)

unsigned long secCounter; //The next time to incremen 'runSecs' counter.

#endif
