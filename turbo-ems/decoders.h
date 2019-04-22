#ifndef DECODERS_H
#define DECODERS_H

#include "angles.h"

void triggerCrank();
void triggerCam();
void update_RPM();
void update_crank_angle();
void update_cam_angle();

#define MAX_STALL_TIME 50000 //The maximum time (in uS) that the system will continue to function before the engine is considered stalled/stopped. This is unique to each decoder.
#define CAM_OFFSET 37319U
#define CRANK_OFFSET 147 //degrees from first tooth edge to TDC

struct toothlog {
volatile uint32_t tim[256];//tooth edge times
volatile uint32_t gap[256];//time gaps between teeth
volatile au_t angle;
volatile int16_t rpm;
volatile uint8_t last=0;
volatile int8_t tooth=-1;//the current tooth number 0=first tooth
volatile int16_t error_extra_tooth;
volatile int16_t error_skipped_tooth;
volatile bool revolution_one = false;
};

struct toothlog crank;
struct toothlog cam;

#endif
