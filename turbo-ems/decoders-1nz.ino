#include <limits.h>
#include "globals.h"
#include "decoders.h"
#include "scheduler.h"
#include "angles.h"
#include "config.h"


void update_RPM()
{
	noInterrupts();
    unsigned long tmp;
    tmp=(crank.gap[crank.last] + crank.gap[(uint8_t)(crank.last-1)] + crank.gap[(uint8_t)(crank.last-2)] + crank.gap[(uint8_t)(crank.last-3)] + crank.gap[(uint8_t)(crank.last-4)]);//get sum of last 4 gaps

    //subtract largest gap, just in case missing tooth
    uint8_t largest=crank.last;
    if( crank.gap[(uint8_t)(crank.last-1)] > crank.gap[largest] )largest=(uint8_t)(crank.last-1);
    if( crank.gap[(uint8_t)(crank.last-2)] > crank.gap[largest] )largest=(uint8_t)(crank.last-2);
    if( crank.gap[(uint8_t)(crank.last-3)] > crank.gap[largest] )largest=(uint8_t)(crank.last-3);
    if( crank.gap[(uint8_t)(crank.last-4)] > crank.gap[largest] )largest=(uint8_t)(crank.last-4);
    tmp-=crank.gap[largest];
    interrupts();
    tmp*=9;//multiply by 9 for time of full rotatation; 9*4=36

    if(tmp==0){
      crank.rpm=0;
      return;
    }

    crank.rpm = 60000000/tmp;
}

void update_crank_angle()
{
  	if(crank.tooth<0)
  	{
  	  crank.angle=0;
      return;
  	}

    noInterrupts();
    
    au_t crank_angle = DEGREES_TO_AU(CRANK_OFFSET);
    crank_angle += crank.tooth * (32767U/36U);//360 degrees
    crank_angle += time_to_au( micros_safe() - crank.tim[crank.last] );

    if ( crank.revolution_one == true ) {
        crank_angle += 32767U;
    }

	if(cam.tooth>=0)
	{
		update_cam_angle();
		au_t tmp1=cam.angle-crank_angle;
		au_t tmp2=crank_angle-cam.angle;
		if(tmp2<tmp1)tmp1=tmp2;
	  if ( tmp1 > 16383U )//180 degrees
	  {
	    crank.revolution_one!=crank.revolution_one;
      	crank_angle +=32767U;//360 degrees
	  }
	}

	interrupts();

    crank.angle = crank_angle;
}

void update_cam_angle()//returns a value close to crank angle, but with vvt offset. One full rotation equals 720 degrees
{
    if (cam.tooth<0)
    {
      cam.angle=0;
      return;//-1 for error
    }
  
    noInterrupts();

    au_t cam_angle = (cam.tooth * 16383U) + CAM_OFFSET;
    cam_angle += time_to_au( micros_safe() - cam.tim[cam.last] );

    interrupts();

    cam.angle = cam_angle;
}

void triggerCrank() //TODO: more safety checks?
{
    uint32_t curr_time=micros_safe();

	if (curr_time-crank.tim[crank.last] < 200)
	{
		return;
	}

	++crank.last;

    crank.tim[crank.last]=curr_time;
    crank.gap[crank.last]=crank.tim[crank.last]-crank.tim[(uint8_t)(crank.last-1)];

    if (	(crank.gap[crank.last] > crank.gap[(uint8_t)(crank.last-1)]*2) &&
			(crank.gap[crank.last] < crank.gap[(uint8_t)(crank.last-1)]*4) &&
			(crank.gap[crank.last] > crank.gap[(uint8_t)(crank.last-2)]*2) &&
			(crank.gap[crank.last] < crank.gap[(uint8_t)(crank.last-2)]*4) &&
			(crank.gap[crank.last] > crank.gap[(uint8_t)(crank.last-3)]*2) &&
          	(crank.gap[crank.last] < crank.gap[(uint8_t)(crank.last-3)]*4) &&
          	(crank.gap[crank.last] > crank.gap[(uint8_t)(crank.last-4)]*2) &&
          	(crank.gap[crank.last] < crank.gap[(uint8_t)(crank.last-4)]*4) )
    {
        if(crank.tooth<33)++crank.error_skipped_tooth;
		if(crank.tooth>33)++crank.error_extra_tooth;
        currentStatus.hasSync = true;
        currentStatus.startRevolutions++;
        crank.tooth = 0;
        crank.revolution_one = !crank.revolution_one;
    }else
	{
    	++crank.tooth;
	}

	
}

void triggerCam()//For Toyota Echo Camshaft. Toyota Echo cam has 3 protruding points spaced 90 degrees apart.
{
    uint32_t curr_time=micros_safe();

	if (curr_time-cam.tim[cam.last] < 3500)
	{
	//	++cam.error_count;
		return;
	}

	++cam.last;
    
	cam.tim[cam.last]=curr_time;
	cam.gap[cam.last]=cam.tim[cam.last]-cam.tim[(uint8_t)(cam.last-1)];

    if (    	(cam.gap[cam.last] > cam.gap[(uint8_t)(cam.last-1)]*3/2) &&
			    (cam.gap[cam.last] < cam.gap[(uint8_t)(cam.last-1)]*3) && 
			    (cam.gap[cam.last] > cam.gap[(uint8_t)(cam.last-2)]*3/2) &&
			    (cam.gap[cam.last] < cam.gap[(uint8_t)(cam.last-2)]*3))//We just passed the missing tooth
    {
		if(cam.tooth<2)++cam.error_skipped_tooth;
		if(cam.tooth>2)++cam.error_extra_tooth;
		cam.tooth = 0;
    }else
	{
		++cam.tooth;
	};
}
