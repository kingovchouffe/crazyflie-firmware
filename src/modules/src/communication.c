/*
 * Code for the communication between crazyflie and PC to transmit sensor's data
 */

#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "config.h"
#include "crtp.h"
#include "log.h"
#include "crc.h"
#include "worker.h"
#include "num.h"

#include "console.h"
#include "cfassert.h"
#include "debug.h"
#include "static_mem.h"
#include "stabilizer.h"
#include "stabilizer_types.h"
#include "position_control.h"

static bool isInit;
static double currentPos[3];
static double currentVel[3];
point_t pos1;
velocity_t vel1;
//Private function
static void communicationTask();
static bool dataTransfer();
STATIC_MEM_TASK_ALLOC(communicationTask, COMMUNICATION_TASK_STACKSIZE);

void communicationInit(){
	 if (isInit)
	    return;

	STATIC_MEM_TASK_CREATE(communicationTask, communicationTask, COMMUNICATION_TASK_NAME, NULL, COMMUNICATION_TASK_PRI);
	isInit = true;
}

void communicationTask(){
	dataTransfer();
}

bool dataTransfer(){
	 currentPos[0]=pos1.x;
	 currentPos[1]=pos1.y;
	 currentPos[2]=pos1.z;

	 currentVel[0]=vel1.x;
	 currentVel[1]=vel1.y;
	 currentVel[2]=vel1.z;
	 positionControl(currentPos, currentVel);
	DEBUG_PRINT("phid=%f \n",  ctrlpos.phid);
	DEBUG_PRINT("thetad=%f \n", ctrlpos.thetad);
	DEBUG_PRINT("thrust=%f \n", ctrlpos.thrust);
	return true;
}
