#ifndef __COMMAND_FUNCTIONS_H__
#define __COMMAND_FUNCTIONS_H__


#include "comm.h"
#include "sensors.h"
#include "DRV_10970.h"
/* These are the main functionalies and/or processes, they rely on the support
 * functions (located in supportFunctions.cpp) to properally execute */

//This tests the sensors and makes sure they are reading correctly
void testFun();
//This function has the satellite do nothing until something is in UART
void standby();
//This corrdinates the satellite's rotation
void orient(const char *direction);

#endif