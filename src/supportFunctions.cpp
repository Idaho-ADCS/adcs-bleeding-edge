/* These are functions that support the main command functions in one way
 * or another they are called upon in the commandFunctions.cpp file */

#include "supportFunctions.h"
#include "commandFunctions.h"
#include <FreeRTOS_SAMD51.h>
#include "comm.h"
#include <stdint.h>

//Parses cmd and calls appropriate function
void doCmd(char *cmd)
{
	if (cmd[0] == 0xa0) //Binary: 10100000
	{
		testFun();
	}
	else if (cmd[0] == 0xc0) //Binary: 11000000
	{
		standby();
	}
	//For now, the direction doesn't matter, but it's set up for later
	else if (cmd[0] == 0xe0) //Binary: 11100000
	{
		orient("X+");
	}
	else if (cmd[0] == 0xe1) //Binary: 11100001
	{
		orient("Y+");
	}
	else if (cmd[0] == 0xe2) //Binary: 11100010
	{
		orient("X-");
	}
	else if (cmd[0] == 0xe3) //Binary: 11100011
	{
		orient("Y-");
	}
	return;
}

//Reads all sensor output and compiles everything into the data array
/* WIP: I am unsure what data type data should be (prob int or float?)..
 * I know it should be an array -Kristie */
void getData(int *data)
{
	return;
}

/* WIP: This will use the same data from function above, sends out to main
 * satellite system */
void sendToSystem(int *data)
{
	return;
}

//Runs motors for a certain number of rotations
/* WIP: This function will need to take a rotation value (assuming int type)
 * from orient and then rotate the motors that amount */
void startRotation(int rotations)
{
	return;
}

