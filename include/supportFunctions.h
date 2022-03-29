#ifndef __SUPPORT_FUNCTIONS_H__
#define __SUPPORT_FUNCTIONS_H__

/* These are functions that support the main command functions in one way
 * or another they are called upon in the commandFunctions.cpp file */
/* I tried to make these as self-explanatory as possible by name
 * but I also added little descriptions anyways -Kristie */

// descriptive names for mode values
#define MODE_STANDBY 0
#define MODE_TEST 1

// #define TWO_IMUS

//Parses cmd and calls appropriate function
void doCmd(char *cmd);
//Reads output of sensors and compiles it into the array
void getData(int *data);
//Sends array data to main satellite system
void sendToSystem(int *data);
//Runs motors for a certain number of rotations
void startRotation(int rotations);

#endif