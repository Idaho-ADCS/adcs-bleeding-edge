#ifndef __SUPPORT_FUNCTIONS_H__
#define __SUPPORT_FUNCTIONS_H__

/* These are functions that support the main command functions in one way
 * or another they are called upon in the commandFunctions.cpp file */
/* I tried to make these as self-explanatory as possible by name
 * but I also added little descriptions anyways -Kristie */

// TODO: write test tasks for each of the test modes defined in supportFunctions.h

// descriptive names for mode values
#define MODE_STANDBY 0
#define MODE_TEST 1             /* send heartbeat packet every second, eventually more */
#define MODE_TEST_HRTBT 2       /* send heartbeat packet every second */
#define MODE_TEST_MOTION 3      /* start spinning flywheel, stop once rotation speed is above threshold */
#define MODE_TEST_AD 4          /* calculate and output result of attitude determination */
#define MODE_TEST_AC 5          /* attempt to spin the satellite a pre-determined amount */
#define MODE_TEST_SMPLTUMBLE 6  /* starting in spin, attempt to stop spinning */
#define MODE_TEST_ORIENT 7      /* try orienting the adcs system */

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