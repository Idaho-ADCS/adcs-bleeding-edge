#ifndef VALIDATION_TEST_H
#define VALIDATION_TEST_H
/*
 * These tasks are defined for testing individual parts of the ADCS system. To enable a specific test mode, set that as the ADCS mode. For list of operation modes
 *  see supportFunctions.h
 */

#include <global.h>
#include <DRV_10970.h>
#include <ICM_20948.h>
#include <ADCSPhotodiodeArray.h>
#include <FreeRTOS_SAMD51.h>
#include <stdint.h>
#include <comm.h>               /* data packet and transmission functions */
#include <sensors.h>            /* read from sensors */
#include <supportFunctions.h>   /* ADCS operation modes */

extern DRV10970 flywhl;
extern ICM_20948_I2C IMU1;
extern QueueHandle_t modeQ;     /* state machine publishes mode here */

void create_test_tasks(); // create all RTOS tasks for testing

// TEST TASKS //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void writeUART(void *pvParameters);              // transmit heartbeat every second, active when mode is TEST_MODE
void basic_motion(void* pvParameters);                  // test the flywheel to make sure it spins, active for MODE_TEST_HRTBT
void basic_heartbeat(void* pvParameters);               // test reading sensors and transmitting heartbeat to the main system, MODE_TEST_MOTION
void basic_attitude_determination(void* pvParameters);  // test attitude determination, MODE_TEST_AD
void basic_attitude_control(void* pvParameters);        // test attitude control, MODE_TEST_AC
void simple_detumble(void* pvParameters);               // test basic ability to stop system from spinning, MODE_TEST_SMPLTUMBLE
void simple_orient(void* pvParameters);                 // test ability to orient the system, MODE_TEST_ORIENT

#endif
