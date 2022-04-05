#ifndef GLOBAL_H
#define GLOBAL_H

/*
 * Contains all preprocessor definitions that should be held(included) commonly across all source and header
 * 	files in the project.
 * 
 * Notable entries will include:
 * 	1. Sensor configurations (number of IMUs)
 * 	2. Peripheral interface enables (Serial, I2C, UART)
 * 	3. Debug/Test defines for test modes
 */

/* SENSOR CONFIGURATIONS */
#define TWO_IMUS 0

/* PERIPHERAL DEFINITIONS */
#define SERCOM_USB Serial
#define SERCOM_UART Serial1
#define SERCOM_I2C Wire

/* DEBUG/TEST DEFINITIONS */
#define TEST_CONTROL_FLOW 1 /* if non-zero then assume satellite is connected but no sensors */
#define RTOS_TEST_SUITE 0 	/* if non-zero then rtos test tasks in validation_tests.h will be created */ 
#define DEBUG 1 			/* if non-zero then debug statements will be printed over USB serial */


#endif