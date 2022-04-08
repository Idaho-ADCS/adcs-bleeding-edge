#ifndef GLOBAL_H
#define GLOBAL_H

#include <Arduino.h>

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


enum MotorDirection {REV=0, FWD=1};


/*
 * Print a summary of the defines above and what their current ON/OFF state is.
 */
static void print_global_config(void){
    SERCOM_USB.println("=== GLOBAL CONFIG ===");
    // test modes
    SERCOM_USB.print("DEBUG [");
    if(DEBUG){
        SERCOM_USB.print("ON]");
    }else{
        SERCOM_USB.print("OFF]");
    }

    SERCOM_USB.print("TEST CONTROL FLOW [");
    if(TEST_CONTROL_FLOW){
        SERCOM_USB.print("ON]");
    }else{
        SERCOM_USB.print("OFF]");
    }

    SERCOM_USB.print("RTOS_TEST_SUITE [");
    if(RTOS_TEST_SUITE){
        SERCOM_USB.print("ON]");
    }else{
        SERCOM_USB.print("OFF]");
    }

    // two imus 
    SERCOM_USB.print("TWO_IMUS [");
    if(SERCOM_USB){
        SERCOM_USB.print("ON]");
    }
    SERCOM_USB.println();
}


#endif