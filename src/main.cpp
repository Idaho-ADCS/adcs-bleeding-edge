// Our own headers
#include "adcs_pin_definitions.h"
#include "comm.h" 					/* data packets and UART transmission functions */
#include "sensors.h" 				/* read from sensors into heartbeat packet */
#include "supportFunctions.h"
#include "commandFunctions.h"
#include "DRV_10970.h"
#include "validation_tests.h" // tests for functionality

// only include this if test functions are desired
//#include <test.h>

// Arduino library headers
#include "INA209.h"
#include "ICM_20948.h"
#include "FreeRTOS_SAMD51.h"
// Standard C/C++ library headers
#include <stdint.h>

// TESTING DEFINES //////////////////////////////////////////////////////////////
#define DEBUG 			/* print the serial debug messages over USB connection */
#define TESTHEART  		/* test if can transmit heartbeat */
//#define TESTMOTION 	/* test if can spin flywheel */
/////////////////////////////////////////////////////////////////////////////////

/* NON-RTOS GLOBAL VARIABLES ================================================ */

/**
 * @brief
 * IMU objects, attached to IMUs. Used to read data from IMUs.
 */
ICM_20948_I2C IMU1;
#ifdef TWO_IMUS
ICM_20948_I2C IMU2;
#endif

// INA209 object
INA209 ina209(0x40);

// DRV10970 object
DRV10970 flywhl;

/* RTOS GLOBAL VARIABLES ==================================================== */

/**
 * @brief
 * FreeRTOS queue that stores the current mode of the ADCS.
 * Possible values:
 *   MODE_STANDBY (0)
 *   MODE_TEST    (1)
 */
QueueHandle_t modeQ;

/* HELPER FUNCTIONS ========================================================= */

void init_hardware(void);
void init_PWM_50kHz(void);				// set PWM output on a pin to 50kHz instead of default
void init_sensors(void);
void init_rtos_architecture(void);

/* RTOS TASK DECLARATIONS =================================================== */

static void readUART(void *pvParameters);
static void writeUART(void *pvParameters);

/* "MAIN" =================================================================== */

/**
 * @brief
 * Since main is already defined by the Arduino framework, we will use the
 * setup function as if it were main. Since setup runs only once, it
 * essentially behaves the same as main.
 */
int main(void)
{
	#ifdef DEBUG
    	/**
     	* Initialize USB connection to computer. Used to print debug messages.
     	* Baud rate: 115200
     	* Data bits: 8
     	* Parity: none
     	*/
    	SERCOM_USB.begin(115200);
    	while (!SERCOM_USB);  // wait for initialization to complete
    	SERCOM_USB.write("USB interface initialized\r\n");
	#endif

    /**
     * Initialize UART connection to satellite
     * Baud rate: 115200
     * Data bits: 8
     * Parity: odd (1 bit)
     */
    SERCOM_UART.begin(115200, SERIAL_8O1);
    while (!SERCOM_UART);  // wait for initialization to complete
	#ifdef DEBUG
    	SERCOM_USB.write("UART interface initialized\r\n");
	#endif

    /**
     * Initialize I2C network
     * Clock: 400 kHz
     */
    SERCOM_I2C.begin();
    SERCOM_I2C.setClock(400000);

	init_hardware(); 			// setup gpio pins
	//init_PWM_50kHz();
	init_sensors();				// init the sensor interfaces
	init_rtos_architecture(); 	// create rtos tasks and stuff

    // initialization completed, notify satellite
	ADCSdata data_packet;
    data_packet.setStatus(STATUS_HELLO);
    data_packet.computeCRC();
    data_packet.send();

    vTaskStartScheduler(); // start the RTOS

    // should never be reached if everything goes right
    while (1)
	{
		data_packet.setStatus(STATUS_ADCS_ERROR);
		data_packet.computeCRC();
		data_packet.send();
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	return 0;
}

/*
 * Loop is defined as the Idle Task Function for this port of FreeRTOS and must be defined
 * NOTE:
 * 		- USB noted to interrupt from sleep here... https://forum.arduino.cc/t/standby-sleep-mode-on-samd51/576584/6
 * 		- ALSO... this doesn't turn off pins if they are set high, state is saved
 */
void loop(){
	/* SAMD51 USBCore.cpp sets enable bit during sleep standby mode so USB needs to be disabled if in DEBUG*/
	#ifdef DEBUG
    	USB->HOST.CTRLA.reg &= ~USB_CTRLA_ENABLE; // disable USB in standby sleep mode...
	#endif
	// SLEEPRDY bit must be set to sleep
	if(PM_INTFLAG_SLEEPRDY)
		__WFI(); // enter low power mode when running Idle task to enter low power mode

	return;
}


/* Initialize all the GPIO pins and the builtin LED */
void init_hardware(void){
	Serial.println("INIT HARDWARE\n");
	// INIT SLEEP REG
	// configure power manager (PM) to enter STANDBY mode on _WFI()
	PM->SLEEPCFG.bit.SLEEPMODE = 0x4;
	//while(PM->SLEEPCFG.bit.SLEEPMODE != 0x4); // wait for register to set, cannot sleep
	while(!PM_INTFLAG_SLEEPRDY); // SLEEPRDY flag must be set before __WFI()

	// INIT GPIO
	// enable LED
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	pinMode(9, OUTPUT);
	digitalWrite(9, HIGH); // set the direction pin HIGH??

	pinMode(10, OUTPUT);
	analogWrite(10, 0); // set the PWM pin to 0%

	// INIT INTERRUPTS
	// TODO: setup UART interrupt in init_hardware


}

/*
 * Set the motor driver PWM pin to output @ 50kHz instead of... its default
 * Code from: https://forum.arduino.cc/t/metro-m4-express-atsamd51-pwm-frequency-and-resolution/566491/2
 */
void init_PWM_50kHz(void){
	// Set up the generic clock (GCLK7) to clock timer TCC0
	GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 48MHz clock source by divisor 1: 48MHz/1 = 48MHz
	                     GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
	                     GCLK_GENCTRL_GENEN |        // Enable GCLK7
	                     GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
	                     //GCLK_GENCTRL_SRC_DPLL1;     // Select 100MHz DPLL clock source
	                     //GCLK_GENCTRL_SRC_DPLL0;     // Select 120MHz DPLL clock source
	while (GCLK->SYNCBUSY.bit.GENCTRL7);               // Wait for synchronization

	GCLK->PCHCTRL[25].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC0 peripheral channel
	                      GCLK_PCHCTRL_GEN_GCLK7;    // Connect generic clock 7 to TCC0

	// Enable the peripheral multiplexer on pin D7
	PORT->Group[g_APinDescription[DRV_PWM].ulPort].PINCFG[g_APinDescription[DRV_PWM].ulPin].bit.PMUXEN = 1;

	// Set the D7 (PORT_PB12) peripheral multiplexer to peripheral (even port number) E(6): TCC0, Channel 0
	PORT->Group[g_APinDescription[DRV_PWM].ulPort].PMUX[g_APinDescription[DRV_PWM].ulPin >> 1].reg |= PORT_PMUX_PMUXE(6);

	TCC0->CTRLA.reg = TC_CTRLA_PRESCALER_DIV8 |        // Set prescaler to 8, 48MHz/8 = 6MHz
	                TC_CTRLA_PRESCSYNC_PRESC;        // Set the reset/reload to trigger on prescaler clock

	TCC0->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;             // Set-up TCC0 timer for Normal (single slope) PWM mode (NPWM)
	while (TCC0->SYNCBUSY.bit.WAVE)                    // Wait for synchronization

//  TCC0->PER.reg = 479;                               // Set-up the PER (period) register 100kHz PWM
	TCC0->PER.reg = 959;                               // Set-up the PER (period) register 50kHz PWM
	while (TCC0->SYNCBUSY.bit.PER);                    // Wait for synchronization

	TCC0->CC[0].reg = 478;                           // Set-up the CC (counter compare), channel 0 register for 50% duty-cycle
	while (TCC0->SYNCBUSY.bit.CC0);                    // Wait for synchronization

	TCC0->CTRLA.bit.ENABLE = 1;                        // Enable timer TCC0
	while (TCC0->SYNCBUSY.bit.ENABLE);                 // Wait for synchronization
}


/* Setup the sensor objects */
void init_sensors(void){
	/**
	 * Initialize first IMU
	 * Address: 0x69 or 0x68
	 */
    IMU1.begin(SERCOM_I2C, AD0_VAL);
    while (IMU1.status != ICM_20948_Stat_Ok);  // wait for initialization to
                                               // complete
	#ifdef DEBUG
    	SERCOM_USB.write("IMU1 initialized\r\n");
	#endif

	#ifdef TWO_IMUS
		/**
	 	* Initialize second IMU
	 	* Address: 0x68 or 0x69
	 	*/
    	IMU2.begin(SERCOM_I2C, AD0_VAL^1);  // initialize other IMU with opposite
										// value for bit 0
    	while (IMU2.status != ICM_20948_Stat_Ok);  // wait for initialization to
                                               // complete
		#ifdef DEBUG
    		SERCOM_USB.write("IMU2 initialized\r\n");
		#endif
	#endif

	/**
	 * Write default settings to INA209
	 * Reset: no
	 // * Bus voltage range: 32V
	 * PGA gain: /8
	 * PGA range: +-320mV
	 * ADC resolution: 12 bits
	 * ADC conversion time: 532us
	 * Mode: shunt and bus, continuous
	 */
    ina209.writeCfgReg(0x399f);

	/**
	 * Calibrate INA209
	 * Current LSB: 100uA
	 */
    ina209.writeCal(0x7fff);

	#ifdef DEBUG
    	SERCOM_USB.write("INA209 initialized\r\n");
	#endif

}

/* Create all RTOS tasks, queues, semaphores, etc */
void init_rtos_architecture(void){
	// Create a counting semaphore with a maximum value of 1 and an initial
	// value of 0. Starts ADCS in standby mode.
	modeQ = xQueueCreate(1, sizeof(uint8_t));
	uint8_t mode = MODE_TEST;
	xQueueSend(modeQ, (void*)&mode, (TickType_t)0);

    xTaskCreate(readUART, "Read UART", 2048, NULL, 1, NULL);
    //xTaskCreate(writeUART, "Write UART", 2048, NULL, 1, NULL); // test function to send heartbeat every half-second

    // TESTS
    #ifdef TESTHEART
    	xTaskCreate(basic_heartbeat, "BASIC HEARTBEAT TEST", 256, NULL, 1, NULL);
    #elif TESTMOTION
		xTaskCreate(basic_motion, "BASIC MOTION TEST", 256, NULL, 1, NULL);
    #endif

	#ifdef DEBUG
    	SERCOM_USB.write("Tasks created\r\n");
	#endif
}

/* RTOS TASK DEFINITIONS ==================================================== */

/**
 * @brief
 * Polls the UART module for data. Processes data one byte at a time if the
 * module reports that data is ready to be received.
 *
 * @param[in] pvParameters  Unused but required by FreeRTOS. Program will not
 * compile without this parameter. When a task is instantiated from this
 * function, a set of initialization arguments or NULL is passed in as
 * pvParameters, so pvParameters must be declared even if it is not used.
 *
 * @return None
 *
 * TODO: Remove polling and invoke this task using an interrupt instead.
 */
static void readUART(void *pvParameters)
{
	TEScommand cmd_packet;
	ADCSdata response;
	uint8_t mode;

	#ifdef DEBUG
    	char cmd_str[8];  // used to print command value to serial monitor
	#endif

    while (1)
    {
        if (SERCOM_UART.available())  // at least one byte is in the UART
        {							  // receive buffer

            // copy one byte out of UART receive buffer
			cmd_packet.addByte((uint8_t)SERCOM_UART.read());

			if (cmd_packet.isFull())  // full command packet received
            {
				if (cmd_packet.checkCRC())
				{
					// process command if CRC is valid
					if (cmd_packet.getCommand() == CMD_TEST)
						mode = MODE_TEST;

					if (cmd_packet.getCommand() == CMD_STANDBY)
						mode = MODE_STANDBY;
				}
				else
                {
					// send error message if CRC is not valid
					response.setStatus(STATUS_COMM_ERROR);
					response.computeCRC();
					response.send();
				}

				xQueueOverwrite(modeQ, (void*)&mode);  // enter specified mode

				#ifdef DEBUG
                // convert int to string for USB monitoring
                sprintf(cmd_str, "0x%02x", cmd_packet.getCommand());

                // print command value to USB
                SERCOM_USB.print("Command received: ");
                SERCOM_USB.print(cmd_str);
                SERCOM_USB.print("\r\n");

                if (cmd_packet.getCommand() == CMD_TEST)
                    SERCOM_USB.print("Entering test mode\r\n");

                if (cmd_packet.getCommand() == CMD_STANDBY)
                    SERCOM_USB.print("Entering standby mode\r\n");
				#endif
            }
        }

		// vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
