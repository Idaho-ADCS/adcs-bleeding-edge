// Our own headers
#include <global.h>					/* definitions of interfaces and test modes */
#include "samd51_pin_definitions.h"
#include "comm.h" 					/* data packets and UART transmission functions */
#include "sensors.h" 				/* read from sensors into heartbeat packet */
#include "supportFunctions.h"
#include "commandFunctions.h"
#include "DRV_10970.h"
#include "validation_tests.h" // tests for functionality

// Arduino library headers
#include "INA209.h"
#include "ICM_20948.h"
#include "FreeRTOS_SAMD51.h"
// Standard C/C++ library headers
#include <stdint.h>

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
bool flywhl_is_spinning = false;		// flywheel flag, if set then flywheel is spinning
bool magnetorquer_on = false;			// magnetorquer flag, if set then it is on so don't read magnetometer

/* ISRs ===================================================================== */
// none currently

/* CORE FUNCTIONS =========================================================== */
void state_machine_transition(TEScommand);	// takes the new command and changes system mode(modeQ) to reflect that new state

/* HELPER FUNCTIONS ========================================================= */
void init_hardware(void);
void init_PWM_50kHz(void);				// set PWM output on a pin to 50kHz instead of default
void init_sensors(void);
void init_rtos_architecture(void);

/* RTOS TASK DECLARATIONS =================================================== */
static void readUART(void *pvParameters);
static void heartbeat(void *pvParameters);

/* RTOS HANDLES ============================================================= */
TaskHandle_t* readUART_h;

/**
 * @brief
 * FreeRTOS queue that stores the current mode of the ADCS.
 * Possible values:
 *   MODE_STANDBY (0)
 *   MODE_TEST    (1)
 */
QueueHandle_t modeQ;

/* "MAIN" =================================================================== */

/**
 * @brief
 * Since main is already defined by the Arduino framework, we will use the
 * setup function as if it were main. Since setup runs only once, it
 * essentially behaves the same as main.
 */
void setup(void)
{
	#if DEBUG
    	/**
     	* Initialize USB connection to computer. Used to print debug messages.
     	* Baud rate: 115200
     	* Data bits: 8
     	* Parity: none
     	*/
    	SERCOM_USB.begin(115200);
    	while (!SERCOM_USB);  // wait for initialization to complete
    	SERCOM_USB.write("USB interface initialized\r\n");
    	print_global_config(); // print what debug modes are currently enabled
	#endif

    /**
     * Initialize UART connection to satellite
     * Baud rate: 115200
     * Data bits: 8
     * Parity: odd (1 bit)
     */
    SERCOM_UART.begin(115200, SERIAL_8O1);
    while (!SERCOM_UART);  // wait for initialization to complete
	#if DEBUG
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
}

/*
 * Loop is defined as the Idle Task Function for this port of FreeRTOS and must be defined
 * NOTE:
 * 		- USB noted to interrupt from sleep here... https://forum.arduino.cc/t/standby-sleep-mode-on-samd51/576584/6
 * 		- ALSO... this doesn't turn off pins if they are set high, state is saved
 */
void loop(){
	/* SAMD51 USBCore.cpp sets enable bit during sleep standby mode so USB needs to be disabled if in DEBUG*/
	#if !TEST_CONTROL_FLOW
		#if DEBUG
	    	USB->HOST.CTRLA.reg &= ~USB_CTRLA_ENABLE; // disable USB in standby sleep mode...
		#endif
	#endif

	// SLEEPRDY bit must be set to sleep
	if(PM_INTFLAG_SLEEPRDY)
		__WFI(); // enter low power mode when running Idle task to enter low power mode

	return;
}


/* Initialize all the GPIO pins and the builtin LED */
void init_hardware(void){
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

	#if DEBUG
		SERCOM_USB.println("HARDWARE INITIALIZED");
	#endif
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
	PORT->Group[g_APinDescription[FLYWHL_PWM_PIN].ulPort].PINCFG[g_APinDescription[DRV_PWM].ulPin].bit.PMUXEN = 1;

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

	#if DEBUG
		SERCOM_USB.println("PWM INITIALIZED");
	#endif
}


/* Setup the sensor objects */
void init_sensors(void){
	#if !TEST_CONTROL_FLOW
		/**
		 * Initialize first IMU
		 * Address: 0x69 or 0x68
		 */
	    IMU1.begin(SERCOM_I2C, AD0_VAL);
	    while (IMU1.status != ICM_20948_Stat_Ok);  // wait for initialization to
	                                               // complete
		#if DEBUG
	    	SERCOM_USB.write("IMU1 initialized\r\n");
		#endif

		#if TWO_IMUS
			/**
		 	* Initialize second IMU
		 	* Address: 0x68 or 0x69
		 	*/
	    	IMU2.begin(SERCOM_I2C, AD0_VAL^1);  // initialize other IMU with opposite
											// value for bit 0
	    	while (IMU2.status != ICM_20948_Stat_Ok);  // wait for initialization to
	                                               // complete
			#if DEBUG
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
	#endif

	#if DEBUG
    	SERCOM_USB.println("SENSORS INITIALIZED");
	#endif

}

/* Create all RTOS tasks, queues, semaphores, etc */
void init_rtos_architecture(void){
	// Create a counting semaphore with a maximum value of 1 and an initial
	// value of 0. Starts ADCS in standby mode.
	modeQ = xQueueCreate(1, sizeof(uint8_t));
	uint8_t mode = MODE_TEST;
	xQueueSend(modeQ, (void*)&mode, (TickType_t)0);

    xTaskCreate(readUART, "Read UART", 2048, NULL, 1, readUART_h);
    xTaskCreate(heartbeat, "HEARTBEAT", 2048, NULL, 1, NULL); // test function to send heartbeat every half-second

	#if DEBUG
    	SERCOM_USB.println("INITIALIZED COMMAND MONITOR");
	#endif

    // TESTS
    #if RTOS_TEST_SUITE
		#if DEBUG
	    	SERCOM_USB.println("REQUESTED: INITIALIZE RTOS TEST SUITE");
		#endif
    	create_test_tasks(); // if we are in test mode, create the tasks
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
 */
static void readUART(void *pvParameters)
{
	TEScommand cmd_packet;
	ADCSdata response;
	uint8_t mode;

	#if DEBUG
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
					state_machine_transition(cmd_packet); // publish mode, get ready to enter it too
				}
				else
                {
					// send error message if CRC is not valid
					response.setStatus(STATUS_COMM_ERROR);
					response.computeCRC();
					response.send();
				}

				#if DEBUG
	                // convert int to string for USB monitoring
	                sprintf(cmd_str, "0x%02x", cmd_packet.getCommand());

	                // print command value to USB
	                SERCOM_USB.print("Command received: ");
	                SERCOM_USB.print(cmd_str);
	                SERCOM_USB.print("\r\n");

	                if (cmd_packet.getCommand() == CMD_HEARTBEAT)
	                    SERCOM_USB.print("Entering heartbeat only mode\r\n");

	                if (cmd_packet.getCommand() == CMD_STANDBY)
	                    SERCOM_USB.print("Entering standby mode\r\n");
				#endif
            }
        }

		vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief
 * Reads magnetometer and gyroscope values from IMU and writes them to UART
 * every 0.5 seconds while ADCS is in test mode.
 *
 * @param[in] pvParameters  Unused but required by FreeRTOS. Program will not
 * compile without this parameter. When a task is instantiated from this
 * function, a set of initialization arguments or NULL is passed in as
 * pvParameters, so pvParameters must be declared even if it is not used.
 *
 * @return None
 */
static void heartbeat(void *pvParameters)
{
    uint8_t mode;
    ADCSdata data_packet;

    #ifdef DEBUG
    char mode_str[8];
    #endif

    while (1)
    {
        xQueuePeek(modeQ, (void*)&mode, (TickType_t)0);

        if (mode == CMD_HEARTBEAT)
        {
            data_packet.setStatus(STATUS_OK);
            readIMU(data_packet);
            readINA(data_packet);
            data_packet.computeCRC();
            data_packet.send();  // send to TES
            #ifdef DEBUG
                SERCOM_USB.write("[HEARTBEAT] wrote to UART\r\n");
                //printScaledAGMT(&IMU1);
            #endif

            data_packet.clear();
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/*
 * Handles transition from the current mode (modeQ) to the new mode selected by the input command. 
 */
void state_machine_transition(TEScommand cmand){
	// set new state from the command
	uint8_t mode = cmand.getCommand();
	uint8_t curr_mode = CMD_STANDBY; 
	// get the current state to compare against	
	xQueuePeek(modeQ, (void*)curr_mode, 0);
	// make sure we are entering a new state
	if(mode == curr_mode){ // if not, exit
		return;
	}

	bool command_is_valid = true;

	switch(mode){
		case CMD_HEARTBEAT:
			// do test command stuff
			break;

		// WARNING: if you switch out of a test mode the test will still be on the stack
		// 	on the plus side, the task won't run though
		case CMD_TST_BASIC_MOTION:
		case CMD_TST_BASIC_AD:
		case CMD_TST_BASIC_AC:
		case CMD_TST_SIMPLE_DETUMBLE:
		case CMD_TST_SIMPLE_ORIENT:
			create_test_tasks();
			break;

		case CMD_STANDBY:
			// print heartbeat regularly turn off actuators
			if(magnetorquer_on){
				// TODO: SM turn magnetorquer off
			}
			if(flywhl_is_spinning){
				// TODO: SM turn off the flywheel
			}
			break;
		
		// TODO: SM fill out the other modes with functional code
		case CMD_ORIENT_DEFAULT: // should be orienting to something like X+
		case CMD_ORIENT_X_POS:
		case CMD_ORIENT_Y_POS:
		case CMD_ORIENT_X_NEG:
		case CMD_ORIENT_Y_NEG:

		default: // do nothing
			command_is_valid = false;
			#if DEBUG
				Serial.println("HIT AN UNKNOWN OR UNIMPLEMENTED COMMAND");
			#endif
	}

	if(command_is_valid){
		xQueueOverwrite(modeQ, (void*)&mode);  // enter specified mode

		#if DEBUG
	    	char cmd_str[8];  // used to print command value to serial monitor
            // convert int to string for USB monitoring
            sprintf(cmd_str, "0x%02x", cmand.getCommand());

            // print command value to USB
            SERCOM_USB.print("Command set:");
            SERCOM_USB.print(cmd_str);
            SERCOM_USB.print("\r\n");
        #endif

		// TODO: SM init the new mode, maybe turn off an unneeded actuator? clear some data?
	}

}
