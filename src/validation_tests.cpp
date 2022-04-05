/*
 * These are the tests used to validate the correct operation of our system.
 */

#include <validation_tests.h>

/*
 * Create all RTOS tasks for testing
 */
void create_test_tasks(void){
    xTaskCreate(basic_motion, "BASIC MOTION", 256, NULL, 1, NULL);
    xTaskCreate(basic_attitude_determination, "BASIC AD", 256, NULL, 1, NULL);
    xTaskCreate(basic_attitude_control, "BASIC AC", 256, NULL, 1, NULL);
    xTaskCreate(simple_detumble, "SIMPLE DETUMBLE", 256, NULL, 1, NULL);
    xTaskCreate(simple_orient, "SIMPLE ORIENT", 256, NULL, 1, NULL); 

    #ifdef DEBUG
        SERCOM_USB.println("\tINITIALIZED RTOS TEST SUITE");
    #endif
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
static void writeUART(void *pvParameters)
{
    uint8_t mode;
    ADCSdata data_packet;

    #ifdef DEBUG
    char mode_str[8];
    #endif

    while (1)
    {
        xQueuePeek(modeQ, (void*)&mode, (TickType_t)0);

        if (mode == MODE_TEST)
        {
            data_packet.setStatus(STATUS_OK);
            readIMU(data_packet);
            readINA(data_packet);
            data_packet.computeCRC();
            data_packet.send();  // send to TES
            #ifdef DEBUG
                SERCOM_USB.write("[writeUART] wrote to UART\r\n");
                //printScaledAGMT(&IMU1);
            #endif

            data_packet.clear();
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/*
 * Assume start from start, then begin increasing flywheel RPM until the system begins to spin.
 *  RULES:
 *      1. check the IMU every loop and if the rotation rate is above MAX_TEST_SPD, stop revving
 *      2. RPM only go up if some input is received...
 */
void basic_motion(void* pvParameters){
    uint8_t mode;
    int multiplier = 0.00;
    int pwm_sig = 255 * multiplier; // 0%
    const int MAX_TEST_SPD = 10; // upper limit is 10 degrees per second/1.667 rpm

    const TickType_t FREQ = 2000 / portTICK_PERIOD_MS; 

    while(true){
        #if DEBUG
            SERCOM_USB.println("[BASIC MOTION] checked mode");
        #endif

        xQueuePeek(modeQ, (void*)&mode, (TickType_t)0);

        if(mode == CMD_TST_BASIC_MOTION){
            float rot_vel_z = IMU1.gyrZ();

            if( rot_vel_z < MAX_TEST_SPD && multiplier < 1 ){ // as long as we are spinning slower than our goal, continue
                pwm_sig = 255 * multiplier;
                flywhl.run(FWD, pwm_sig);

                if(multiplier < 1){ multiplier += 0.01; }
                #if DEBUG                
                    else{
                        Serial.println("[BASIC MOTION] multiplier hit ceiling");
                    }
                #endif
            }else{ // stop the test

                flywhl.stop();

                #if DEBUG 
                    if( rot_vel_z > MAX_TEST_SPD ){
                        SERCOM_USB.print("[BASIC MOTION] rotational velocity greater than MAX_TEST_SPD ( ");
                        SERCOM_USB.print(MAX_TEST_SPD);
                        SERCOM_USB.println(" deg/s )");
                    }else if (multiplier >= 1){
                        SERCOM_USB.print("[BASIC MOTION] multiplier hit ceiling but velocity still ");
                        SERCOM_USB.print(rot_vel_z);
                        SERCOM_USB.println(" deg/s");
                    }
                #endif
            }
        }

        vTaskDelay(FREQ);
    }

}


/*
 * Reads all sensors periodically and transmits the data back to the main system via UART.
 *  1. Tests the ADCSdata class in comm.h
 *  2. Tests reading from sensors and data packet construction
 */
void basic_heartbeat(void* pvParameters){
    uint8_t mode;
    ADCSdata dat;

    while(true){
        #if DEBUG
            SERCOM_USB.println("[BASIC HEARTBEAT] checked mode");
        #endif
        xQueuePeek(modeQ, (void*)&mode, (TickType_t)0);

        if(mode == 0){
            // read all sensors
            readIMU(dat);
            readINA(dat); // TODO: reading INA may have an issue currently

            // TODO: read sunsensors

            // send data to satellite...
            dat.computeCRC();
            dat.send();
            dat.clear();
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // transmit once per second
    }
}



/**
 * @brief      calculate and output result of attitude determination, MODE_TEST_AD
 *
 * @param      pvParameters  The pv parameters
 */
void basic_attitude_determination(void* pvParameters){
    uint8_t mode;

    while(true){
        #if DEBUG
            SERCOM_USB.println("[BASIC AD] checked mode");
        #endif
        xQueuePeek(modeQ, &mode, 0);

        if(mode == CMD_TST_BASIC_AD){
            // TODO: write the attitude determination test in validation_tests.cpp
            // TODO: read IMU
            // TODO: use geographic lib to get ideal
            // TODO: calculate the vector...

        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
} 

/**
 * @brief      attempt to spin the satellite a pre-determined amount, MODE_TEST_AC
 *
 * @param      pvParameters  The pv parameters
 */
void basic_attitude_control(void* pvParameters){
    uint8_t mode;
    while(true){
        #if DEBUG
            SERCOM_USB.println("[BASIC AC] checked mode");
        #endif
        xQueuePeek(modeQ, &mode, 0);

        if(mode == CMD_TST_BASIC_AC){
            // TODO: write the attitude control test in validation_tests.cpp
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief      test basic ability to stop system from spinning, MODE_TEST_SMPLTUMBLE

 *
 * @param      pvParameters  The pv parameters
 */
void simple_detumble(void* pvParameters){
    uint8_t mode;
    while(true){
        #if DEBUG
            SERCOM_USB.println("[BASIC DETUMBLE] checked mode");
        #endif
        xQueuePeek(modeQ, &mode, 0);

        if(mode == CMD_TST_SIMPLE_DETUMBLE){
            // TODO: write the detumble test in validation_tests.cpp
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}               

/**
 * @brief      test ability to orient the system, MODE_TEST_ORIENT
 *
 * @param      pvParameters  The pv parameters
 */
void simple_orient(void* pvParameters){
    uint8_t mode;
    while(true){
        #if DEBUG
            SERCOM_USB.println("[BASIC ORIENT] checked mode");
        #endif
        
        xQueuePeek(modeQ, &mode, 0);

        if(mode == CMD_TST_SIMPLE_ORIENT){
            // TODO: write the orient test in validation_tests.cpp
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}