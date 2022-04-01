/*
 * These are the tests used to validate the correct operation of our system.
 */

#include <validation_tests.h>

/*
 * Create all RTOS tasks for testing
 */
void create_test_tasks(void){
    // TODO: validation_tests.cpp -> create rtos test tasks
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
            // SERCOM_USB.write("Wrote to UART\r\n");
            // printScaledAGMT(&IMU1);
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
    const int pwm_sig = 255 * 0.05; // 5%
    const int MAX_TEST_SPD = 10;

    const TickType_t FREQ = 1000 / portTICK_PERIOD_MS;

    while(true){
        xQueuePeek(modeQ, (void*)&mode, (TickType_t)0);

        if(mode == MODE_TEST_MOTION){
            float rot_vel_z = IMU1.gyrZ();

            //------------------------------------------------------------------
            /// @brief      This class describes a write.
            ///
            Serial.write("USB interface initialized\r\n");

            if( rot_vel_z < MAX_TEST_SPD){
                flywhl.run(FWD, pwm_sig);
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
        xQueuePeek(modeQ, (void*)&mode, (TickType_t)0);

        if(mode == MODE_TEST_HRTBT){
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
        xQueuePeek(modeQ, &mode, 0);

        if(mode == MODE_TEST_AD){
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
        xQueuePeek(modeQ, &mode, 0);

        if(mode == MODE_TEST_AC){
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
        xQueuePeek(modeQ, &mode, 0);

        if(mode == MODE_TEST_AD){
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
        xQueuePeek(modeQ, &mode, 0);

        if(mode == MODE_TEST_AD){
            // TODO: write the orient test in validation_tests.cpp
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}