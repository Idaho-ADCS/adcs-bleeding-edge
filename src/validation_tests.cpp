/*
 * These are the tests used to validate the correct operation of our system.
 */

#include <validation_tests.h>

/*
 * Create all RTOS tasks for testing
 */
void create_test_tasks(void){
    #if DEBUG
        SERCOM_USB.println("\tINITIALIZING RTOS TEST SUITE");
    #endif
    
    xTaskCreate(basic_motion, "BASIC MOTION", 256, NULL, 1, NULL);
    xTaskCreate(basic_attitude_determination, "BASIC AD", 256, NULL, 1, NULL);
    xTaskCreate(basic_attitude_control, "BASIC AC", 256, NULL, 1, NULL);
    xTaskCreate(simple_detumble, "SIMPLE DETUMBLE", 256, NULL, 1, NULL);
    xTaskCreate(simple_orient, "SIMPLE ORIENT", 256, NULL, 1, NULL); 
    
    #if DEBUG
        SERCOM_USB.println("\tINITIALIZED RTOS TEST SUITE");
    #endif
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

    // notify the TES by sending an empty packet with status set
    ADCSdata data;
    data.setStatus(STATUS_TEST_START);
    data.computeCRC();

    while(true){
        #if DEBUG
            SERCOM_USB.println("[BASIC MOTION] checked mode");
        #endif

        xQueuePeek(modeQ, (void*)&mode, (TickType_t)0);

        if(mode == CMD_TST_BASIC_MOTION){
            if(multiplier == 0){
                data.send();
            }

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

                // notify the TES by sending an empty packet with status set
                data.setStatus(STATUS_TEST_END);
                data.computeCRC();
                data.send();

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

                // change mode to standby
                uint8_t mode = CMD_STANDBY;
                xQueueOverwrite(modeQ, (void*)&mode);  // enter specified mode
            }
        }

        vTaskDelay(FREQ);
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

    uint8_t mode; // last received ADCS mode

    const int target_rot_vel = 0;   // rotational velocity we want to maintain
    const int step_size = 1;        // minimum step size to take when error is present
    int num_steps = 0;

    const float P = 0.25;            // proportional component
    int I = 0;                      // integral component
    int D = 0;                      // derivative component

    int prev_error = 0;             // error from last loop iteration
    int error = 0;                  // assumed stationary at start

    int pwm_output = 0;             // init at zero, signed to represent direction

    while(true){
        #if DEBUG
            SERCOM_USB.println("[BASIC DETUMBLE] checked mode");
        #endif
        xQueuePeek(modeQ, &mode, 0);

        if(mode == CMD_TST_SIMPLE_DETUMBLE){
            // calculate error
            ICM_20948_I2C *sensor_ptr1 = &IMU1; // IMU data can only be accessed through
            if (IMU1.dataReady())
                IMU1.getAGMT();  // acquires data from sensor
            float rot_vel_z = sensor_ptr1->gyrZ();

            error = rot_vel_z - target_rot_vel;     // difference between current state and target state
            // proportional term calculation
            float p_term = error * P;
            // integral term accumulation
            I += error;
            int i_term = I;
            // derivative term
            int d_term = I/num_steps;
            // output
            pwm_output = p_term; //+ i_term + d_term;
            if(pwm_output > 255){ // cap output
                pwm_output = 255;
            }

            num_steps++;

            // unsigned pwm to motor with direction
            if(error > 0){
                flywhl.run(REV, pwm_output);

            }else if(error < 0){
                flywhl.run(FWD, pwm_output);

            }else{ 
                // TODO: desaturate with magnetorquers if |pwm_output| == 255 or in state of equilibrium
                // for the meantime, just keep doing the same thing
            }

            if(DEBUG){
                SERCOM_USB.println("====== PID LOOP ======");
                SERCOM_USB.print("IMU VELOCITY = ");
                SERCOM_USB.print(rot_vel_z);
                SERCOM_USB.println(" degrees/sec");

                SERCOM_USB.print("ERROR = ");
                SERCOM_USB.println(error);

                SERCOM_USB.print("PWM OUTPUT = ");
                SERCOM_USB.println(pwm_output);
                SERCOM_USB.println("======================");
            }

            prev_error = error;

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