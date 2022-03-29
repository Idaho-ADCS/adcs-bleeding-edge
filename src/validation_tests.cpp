/*
 * These are the tests used to validate the correct operation of our system.
 */

#include "validation_tests.h"

/*
 * Assume start from start, then begin increasing flywheel RPM until the system begins to spin.
 *  RULES:
 *      1. check the IMU every loop and if the rotation rate is above MAX_TEST_SPD, stop revving
 *      2. RPM only go up if some input is received...
 */
void basic_motion1(void* pvParameters){
    const int pwm_sig = 255 * 0.05; // 5%
    const int MAX_TEST_SPD = 10;

    const TickType_t FREQ = 1000 / portTICK_PERIOD_MS;

    while(true){
        float rot_vel_z = IMU1.gyrZ();

        Serial.write("USB interface initialized\r\n");

        if( rot_vel_z < MAX_TEST_SPD){
            flywhl.run(FWD, pwm_sig);
        }

        vTaskDelay(FREQ);
    }

}
