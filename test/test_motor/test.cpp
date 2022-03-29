#include <Arduino.h>
#include <unity.h>

//#include "C:\Users\deepg\OneDrive\Documents\U_of_I\2021-22_UofI\CS480\ADCS\adcs\lib\DRV10970\src\DRV_10970.h"
#include <DRV_10970.h>

DRV10970* DRV;

// Make sure that when we init the DRV10970 it does not start spinning
void test_default_state(void){
    DRV = new DRV10970(DRV_FG, DRV_FR, DRV_BRKMOD, DRV_PWM, DRV_RD);
    TEST_ASSERT_EQUAL(0, DRV->readRPM(false));
}

// Make sure the pins are initialized to valid values
void test_pin_values(void){
    TEST_ASSERT(DRV_FG != 0 && DRV_FR != 0 && DRV_BRKMOD == 0 &&  DRV_PWM != 0 && DRV_RD != 0);
}

// Cycle from 0 to 100% DC and check that frequency pin is outputting
void test_frequency(void){
    float step_size = 0.05;
    float percentage_output = 0;
    int count;

    while(percentage_output <= 1){
        DRV->run(FWD, percentage_output * 255);
        percentage_output += step_size;
        delay(1000);
        count = DRV->readRPM(false);
    }

    DRV->stop();
    TEST_ASSERT(count > 0);
}

void setup(void){
    // start serial debug
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_pin_values);
    RUN_TEST(test_default_state);
    RUN_TEST(test_frequency);
}

void loop(){
    UNITY_END();
}
