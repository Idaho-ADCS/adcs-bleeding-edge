#include <test.h>
#include "DRV_10970.h"
#include "sensors.h"
#include "commandFunctions.h"
#include <FreeRTOS_SAMD51.h>
#include <stdint.h>

DRV10970* DRV;
INA209* ina209;
ICM_20948_I2C IMU1;
ICM_20948_I2C IMU2;

/*
 * ADCS system test rig. Run to see test output on serial output in a loop.
 * @author  Garrett Wells
 * @since   01/23/2022
 */

// Initialization functions, each should print to serial debug on success/failure
void init_test(void);       // init ADCS systems and sensors
void initIMU_test(void);    // init IMU
void initDRV_test(void);    // init motor driver
void initINA_test(void);    // init current monitor

// Test functions, each should get input or send value to sensor or subsystem and then read/get feedback and print success/failure
void testIMU(void);     // test IMU readings to make sure values are plausible
void testDRV_RAMP(void);     // spin up in increments, and test ability to read wheel rpm
void testDRV_CONSTANT(void);     // spin up in increments, and test ability to read wheel rpm
void testINA(void);     // measure the draw of the system, should be non-zero


#ifdef TEST_H
/*
 * Init for the tests
 */
void setup(){
    DRV = new DRV10970(DRV_FG, DRV_FR, DRV_BRKMOD, DRV_PWM, DRV_RD);
    DRV->stop();
    // start serial debug
    Serial.begin(115200);
    while(!Serial){;}

    Serial.println("Ready to Test(1/0)?:");
    while(Serial.available() == 0){}

    int ans = Serial.parseInt();

    init_test(); // init all ADCS systems

}

/*
 * Run tests here to infinity
 */
void loop(){
    Serial.println("------ START TESTING SUBSYSTEMS ------");
    // Test IMU, values back should be close to zero if not moving
    //testIMU();
    testDRV_RAMP();
    testDRV_CONSTANT();
    //testINA();
    Serial.println("------ END TESTING SUBSYSTEMS ------\n");
}
#endif

/*
 * Init all systems here...
 */
void init_test(void){
    Serial.println("------ STARTING SYSTEM INIT ------");
    //initIMU_test();
    initDRV_test();
    //initINA_test();
    Serial.println("------ FINISHING SYSTEM INIT ------");
}

/*
 * Init the IMU connection over the I2C interface
 */
void initIMU_test(void){
    /**
     * Initialize I2C connection to IMU
     * Clock: 400 kHz
     * IMU address: 0x69
     */
    long int t0 = millis();
    SERCOM_I2C.begin();
    SERCOM_I2C.setClock(400000);
    IMU1.begin(SERCOM_I2C, 0);
    while (IMU1.status != ICM_20948_Stat_Ok);  // wait for initialization to
    long int cT = millis();
    Serial.print("INIT IMU [SUCCESS] in ");
    Serial.print(cT - t0);
    Serial.println(" ms");
}

/*
 * Init the motor driver for the flywheel
 */
void initDRV_test(void){
    if(DRV_FG == 0 || DRV_FR == 0 || DRV_PWM == 0 || DRV_RD == 0){
        Serial.println("INIT DRV10970 [FAILED]\n\t invalid pinout");

    }else {
        long int t0 = millis();
        //DRV->stop();
        long int cT = millis();
        Serial.print("INIT DRV10970 [SUCCESS] in ");
        Serial.print(cT - t0);
        Serial.println(" ms");

    }
 }

/*
 * Init the INA209 current sensor for monitoring system draw
 */
void initINA_test(void){
    // TODO init INA209 with real values, defaults are for 32V system
    long int t0 = millis();
    ina209 = new INA209(0x40);
    ina209->writeCfgReg(14751); // default
    ina209->writeCal(4096);
    long int cT = millis();
    Serial.print("INIT INA209 [SUCCESS] in ");
    Serial.print(cT - t0);
    Serial.println(" ms");
}

/*
 * Test the IMU, values should be close to zero but probably not zero if the unit is stationary
 */
void testIMU(void){
    Serial.println("\tIMU TEST");
    printScaledAGMT(&IMU1);
    Serial.println("");
}

/*
 * Test the DRV10979 motor driver
 */
void testDRV_RAMP(void){
    Serial.println("\tDRV10970 RAMP & RPM TEST");

    // set LED on pin 13 HIGH
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    const int duration = 1000; // 0.5s
    volatile long int t0 = millis();
    long int currentTime;
    double percentage = 0.1;
    while(true){

        currentTime = millis();
        if((currentTime - t0 > duration) && percentage < 1.0){
            // set duty cycle
            Serial.print("\tDC = ");
            Serial.println(percentage * 100);
            DRV->run(FWD, percentage*255);
            delay(2000);
            // read rpm
            int rpm_ticks = DRV->readRPM(false);
            Serial.print("\t\tRPM ticks: ");
            Serial.println(rpm_ticks);
            // change percentage for next run
            percentage += 0.1;
            t0 = currentTime;
        }else if(percentage >= 1.0){
            Serial.println("Ending Loop\n");
            break;
        }
    }

    Serial.println("\tstopping DRV10970");
    DRV->stop();
    Serial.println("\tDRV10970 stopped");
    delay(1000);
}

void testDRV_CONSTANT(){
    Serial.println("\tDRV10970 CONSTANT OUTPUT TEST");

    // set LED on pin 13 HIGH
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    // timing settings
    const int duration = 10000; // 10s
    const int poll_rt = 500; // poll frequency once every half second
    long int t0_test = millis();
    long int t0_prt = t0_test;
    long int currentTime;

    // start motor
    DRV->run(FWD, 255*0.2);

    while(true){
        currentTime = millis();

        if(currentTime - t0_prt > poll_rt){
            // read rpm
            int rpm_ticks = DRV->readRPM(false);
            Serial.print("\t\tRPM ticks: ");
            Serial.println(rpm_ticks);
            t0_prt = currentTime;
        }

        if(currentTime - t0_test > duration){
            Serial.println("\t\tEnding Loop\n");
            break;
        }
    }

    Serial.println("\tstopping DRV10970");
    DRV->stop();
    Serial.println("\tDRV10970 stopped");
    //delay(1000);
}

/*
 * Test the INA209's ability to monitor the power draw of the system
 */
void testINA(void){
    Serial.println("\tINA209 TEST");
    Serial.print("\t\tINA209 current (");
    Serial.print(ina209->current());
    Serial.println(") Amps");
}
