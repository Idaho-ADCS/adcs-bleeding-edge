/****************************************************************
 * Library for interfacing with the DRV 10970 motor driver.
 *
 * @author: Garrett Wells
 * @date: 01/18/22
 *
 * Provides a configurable interface for working with the DRV 10970 motor driver. Initially written for University of Idaho senior capstone 2022 for use by the
 * Attitude adjustment team working for NASA.
 ****************************************************************/
#include "DRV_10970.h"

DRV10970::DRV10970(int fg, int fr, int brkmod, int pwm, int rd){
    FG = fg;            // frequency indication pin
    FR = fr;            // motor direction control
    BRKMOD = brkmod;    // brake mode setting pin
    PWM = pwm;          // variable duty cycle pwm input pin for speed control
    RD = rd;            // lock indication pin
    // set pin modes
    pinMode(FG, INPUT);
    pinMode(FR, OUTPUT);
    digitalWrite(FR, LOW);
    //pinMode(BRKMOD, OUTPUT); // this pin not currently exposed
    pinMode(PWM, OUTPUT);
    stop(); // make sure the motor output pin is not floating initially

    pinMode(RD, INPUT);
}

/*
 * Write the motor direction to the motor direction control or FR pin.
 *      HIGH = UVW driving sequence (CW)
 *      LOW = UWV (CCW)
 *      NOTE: Minimum PWM duty cycle is 10%.
 */
void DRV10970::run(MotorDirection dir, int dc){
    // write direction
    digitalWrite(FR, dir);
    // write PWM
    analogWrite(PWM, dc);
}

/*
 * Stop the motor spindle and send into a low power mode. Exit low power mode
 *  by driving PWM pin
 *      NOTE: pwm must be low for at least 1.2ms to enter low power mode
 */
void DRV10970::stop(){
    analogWrite(PWM, LOW);
}

/*
 * Read the RPM of the spindle using the FG pin to measure frequency.
 */
int DRV10970::readRPM(bool debug=false){
    long int t0 = millis(); // read start time
    long int cT = millis(); // current time
    int gap = 50;                    // milliseconds to measure for
    int toggles = 0;                 // set number of electrical toggles
    int prev_state = digitalRead(FG);     // state variable
    int cState;
    while(cT - t0 < gap){
        cState = digitalRead(FG); // the current state of the FG pin

        if(debug) Serial.println(cState);

        if(prev_state != cState){ // if has toggled, count it
            prev_state = cState;
            toggles++;
        }
        cT = millis();
    }

    // TODO: do some math here to figure out the RPM
    // probably something like
    // toggles gap

    //We are assuming here that 1 "toggle" is equal to 1 full rotation of the spindle. If we record for 1s, you can multiply the
    //value we get by 60; which would give us the revolutions per min (RPM). *IN THEORY*
    //toggles = toggles*60;


    return toggles;
}

/*
 * Read the RD pin to determine if the spindle is currently locked (HIGH).
 */
bool DRV10970::spindleFree(){
    return digitalRead(RD) == HIGH;
}
