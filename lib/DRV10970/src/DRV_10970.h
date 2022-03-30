/****************************************************************
 * Library for interfacing with the DRV 10970 motor driver.
 *
 * @author: Garrett Wells
 * @date: 01/18/22
 *
 * Provides a configurable interface for working with the DRV 10970 motor driver. Initially written for University of Idaho senior capstone 2022 for use by the
 * Attitude adjustment team working for NASA.
 ****************************************************************/
#ifndef DRV_10970_H
#define DRV_10970_H

#include <Arduino.h>
#include <adcs_pin_definitions.h>

// default pinout for the SAMD51
const int   DRV_FG = FLYWHL_RPM,     // frequency/rpm indication pin
            DRV_FR = FLYWHL_DIR,     // motor direction pin
            DRV_BRKMOD = FLYWHL_BRKMOD, // brake mode (coast/brake), not currently available
            DRV_PWM = FLYWHL_PWM,   // pwm output pin
            DRV_RD = FLYWHL_LOCKED;     // lock indication pin

enum MotorDirection {REV=0, FWD=1};

class DRV10970 {
    private:
        int FG, FR, BRKMOD, PWM, RD; // interface pins
    public:
        DRV10970(void){}
        DRV10970(int fg, int fr, int brkmod, int pwm, int rd);
        void run(MotorDirection dir, int dc); // drive motor in direction at dutycycle dc
        void stop(); // stop motor driver and put in low power state
        int readRPM(bool); // returns the rpm of motor spindle
        bool spindleFree(); // returns true if motor spindle is free to spin
 };
#endif