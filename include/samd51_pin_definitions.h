#ifndef ADCS_PIN_DEFINITIONS_H
#define ADCS_PIN_DEFINITIONS_H

/*
 * Define pins that are used by the ADCS system for reference and porting to other systems
 *
 * DIGITAL: 0, 1, 4, 5, 6, 9, 10, 11, 12, 13, 22 (MISO), 23 (MOSI), 24 (SCK)
 * ANALOG: A0-5
 * 
 * There will be sunsensors... 
 * All other sensors are I2C interfaces
 */


// FLYWHEEL MOTOR DRIVER PINS
#define FLYWHL_RPM_PIN 6     // frequency/rpm indication pin
#define FLYWHL_DIR_PIN 9     // motor direction pin
#define FLYWHL_BRKMOD_PIN -1  // brake mode (coast/brake), not currently available
#define FLYWHL_PWM_PIN 10    // pwm output pin
#define FLYWHL_LOCK_INDICATION_PIN 5  // lock indication pin

// MOTOR ENABLE
#define MOTOR_ENABLE_PIN A1

// PHOTODIODE
#define PHOTODIODE_INPUT_PIN A0

// SUN SENSOR MULTIPLEXER
#define SUNSENSOR_C 11
#define SUNSENSOR_B 12
#define SUNSENSOR_A 13

// MAGNETORQUER CONTROL PINS
#define MAGNETORQUER_2_REV_PIN 4    // reverse pin for ZXBM5210 controlling first magnetorquer
#define MAGNETORQUER_2_FWD_PIN 22   // same but forward pin

#define MAGNETORQUER_1_REV_PIN 23   // reverse pin for ZXBM5210 controlling second magnetorquer
#define MAGNETORQUER_1_FWD_PIN 24   // same but forward pin

// BUCK CONVERTER
#define BUCK_ENABLE_PIN A5          // Buck converter is ON when high, provides power to both magnetorquers

// UART
#define RX 0
#define TX 1

// I2C
#define SDA_PIN 20
#define SCL_PIN 21


#endif