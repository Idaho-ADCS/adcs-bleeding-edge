#include "ZXMB5210.h"

/*
 * @brief      Construct a new instance.
 *
 * @param[in]  fwd   Forward pin
 * @param[in]  rev   The reverse pin
 * @param[in]  buck  The buck enable pin, must be high to drive motor
 */
ZXMB5210::ZXMB5210(uint8_t fwd, uint8_t rev, uint8_t buck){
	this->fwd_pin = fwd;
	this->rev_pin = rev;
	this->buck_enable = buck;

	pinMode(this->buck_enable, OUTPUT);
	digitalWrite(this->buck_enable, LOW);

	pinMode(this->fwd_pin, OUTPUT);
	digitalWrite(this->fwd_pin, LOW);

	pinMode(this->rev_pin, OUTPUT);
	digitalWrite(this->fwd_pin, LOW);

}

/**
 * @brief      Constructs a new instance. Doesn't set the state of the buck converter.
 *
 * @param[in]  fwd   Forward pin
 * @param[in]  rev   The reverse pin
 */
ZXMB5210::ZXMB5210(uint8_t fwd, uint8_t rev){
	this->fwd_pin = fwd;
	this->rev_pin = rev;

	pinMode(this->fwd_pin, OUTPUT);
	digitalWrite(this->fwd_pin, LOW);

	pinMode(this->rev_pin, OUTPUT);
	digitalWrite(this->fwd_pin, LOW);

}

/**
 * @brief      drive the motor forward
 */
void ZXMB5210::fwd(void){
	if(this->buck_enable < 255){ // send power to the magnetorquers
		digitalWrite(this->buck_enable, HIGH);
	}
	digitalWrite(this->fwd_pin, HIGH);
	digitalWrite(this->rev_pin, LOW);
}

/**
 * @brief      drive the motor in reverse
 */
void ZXMB5210::rev(void){
	if(this->buck_enable < 255){ // send power to the magnetorquers
		digitalWrite(this->buck_enable, HIGH);
	}
	digitalWrite(this->fwd_pin, LOW);
	digitalWrite(this->rev_pin, HIGH);

}

/**
 * @brief      motor driver enters standby mode, with outputs to the motor floating
 */
void ZXMB5210::standby(void){
	if(this->buck_enable < 255){ // turn off power to the magnetorquers
		digitalWrite(this->buck_enable, LOW);
	}
	digitalWrite(this->fwd_pin, LOW);
	digitalWrite(this->rev_pin, LOW);

}

/**
 * @brief      motor driver enters brake mode, with outputs to the motor both low, short circuit brake
 */
void ZXMB5210::stop(void){
	if(this->buck_enable < 255){ // turn off power to the magnetorquers
		digitalWrite(this->buck_enable, LOW);
	}
	digitalWrite(this->fwd_pin, HIGH);
	digitalWrite(this->rev_pin, HIGH);

}