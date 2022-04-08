#include "ADCSPhotodiodeArray.h"	

/* 
 * Configure the pins attached to the multiplexer (a-c) as outputs and the pin on analog input as an input.
 */
ADCSPhotodiodeArray::ADCSPhotodiodeArray(uint8_t analog_input, uint8_t a, uint8_t b, uint8_t c){
	// configure digital pins
	pinMode(a, OUTPUT);
	digitalWrite(a, LOW);

	pinMode(b, OUTPUT);
	digitalWrite(b, LOW);

	pinMode(c, OUTPUT);
	digitalWrite(c, LOW);

	// configure analog pins
	pinMode(analog_input, INPUT);
}

/*
 * Read the value measured on one of the 6 multiplexer channels.
 */
int ADCSPhotodiodeArray::read(uint8_t channel){
	switch(channel){
		case 0:
			digitalWrite(a, LOW);
			digitalWrite(b, LOW);
			digitalWrite(c, LOW);
			break;

		case 1:
			digitalWrite(a, HIGH);
			digitalWrite(b, LOW);
			digitalWrite(c, LOW);
			break;

		case 2:
			digitalWrite(a, LOW);
			digitalWrite(b, HIGH);
			digitalWrite(c, LOW);
			break;

		case 3:
			digitalWrite(a, HIGH);
			digitalWrite(b, HIGH);
			digitalWrite(c, LOW);
			break;

		case 4:
			digitalWrite(a, LOW);
			digitalWrite(b, LOW);
			digitalWrite(c, HIGH);
			break;

		case 5:
			digitalWrite(a, LOW);
			digitalWrite(b, LOW);
			digitalWrite(c, HIGH);
			break;

		default:
			digitalWrite(a, LOW);
			digitalWrite(b, LOW);
			digitalWrite(c, LOW);

	}

	return analogRead(analog_pin);
}