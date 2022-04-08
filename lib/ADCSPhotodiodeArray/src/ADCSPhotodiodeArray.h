#ifndef ADCSPHOTODIODEARRAY_H
#define ADCSPHOTODIODEARRAY_H

#include <Arduino.h>

/*
 * Truth table for the multiplexer that the 6 photodiodes will be hooked up to.
 *  
 * INPUTS
 * | A | B | C | CHANNEL | 
 * -----------------------
 * | L | L | L |    0    |
 * | H | L | L |    1    |
 * | L | H | L |    2    |
 * | H | H | L |    3    |
 * | L | L | H |    4    |
 * | H | L | H |    5    |
 * 
 */

class ADCSPhotodiodeArray {
private:
	uint8_t analog_pin, a, b, c;

public:
	ADCSPhotodiodeArray(uint8_t analog_input, uint8_t a, uint8_t b, uint8_t c);
	int read(uint8_t channel); // select which channel to read from by number
};

#endif
