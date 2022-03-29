#ifndef __SENSORS_H__
#define __SENSORS_H__

#include "comm.h"
#include "ICM_20948.h"
#include "INA209.h"

void readIMU(ADCSdata &data_packet);
void readINA(ADCSdata &data_packet);

// "helper" functions from IMU demo code that print sensor data to serial
// monitor over USB
void printPaddedInt16b(int16_t val);
void printRawAGMT(ICM_20948_AGMT_t agmt);
void printFormattedFloat(float val, uint8_t leading, uint8_t decimals);
void printScaledAGMT(ICM_20948_I2C *sensor);

#endif