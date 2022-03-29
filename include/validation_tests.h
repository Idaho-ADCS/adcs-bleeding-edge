#ifndef VALIDATION_TEST_H
#define VALIDATION_TEST_H

#include "DRV_10970.h"
#include "ICM_20948.h"
#include <FreeRTOS_SAMD51.h>
#include <stdint.h>

extern DRV10970 flywhl;
extern ICM_20948_I2C IMU1;

void basic_motion1(void* pvParameters);

#endif
