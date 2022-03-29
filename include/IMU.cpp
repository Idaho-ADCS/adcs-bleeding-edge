/****************************************************************
* Based on the Sparkfun provided library for the ICM-20949.
* Modified by Garrett Wells for a CS senior capstone project. This file provides
*   only the initialization and setup configuration stuff for the SAMD51. Use with
*   different hardware may result in unexpected errors or lack of function.
*
* USING THIS FILE:
*       To use the IMU...
*       1. Call initIMU(), should setup IMU using ICM_20848 library
*       2. Use library functions to get data (see ICM_20848.h in lib). Useful
            functions below.
            * getAGMT() reads from sensors and stores values in agmt struct
            * accX(), accY(), accZ() read only acceleration
            * magXYZ() read only magnetic values on an axis
            * gyrXYZ() read only gyro values on axis
            * temp() read only temperature
            * sleep() put device into sleep, may take a bit to wake up
            * lowPower() less time to wake than sleep??
*       3. To print debug data to the serial port there are helper functions in this file.
*           Make sure you setup Serial first.
***************************************************************/
#include "ICM_20949.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20949_IMU
#include <Wire.h>
#include "wiring_private.h"

// Create a new wire interface
TwoWire myWire(&sercom2, 12, 14);

#define SERIAL_PORT SerialUSB

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT myWire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 1       // The value of the last bit of the I2C address.                \
                        // On the SparkFun 9DoF IMU breakout the default is 1, and when \
                        // the ADR jumper is closed the value becomes 0

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

/*
    Call this function before attempting to read from the IMU, sets up the
    I2C interface for the SAMD51
*/
void initIMU()
{
    #ifdef USE_SPI
        SPI_PORT.begin();
    #else
        WIRE_PORT.begin();
        WIRE_PORT.setClock(400000);
    #endif

    bool initialized = false;
    while (!initialized)
    {
        #ifdef USE_SPI
        myICM.begin(CS_PIN, SPI_PORT);
        #else
        myICM.begin(WIRE_PORT, AD0_VAL);
        #endif

        if (myICM.status != ICM_20948_Stat_Ok)
        {
            delay(500);
        }
        else
        {
            initialized = true;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// Below here are some helper functions to print the data nicely!
////////////////////////////////////////////////////////////////////////////////
void printPaddedInt16b(int16_t val)
{
    if (val > 0)
    {
        SERIAL_PORT.print(" ");
        if (val < 10000)
        {
            SERIAL_PORT.print("0");
        }
        if (val < 1000)
        {
            SERIAL_PORT.print("0");
        }
        if (val < 100)
        {
            SERIAL_PORT.print("0");
        }
        if (val < 10)
        {
            SERIAL_PORT.print("0");
        }
    }
    else
    {
        SERIAL_PORT.print("-");
        if (abs(val) < 10000)
        {
            SERIAL_PORT.print("0");
        }
        if (abs(val) < 1000)
        {
            SERIAL_PORT.print("0");
        }
        if (abs(val) < 100)
        {
            SERIAL_PORT.print("0");
        }
        if (abs(val) < 10)
        {
            SERIAL_PORT.print("0");
        }
    }
    SERIAL_PORT.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt)
{
    SERIAL_PORT.print("RAW. Acc [ ");
    printPaddedInt16b(agmt.acc.axes.x);
    SERIAL_PORT.print(", ");
    printPaddedInt16b(agmt.acc.axes.y);
    SERIAL_PORT.print(", ");
    printPaddedInt16b(agmt.acc.axes.z);
    SERIAL_PORT.print(" ], Gyr [ ");
    printPaddedInt16b(agmt.gyr.axes.x);
    SERIAL_PORT.print(", ");
    printPaddedInt16b(agmt.gyr.axes.y);
    SERIAL_PORT.print(", ");
    printPaddedInt16b(agmt.gyr.axes.z);
    SERIAL_PORT.print(" ], Mag [ ");
    printPaddedInt16b(agmt.mag.axes.x);
    SERIAL_PORT.print(", ");
    printPaddedInt16b(agmt.mag.axes.y);
    SERIAL_PORT.print(", ");
    printPaddedInt16b(agmt.mag.axes.z);
    SERIAL_PORT.print(" ], Tmp [ ");
    printPaddedInt16b(agmt.tmp.val);
    SERIAL_PORT.print(" ]");
    SERIAL_PORT.println();
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
    float aval = abs(val);
    if (val < 0)
    {
        SERIAL_PORT.print("-");
    }
    else
    {
        SERIAL_PORT.print(" ");
    }
    for (uint8_t indi = 0; indi < leading; indi++)
    {
        uint32_t tenpow = 0;
        if (indi < (leading - 1))
        {
            tenpow = 1;
        }
        for (uint8_t c = 0; c < (leading - 1 - indi); c++)
        {
            tenpow *= 10;
        }
        if (aval < tenpow)
        {
            SERIAL_PORT.print("0");
        }
        else
        {
            break;
        }
    }
    if (val < 0)
    {
        SERIAL_PORT.print(-val, decimals);
    }
    else
    {
        SERIAL_PORT.print(val, decimals);
    }
}

#ifdef USE_SPI
void printScaledAGMT(ICM_20948_SPI *sensor)
{
    #else
    void printScaledAGMT(ICM_20948_I2C *sensor)
    {
        #endif
        SERIAL_PORT.print("Scaled. Acc (mg) [ ");
        printFormattedFloat(sensor->accX(), 5, 2);
        SERIAL_PORT.print(", ");
        printFormattedFloat(sensor->accY(), 5, 2);
        SERIAL_PORT.print(", ");
        printFormattedFloat(sensor->accZ(), 5, 2);
        SERIAL_PORT.print(" ], Gyr (DPS) [ ");
        printFormattedFloat(sensor->gyrX(), 5, 2);
        SERIAL_PORT.print(", ");
        printFormattedFloat(sensor->gyrY(), 5, 2);
        SERIAL_PORT.print(", ");
        printFormattedFloat(sensor->gyrZ(), 5, 2);
        SERIAL_PORT.print(" ], Mag (uT) [ ");
        printFormattedFloat(sensor->magX(), 5, 2);
        SERIAL_PORT.print(", ");
        printFormattedFloat(sensor->magY(), 5, 2);
        SERIAL_PORT.print(", ");
        printFormattedFloat(sensor->magZ(), 5, 2);
        SERIAL_PORT.print(" ], Tmp (C) [ ");
        printFormattedFloat(sensor->temp(), 5, 2);
        SERIAL_PORT.print(" ]");
        SERIAL_PORT.println();
    }
