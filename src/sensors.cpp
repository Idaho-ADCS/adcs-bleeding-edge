#include "sensors.h"

extern ICM_20948_I2C IMU1;
#ifdef TWO_IMUS
extern ICM_20948_I2C IMU2;
#endif

extern INA209 ina209;

void readIMU(ADCSdata &data_packet)
{
	ICM_20948_I2C *sensor_ptr1 = &IMU1; // IMU data can only be accessed through
                                        // a pointer
#ifdef TWO_IMUS
	ICM_20948_I2C *sensor_ptr2 = &IMU2; // IMU data can only be accessed through
                                        // a pointer
#endif

	float mx, my, mz, gx, gy, gz;

	if (IMU1.dataReady())
		IMU1.getAGMT();  // acquires data from sensor

#ifdef TWO_IMUS
	if (IMU2.dataReady())
		IMU2.getAGMT();
#endif

#ifdef TWO_IMUS
	// extract data from IMU object
	mx = (sensor_ptr1->magY() + sensor_ptr2->magY()) / 2;
	my = (sensor_ptr1->magX() + sensor_ptr2->magX()) / 2;
	mz = (sensor_ptr1->magZ() + sensor_ptr2->magZ()) / -2;

	gx = (sensor_ptr1->gyrY() + sensor_ptr2->gyrY()) / 2;
	gy = (sensor_ptr1->gyrX() + sensor_ptr2->gyrX()) / 2;
	gz = (sensor_ptr1->gyrZ() + sensor_ptr2->gyrZ()) / -2;
#else
	mx = sensor_ptr1->magY();
	my = sensor_ptr1->magX();
	mz = sensor_ptr1->magZ() * -1;

	gx = sensor_ptr1->gyrY();
	gy = sensor_ptr1->gyrX();
	gz = sensor_ptr1->gyrZ() * -1;
#endif

	data_packet.setIMUdata(mx, my, mz, gx, gy, gz);
}

void readINA(ADCSdata &data_packet)
{
	float voltage;
	float current;

	int v_raw;
	int i_raw;

	v_raw = ina209.busVol();
	i_raw = ina209.current();

	voltage = v_raw / 1000;
	current = i_raw / 10;

	data_packet.setINAdata(voltage, current);

	char debug_str[32];
	sprintf(debug_str, "%d", v_raw);
	SERCOM_USB.write("Bus voltage: ");
	SERCOM_USB.write(debug_str);
	SERCOM_USB.write(" mV\r\n");
	sprintf(debug_str, "%d", i_raw*100);
	SERCOM_USB.write("Shunt current: ");
	SERCOM_USB.write(debug_str);
	SERCOM_USB.write(" uA\r\n");
}

void printPaddedInt16b(int16_t val)
{
	if (val > 0)
	{
		SERCOM_USB.print(" ");
		if (val < 10000)
		{
			SERCOM_USB.print("0");
		}
		if (val < 1000)
		{
			SERCOM_USB.print("0");
		}
		if (val < 100)
		{
			SERCOM_USB.print("0");
		}
		if (val < 10)
		{
			SERCOM_USB.print("0");
		}
	}
	else
	{
		SERCOM_USB.print("-");
		if (abs(val) < 10000)
		{
			SERCOM_USB.print("0");
		}
		if (abs(val) < 1000)
		{
			SERCOM_USB.print("0");
		}
		if (abs(val) < 100)
		{
			SERCOM_USB.print("0");
		}
		if (abs(val) < 10)
		{
			SERCOM_USB.print("0");
		}
	}
	SERCOM_USB.print(abs(val));
}

/**
 * @brief
 * This function has something to do with printing IMU data over USB to the
 * serial monitor. Other than that, I have no idea what it does. It came as part
 * of the IMU demo code, and printScaledAGMT, which is used to validate data
 * transmissions, relies on this function.
 * 
 * @param[in] agmt  An instance of the IMU data object
 * 
 * @return None
 */
void printRawAGMT(ICM_20948_AGMT_t agmt)
{
	SERCOM_USB.print("RAW. Acc [ ");
	printPaddedInt16b(agmt.acc.axes.x);
	SERCOM_USB.print(", ");
	printPaddedInt16b(agmt.acc.axes.y);
	SERCOM_USB.print(", ");
	printPaddedInt16b(agmt.acc.axes.z);
	SERCOM_USB.print(" ], Gyr [ ");
	printPaddedInt16b(agmt.gyr.axes.x);
	SERCOM_USB.print(", ");
	printPaddedInt16b(agmt.gyr.axes.y);
	SERCOM_USB.print(", ");
	printPaddedInt16b(agmt.gyr.axes.z);
	SERCOM_USB.print(" ], Mag [ ");
	printPaddedInt16b(agmt.mag.axes.x);
	SERCOM_USB.print(", ");
	printPaddedInt16b(agmt.mag.axes.y);
	SERCOM_USB.print(", ");
	printPaddedInt16b(agmt.mag.axes.z);
	SERCOM_USB.print(" ], Tmp [ ");
	printPaddedInt16b(agmt.tmp.val);
	SERCOM_USB.print(" ]");
	SERCOM_USB.println();
}

/**
 * @brief
 * This function has something to do with printing IMU data over USB to the
 * serial monitor. Other than that, I have no idea what it does. It came as part
 * of the IMU demo code, and printScaledAGMT, which is used to validate data
 * transmissions, relies on this function.
 * 
 * @param[in] val       Value to print
 * @param[in] leading   Number of digits left of the decimal
 * @param[in] decimals  Number of digits right of the decimal
 * 
 * @return None
 */
void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
	float aval = abs(val);
	if (val < 0)
	{
		SERCOM_USB.print("-");
	}
	else
	{
		SERCOM_USB.print(" ");
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
			SERCOM_USB.print("0");
		}
		else
		{
			break;
		}
	}
	if (val < 0)
	{
		SERCOM_USB.print(-val, decimals);
	}
	else
	{
		SERCOM_USB.print(val, decimals);
	}
}

/**
 * @brief
 * Prints IMU data over USB to the serial monitor. Converts raw data to a form
 * that is readable by humans. Came as part of the IMU demo code.
 * 
 * @param[in] sensor  Pointer to IMU object
 * 
 * @return None
 */
void printScaledAGMT(ICM_20948_I2C *sensor)
{
	SERCOM_USB.print("Scaled. Acc (mg) [ ");
	printFormattedFloat(sensor->accX(), 5, 2);
	SERCOM_USB.print(", ");
	printFormattedFloat(sensor->accY(), 5, 2);
	SERCOM_USB.print(", ");
	printFormattedFloat(sensor->accZ(), 5, 2);
	SERCOM_USB.print(" ], Gyr (DPS) [ ");
	printFormattedFloat(sensor->gyrX(), 5, 2);
	SERCOM_USB.print(", ");
	printFormattedFloat(sensor->gyrY(), 5, 2);
	SERCOM_USB.print(", ");
	printFormattedFloat(sensor->gyrZ(), 5, 2);
	SERCOM_USB.print(" ], Mag (uT) [ ");
	printFormattedFloat(sensor->magX(), 5, 2);
	SERCOM_USB.print(", ");
	printFormattedFloat(sensor->magY(), 5, 2);
	SERCOM_USB.print(", ");
	printFormattedFloat(sensor->magZ(), 5, 2);
	SERCOM_USB.print(" ], Tmp (C) [ ");
	printFormattedFloat(sensor->temp(), 5, 2);
	SERCOM_USB.print(" ]");
	SERCOM_USB.println();
}