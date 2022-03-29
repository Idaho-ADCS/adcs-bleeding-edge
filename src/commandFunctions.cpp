/* These are the main functionalies and/or processes, they rely on the support
 * functions (located in supportFunctions.cpp) to properally execute */

#include "commandFunctions.h"
#include "supportFunctions.h"

extern ICM_20948_I2C IMU1;
#ifdef TWO_IMUS
extern ICM_20948_I2C IMU2;
#endif
extern DRV10970* DRV;

//This tests the sensors and makes sure they are reading correctly
void testFun()
{
	//Call test/test.cpp?
	return;
}

//This will loop indefinately until there is something in UART
void standby()
{
	//Delay then return?
	//In standby, auto try and detumble? Since this is the first mode...
	return;
}

//Starts rotation and checks that rotation actually occured
//This function will be changed later to support direction
void orient(const char *direction)
{
	ICM_20948_I2C *sensor_ptr = &IMU1;
//First ping values
	int8_t     magX;
	int8_t	   magY;
	int8_t	   magZ;
	fixed5_3_t gyroX;
	fixed5_3_t gyroY;
	fixed5_3_t gyroZ;
//Second ping values
	int8_t     DIFFmagX;
	int8_t	   DIFFmagY;
	int8_t	   DIFFmagZ;
	fixed5_3_t DIFFgyroX;
	fixed5_3_t DIFFgyroY;
	fixed5_3_t DIFFgyroZ;

	const int duration = 10000; // 10s
    volatile long int t0 = millis();


	//For now, check that satelite is currently stablized. If not, stablize then rotate.
	//Copied from main.cpp/writeUART(): Get current rotation/speed of rotation
	//To test if there is rotation, it will test the difference between 2 data points taken ~1 sec apart.
	if (IMU1.dataReady())
	{
		IMU1.getAGMT();  // acquires data from sensor

//Get data for first set of values
		// extract data from IMU object
		magX = (int8_t)sensor_ptr->magX();
		magY = (int8_t)sensor_ptr->magY();
		magZ = (int8_t)sensor_ptr->magZ();

		gyroX = floatToFixed(sensor_ptr->gyrX());
		gyroY = floatToFixed(sensor_ptr->gyrY());
		gyroZ = floatToFixed(sensor_ptr->gyrZ());

		//Copied from test/test.cpp
		const int duration = 1000; // 1s
		volatile long int t0 = millis();
		while(millis() - t0 < duration){/*do nothing*/}

//After wait, grab the second set of values and calculate difference
		IMU1.getAGMT();  // acquires data from sensor

		// extract data from IMU object
		DIFFmagX = magX-((int8_t)sensor_ptr->magX());
		DIFFmagY = magY-((int8_t)sensor_ptr->magY());
		DIFFmagZ = magZ-((int8_t)sensor_ptr->magZ());

		DIFFgyroX = gyroX-(floatToFixed(sensor_ptr->gyrX()));
		DIFFgyroY = gyroY-(floatToFixed(sensor_ptr->gyrY()));
		DIFFgyroZ = gyroZ-(floatToFixed(sensor_ptr->gyrZ()));
		//gyro Z AXIS

		//0 would mean no rotation, anything other than 0 would be a range of rotation (bigger the number, more rotation)
		//We really only care about what's going on with the Z-axis, the others are a sanity check.
		if(DIFFgyroZ != 0) 
		{
			//Start planned rotation here
			return; 
		}
		else //If here, there is some rotation occuring
		{
			//Rotate to stop rotation and call the function again after a select duration 
			if(DIFFgyroZ > 0) //Positive spin
			{
				DRV->run(FWD, 0.1*255); // start at 10%
				while(millis() - t0 < duration){/*do nothing*/}
				DRV->stop();
			}
			else //Negitive spin
			{
				DRV->run(FWD, 0.1*255); // start at 10%
				while(millis() - t0 < duration){/*do nothing*/}
				DRV->stop();
			}
			orient(direction); //Call again
		}
	
	}
	return;//Probably print error here.. (If here, IMU's were not ready)
}
