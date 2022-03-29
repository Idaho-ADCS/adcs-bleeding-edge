/* These are the main functionalies and/or processes, they rely on the support
 * functions (located in supportFunctions.cpp) to properally execute */

#include "commandFunctions.h"
#include "supportFunctions.h"
#include "testFun.h"

//This tests the sensors and makes sure they are reading correctly
//This function can be depriciated to just call TEST() when needed.
void testFun()
{
	TEST(); //In testFun.cpp
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
//This function can probably be simplifed and have some things ran in supportFunctions.cpp, but good 'nuff for now. 
void orient(const char *direction)
{
	ICM_20948_I2C *sensor_ptr1 = &IMU1;
	ICM_20948_I2C *sensor_ptr2 = &IMU2;
//First ping values for IMU 1
	fixed5_3_t gyroZ1;
//Second ping values for IMU 1
	fixed5_3_t DIFFgyroZ1;
//First ping values for IMU 2
	fixed5_3_t gyroZ2;
//Second ping values for IMU 2
	fixed5_3_t DIFFgyroZ2;


	//For now, check that satelite is currently stablized. If not, stablize then rotate.
	//Copied from main.cpp/writeUART(): Get current rotation/speed of rotation
	//To test if there is rotation, it will test the difference between 2 data points taken ~1 sec apart.
	if (IMU1.dataReady() && IMU2.dataReady())
	{
		IMU1.getAGMT();  // acquires data from sensor
		IMU2.getAGMT();  // acquires data from sensor

//Get data for first set of values
		// extract data from IMU object
		gyroZ1 = floatToFixed(sensor_ptr1->gyrZ());
		gyroZ2 = floatToFixed(sensor_ptr2->gyrZ());
		
		//Copied from test/test.cpp
		const int duration = 1000; // 1s
		volatile long int t0 = millis();
		while(millis() - t0 < duration){/*do nothing*/}

//After wait, grab the second set of values and calculate difference
		IMU1.getAGMT();  // acquires data from sensor
		IMU2.getAGMT();  // acquires data from sensor

		// extract data from IMU object
		DIFFgyroZ1 = gyroZ1-(floatToFixed(sensor_ptr1->gyrZ()));
		DIFFgyroZ2 = gyroZ2-(floatToFixed(sensor_ptr2->gyrZ()));
		//gyro Z AXIS

		//Takes an average of the second set of data
		fixed5_3_t Average = (DIFFgyroZ1+DIFFgyroZ2)/2;

		//0 would mean no rotation, anything other than 0 would be in some range of rotation (bigger the number, more rotation)
		//We really only care about what's going on with the Z-axis, the others are a sanity check.
		if(Average <= 1 && Average >= -1) //We probably can't get to a perfect 0, but a low rotation should be ok
		{
			//Start planned rotation here. Since we are just correcting for now, nothing more needs to be done; leave function.
			return; 
		}
		else //If here, there is some rotation occuring
		{
			//Rotate to stop rotation and call the function again after a select duration 
			DRV->run(FWD, 0.2*255); //20% TEMP--------

			//IDEA:
			//txt file that holds all of the previous data points of rotation (makes it easier to troubleshoot now, can move to array later)
			//Do the math for rotation

				//If theres a negitive value, rotate forwards
				//positive value, rotate backwards
			//rotate
			//call function again (loopies)
		}
	
	return;
	}
}
