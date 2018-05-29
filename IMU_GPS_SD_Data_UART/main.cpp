#include "main.h"


void serviceIMU()
{
	gpioIMU_D9 = 1;
	 mpu9250.readAccelData(accelCount);  // Read the x/y/z adc values
	// Now we'll calculate the accleration value into actual g's
	ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
	ay = (float)accelCount[1]*aRes - accelBias[1];
	az = (float)accelCount[2]*aRes - accelBias[2];

	mpu9250.readGyroData(gyroCount);  // Read the x/y/z adc values
	// Calculate the gyro value into actual degrees per second
	gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
	gy = (float)gyroCount[1]*gRes - gyroBias[1];
	gz = (float)gyroCount[2]*gRes - gyroBias[2];

	mpu9250.readMagData(magCount);  // Read the x/y/z adc values
	// Calculate the magnetometer values in milliGauss
	// Include factory calibration per data sheet and user environmental corrections
	mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
	my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];
	mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];
	gpioIMU_D9 = 0;
}

void servicePrint()
{
	gpioPrint_D12 = 1;
    pc.printf("ax = %f", 1000*ax);
    pc.printf(" ay = %f", 1000*ay);
    pc.printf(" az = %f  mg\n\r", 1000*az);

    pc.printf("gx = %f", gx);
    pc.printf(" gy = %f", gy);
    pc.printf(" gz = %f  deg/s\n\r", gz);

    pc.printf("gx = %f", mx);
    pc.printf(" gy = %f", my);
    pc.printf(" gz = %f  mG\n\r", mz);

 //   tempCount = mpu9250.readTempData();  // Read the adc values
 //   temperature = ((float) tempCount) / 333.87f + 21.0f; // Temperature in degrees Centigrade
 //   pc.printf(" temperature = %f  C\n\r", temperature);

//    pc.printf("q0 = %f\n\r", q[0]);
//    pc.printf("q1 = %f\n\r", q[1]);
//    pc.printf("q2 = %f\n\r", q[2]);
//    pc.printf("q3 = %f\n\r", q[3]);

    calcAngles();

    pc.printf("Yaw, Pitch, Roll: %f %f %f\n\r", yaw, pitch, roll);
    pc.printf("average rate = %f\n\r", (float) sumCount/sum);


    count = t.read_ms();


	pc.printf("lat, long, alt = %f,%f,%f\r\n", gps.latitude,  gps. longitude, gps.altitude);
	memcpy( gpsString, cDataBuffer, 55 );
	pc.printf("%s\r\n\r\n",gpsString);


	if(count > 1<<21)
	{
		t.start(); // start the timer over again if ~30 minutes has passed
		count = 0;
		deltat= 0;
		lastUpdate = t.read_us();
	}
	sum = 0;
	sumCount = 0;
	gpioPrint_D12 = 0;
}


void serviceQuat()
{

	gpioQuat_D11 = 1;
	Now = t.read_us();
	deltat = (float)((Now - lastUpdate)/1000000.0f) ; // set integration time by time elapsed since last filter update
	lastUpdate = Now;

	sum += deltat;
	sumCount++;

	//    if(lastUpdate - firstUpdate > 10000000.0f) {
	//     beta = 0.04;  // decrease filter gain after stabilized
	//     zeta = 0.015; // increasey bias drift gain after stabilized
	//   }

	// Pass gyro rate as rad/s
	//  mpu9250.MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
	mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);
	gpioQuat_D11 = 0;
}

void calcAngles()
{
	  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
	  // In this coordinate system, the positive z-axis is down toward Earth.
	  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
	  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
	  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
	  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
	  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
	  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
	  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.

	yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
	pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
	roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
	pitch *= 180.0f / PI;
	yaw   *= 180.0f / PI;
	yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
	roll  *= 180.0f / PI;
}

#if 1
void serviceGPS()
{
    gpioGPS_D10 = 1;
    char c;
    while(gpsSerial.readable())
    {
		c = gpsSerial.getc();

		if(c == '$')  // wait a $
		{
			rx_in = 0;
			buff_flag = 1;
		}

		if (buff_flag == 1);
		{
			cDataBuffer[rx_in] = c;
			rx_in++;


			if( c == '\n' )
			{
				cDataBuffer[rx_in] = '\0';
				strncpy(cDataBufferParse, cDataBuffer, GPS_BUFFER_SIZE);
				buff_flag = 0;
				parseGPS = 1;
				rx_in = 0 ;
				gps.parse(cDataBufferParse);
				//pc.printf("Dont interrupt me \r\n");
			}
		}
	}

    gpioGPS_D10 = 0;
}

#endif

void serviceHeartbeat()
{
	//gpioSD_D13 = 1;
	gpioSD_D13 = !gpioSD_D13;
	led1 = !led1;
	//      led3 = !led3;
	        led2 = !led2;

	//gpioSD_D13 = 0;
}


int main()
{

	initMain();


	//-----------------------------------------------------------------------------------------------//
	while(1)
	{


		if((mainCount % 1000) == 0)
		{
			serviceQuat();
		}
		if((mainCount % 20000) == 0)
		{
			serviceIMU();
		}
		if ((mainCount % 1000000) == 0)
		{
			serviceHeartbeat();
			servicePrint();
		}

		mainCount++;
		if (mainCount == 100000000)
		{
			mainCount =1;
		}
	}
}




