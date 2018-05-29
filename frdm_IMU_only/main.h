#include "mbed.h"
#include "MPU9250.h"

MPU9250 mpu9250;

DigitalOut myled(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);


DigitalOut gpio1(D9);
DigitalOut gpio2(D10);
DigitalOut gpio3(D11);

Serial pc(USBTX, USBRX);






//InterruptIn IMU_interrupt(D8);

// threadSensor;
//Thread threadQuat;
//Thread threadPrint;
//Thread queuuethread;


//float sumSensorT, sumQuatT = 0;
//uint32_t sumCountSensorT, sumCountQuatT = 0;
//char buffer[14];

//int delt_t = 0; // used to control display output rate
//int count = 0;  // used to control display output rate

//   Timer t;


void InitIMU();
//serviceIMU();
void servicePrint();

void serviceIMU()
{

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
}

void serviceQuat()
{

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
