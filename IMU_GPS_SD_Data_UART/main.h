#include "mbed.h"
#include "MPU9250.h"
#include "ubloxM8N.h"
//#include "FastIO.h"
//#include "SDFileSystem.h"



#define GPS_BUFFER_SIZE 256


//Ticker sysTick;
//Timer sysTimer;

volatile uint32_t mainCount;

//---------------------//
//  Structure and class definitions
//---------------------//
MPU9250 mpu9250;
ubloxM8N gps;


typedef struct{
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t mx;
	int16_t my;
	int16_t mz;

	int16_t roll;
	int16_t pitch;
	int16_t yaw;

	int16_t gpsLat;
	int16_t gpsLong;
	int16_t gpsSpeed;
	int16_t gpsHeading;
	int16_t reserved;


}sensorDataStruct;

sensorDataStruct sensorData;
char cDataBuffer[GPS_BUFFER_SIZE];
char cDataBufferParse[GPS_BUFFER_SIZE];
char gpsString[75];


volatile int rx_in=0;
volatile int rx_out=0;
volatile int buff_flag=0;
volatile int parseGPS=0;

//char sampledData[50][50];
int dataStructIdx = 0;

//MemoryPool<sensorDataStruct, 64> mpool;
//Queue<sensorDataStruct, 64> queue;

FILE *fp_log;
FILE *fp_;



//---------------------//
// I/O's
//---------------------//
Serial pc(USBTX, USBRX); // tx, rx
Serial gpsSerial(D1, D0);
//Set up I2C, (SDA,SCL)
//I2C i2c(I2C_SDA, I2C_SCL);


//InterruptIn IMU_interrupt(D8);

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);

DigitalOut gpioIMU_D9(D9);
DigitalOut gpioGPS_D10(D10);
DigitalOut gpioQuat_D11(D11);
DigitalOut gpioPrint_D12(D12);
DigitalOut gpioSD_D13(D13);



//Create an SDFileSystem object
//SDFileSystem sd(PTE3, PTE1, PTE2, PTE4, "sd");

//  GVs
int sysCount = 0;
bool newIMUData = false;

//---------------------//
//  Function Prototypes
//---------------------//

void InitIMU();
void servicePrint();
void calcAngles();
void serviceGPS();

void initMain()
{
	pc.baud(115200);

	//Set up I2C
	i2c.frequency(400000);  // use fast (400 kHz) I2C
	pc.printf("CPU SystemCoreClock is %d Hz\r\n", SystemCoreClock);

	//t.start();

	InitIMU();

	gpsSerial.attach(&serviceGPS, Serial::RxIrq);
}


void InitIMU()
{

#define IMUdebug_print 1

	  uint8_t whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
	  pc.printf("I AM 0x%x\n\r", whoami); pc.printf("I SHOULD BE 0x71\n\r");

	  if (whoami == 0x71) // WHO_AM_I should always be 0x68
	  {
	    pc.printf("MPU9250 WHO_AM_I is 0x%x\n\r", whoami);
	    pc.printf("MPU9250 is online...\n\r");

	    sprintf(buffer, "0x%x", whoami);

	    wait(1);

	    mpu9250.resetMPU9250(); // Reset registers to default in preparation for device calibration
	    mpu9250.MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
	    pc.printf("x-axis self test: acceleration trim within : %f % of factory value\n\r", SelfTest[0]);
	    pc.printf("y-axis self test: acceleration trim within : %f % of factory value\n\r", SelfTest[1]);
	    pc.printf("z-axis self test: acceleration trim within : %f % of factory value\n\r", SelfTest[2]);
	    pc.printf("x-axis self test: gyration trim within : %f % of factory value\n\r", SelfTest[3]);
	    pc.printf("y-axis self test: gyration trim within : %f % of factory value\n\r", SelfTest[4]);
	    pc.printf("z-axis self test: gyration trim within : %f % of factory value\n\r", SelfTest[5]);
	    mpu9250.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
	    pc.printf("x gyro bias = %f\n\r", gyroBias[0]);
	    pc.printf("y gyro bias = %f\n\r", gyroBias[1]);
	    pc.printf("z gyro bias = %f\n\r", gyroBias[2]);
	    pc.printf("x accel bias = %f\n\r", accelBias[0]);
	    pc.printf("y accel bias = %f\n\r", accelBias[1]);
	    pc.printf("z accel bias = %f\n\r", accelBias[2]);
	    wait(2);
	    mpu9250.initMPU9250();
	    pc.printf("MPU9250 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
	    mpu9250.initAK8963(magCalibration);
	    pc.printf("AK8963 initialized for active data mode....\n\r"); // Initialize device for active mode read of magnetometer
	    pc.printf("Accelerometer full-scale range = %f  g\n\r", 2.0f*(float)(1<<Ascale));
	    pc.printf("Gyroscope full-scale range = %f  deg/s\n\r", 250.0f*(float)(1<<Gscale));
	    if(Mscale == 0) pc.printf("Magnetometer resolution = 14  bits\n\r");
	    if(Mscale == 1) pc.printf("Magnetometer resolution = 16  bits\n\r");
	    if(Mmode == 2) pc.printf("Magnetometer ODR = 8 Hz\n\r");
	    if(Mmode == 6) pc.printf("Magnetometer ODR = 100 Hz\n\r");
	    wait(1);
	   }
	   else
	   {
	    pc.printf("Could not connect to MPU9250: \n\r");
	    pc.printf("%#x \n",  whoami);

	    sprintf(buffer, "WHO_AM_I 0x%x", whoami);


	    while(1) ; // Loop forever if communication doesn't happen
	    }

	    mpu9250.getAres(); // Get accelerometer sensitivity
	    mpu9250.getGres(); // Get gyro sensitivity
	    mpu9250.getMres(); // Get magnetometer sensitivity
	    pc.printf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f/aRes);
	    pc.printf("Gyroscope sensitivity is %f LSB/deg/s \n\r", 1.0f/gRes);
	    pc.printf("Magnetometer sensitivity is %f LSB/G \n\r", 1.0f/mRes);
	    magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
	    magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
	    magbias[2] = +125.;  // User environmental x-axis correction in milliGauss
}

