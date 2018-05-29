#include "mbed.h"
#include "rtos.h"
//#include "mbed_events.h"
#include "MPU9250.h"
#include "ubloxM8N.h"
//#include "FastIO.h"
#include "SDFileSystem.h"



#define MODSERIAL_DEFAULT_RX_BUFFER_SIZE 512
#define MODSERIAL_DEFAULT_TX_BUFFER_SIZE 1024 
#include "MODSERIAL.h"



#define measureTimingFalse 0
#define float2int16 
//----------------------------------------------------------------------------//
// -------  Define mutex, senaphores, etc... -----------//
//----------------------------------------------------------------------------//
osMutexId stdio_mutex;
osMutexDef(stdio_mutex);

Thread printfThread;
Thread queueThread;
        
EventQueue queueMain(32 * EVENTS_EVENT_SIZE);
EventQueue queuePrint;



Ticker sysTick;



//---------------------//
//  Structure and class definitions
//---------------------//
MPU9250_DMP imu;
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
        
        
}sensorDataStruct;

sensorDataStruct[50];

sampledData[50][50];
dataStructIdx = 0;

MemoryPool<sensorDataStruct, 16> mpool;
Queue<sensorDataStruct, 16> queue;




//---------------------//
// I/O's
//---------------------//
MODSERIAL pc(USBTX, USBRX); // tx, rx
MODSERIAL gpsSerial(D1, D0);
//Set up I2C, (SDA,SCL)
I2C i2c(I2C_SDA, I2C_SCL);

InterruptIn IMU_interrupt(D8);

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);

DigitalOut gpioIMU_D9(D9);
DigitalOut gpioGPS_D10(D10);
DigitalOut gpioQuat_D11(D11);
DigitalOut gpioPrint_D12(D12);
DigitalOut gpioSD_D13(D13);


//FastOut<D9> gpioIMU_D9;
//FastOut<D10> gpioGPS_D10;
//FastOut<D11> gpioQuat_D11;
//FastOut<D12> gpioPrint_D12;
//FastOut<D13> gpioSD_D13



//Create an SDFileSystem object
SDFileSystem sd(PTE3, PTE1, PTE2, PTE4, "sd");

//  GVs
int sysCount = 0;
bool newIMUData = false;


//---------------------//
//  Function Prototypes
//---------------------//
void parse(char *cmd);

void serviceQuat();
void serviceIMU();
void serviceGPS();
void serivcePrint();
void serviceHeartbeat();
void serviceSD();

void main_init();
void MPU9250Update();

void InitMain()
{
	 pc.baud(115200);

	  //Set up I2C
	  i2c.frequency(400000);  // use fast (400 kHz) I2C
	  pc.printf("CPU SystemCoreClock is %d Hz\r\n", SystemCoreClock);

	  InitIMU();

}

void InitIMU()
{

#define IMUdebug_print 1

	uint8_t whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250

#if IMUdebug_print
	// Read the WHO_AM_I register, this is a good test of communication
	  pc.printf("I AM 0x%x\n\r", whoami); pc.printf("I SHOULD BE 0x71\n\r");
#endif

	if (whoami == 0x71) // WHO_AM_I should always be 0x68
	{
		mpu9250.resetMPU9250(); // Reset registers to default in preparation for device calibration
		mpu9250.MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
		wait(1);
		mpu9250.calibrateMPU9250(); // Calibrate gyro and accelerometers, load biases in bias registers
		wait(2);
		mpu9250.initMPU9250();
		mpu9250.initAK8963();

	#if IMUdebug_print
		pc.printf("MPU9250 WHO_AM_I is 0x%x\n\r", whoami);
		pc.printf("MPU9250 is online...\n\r");

		pc.printf("x-axis self test: acceleration trim within : %f % of factory value\n\r", SelfTest[0]);
		pc.printf("y-axis self test: acceleration trim within : %f % of factory value\n\r", SelfTest[1]);
		pc.printf("z-axis self test: acceleration trim within : %f % of factory value\n\r", SelfTest[2]);
		pc.printf("x-axis self test: gyration trim within : %f % of factory value\n\r", SelfTest[3]);
		pc.printf("y-axis self test: gyration trim within : %f % of factory value\n\r", SelfTest[4]);
		pc.printf("z-axis self test: gyration trim within : %f % of factory value\n\r", SelfTest[5]);

		pc.printf("x gyro bias = %f\n\r", mpu9250.gyroBias[0]);
		pc.printf("y gyro bias = %f\n\r", mpu9250.gyroBias[1]);
		pc.printf("z gyro bias = %f\n\r", mpu9250.gyroBias[2]);
		pc.printf("x accel bias = %f\n\r", mpu9250.accelBias[0]);
		pc.printf("y accel bias = %f\n\r", mpu9250.accelBias[1]);
		pc.printf("z accel bias = %f\n\r", mpu9250.accelBias[2]);

		pc.printf("MPU9250 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

		pc.printf("AK8963 initialized for active data mode....\n\r"); // Initialize device for active mode read of magnetometer
		pc.printf("Accelerometer full-scale range = %f  g\n\r", 2.0f*(float)(1<<Ascale));
		pc.printf("Gyroscope full-scale range = %f  deg/s\n\r", 250.0f*(float)(1<<Gscale));
		if(Mscale == 0) pc.printf("Magnetometer resolution = 14  bits\n\r");
		if(Mscale == 1) pc.printf("Magnetometer resolution = 16  bits\n\r");
		if(Mmode == 2) pc.printf("Magnetometer ODR = 8 Hz\n\r");
		if(Mmode == 6) pc.printf("Magnetometer ODR = 100 Hz\n\r");
		wait(1);

		mpu9250.getAres(); // Get accelerometer sensitivity
		mpu9250.getGres(); // Get gyro sensitivity
		mpu9250.getMres(); // Get magnetometer sensitivity
		pc.printf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f/mpu9250.aRes);
		pc.printf("Gyroscope sensitivity is %f LSB/deg/s \n\r", 1.0f/mpu9250.gRes);
		pc.printf("Magnetometer sensitivity is %f LSB/G \n\r", 1.0f/mpu9250.mRes);

	#endif

//		magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
//		magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
//		magbias[2] = +125.;  // User environmental x-axis correction in milliGauss

		// Enable int
    	mpu9250.writeByte(0x68<<1, 0x38, 1);
    	wait(0.1);
    	// Act low ( 0 for act high)
    	mpu9250.writeByte(0x68<<1, 0x37, 0x81);
	}
	else
	{
	pc.printf("Could not connect to MPU9250: \n\r");
	pc.printf("%#x \n",  whoami);
	while(1) ; // Loop forever if communication doesn't happen
	}

}
