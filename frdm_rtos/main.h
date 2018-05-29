#include "mbed.h"
#include "rtos.h"

//#include <math.h>
#include <SparkFunMPU9250-DMP.h>
#define MPU9250


osMutexId stdio_mutex;
osMutexDef(stdio_mutex);

MPU9250_DMP imu;

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);


typedef struct{
	float gx;
	float gy;
	float gz;
	float ax;
	float ay;
	float az;
	float mx;
	float my;
	float mz;

}sensorDataStruct;


MemoryPool<sensorDataStruct, 16> mpool;
Queue<sensorDataStruct, 16> queue;


Serial pc(USBTX, USBRX);


Serial gpsSerial(D1, D0);
InterruptIn IMU_interrupt(D8);


void parse(char *cmd);



#define GPSBAUD 9600

Thread *thread_set;
Thread thread1;
Thread thread2;
Thread thread3;
Thread thread4;

Timer deadt;
Timer deadt2;

#define GPS_BUFFER_SIZE 255
char cDataBuffer[GPS_BUFFER_SIZE];
char cDataBufferParse[GPS_BUFFER_SIZE];

volatile int rx_in=0;
volatile int rx_out=0;
volatile int buff_flag=0;
volatile int parseGPS=0;



void printIMUData(void);
void main_init();
void MPU9250Update();
