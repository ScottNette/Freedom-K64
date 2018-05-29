#include "mbed.h"
#include "SDFileSystem.h"
#include "FXOS8700Q.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


//Create an SDFileSystem object
SDFileSystem sd(PTE3, PTE1, PTE2, PTE4, "sd");

//Port Assignments________________________________________________________________________________________________
FXOS8700Q_acc acc( PTE25, PTE24, FXOS8700CQ_SLAVE_ADDR1); // Proper Ports and I2C Address for K64F Freedom board
FXOS8700Q_mag mag( PTE25, PTE24, FXOS8700CQ_SLAVE_ADDR1); // Proper Ports and I2C Address for K64F Freedom board

Serial pc(USBTX, USBRX);

//MotionSensor Data given in terms of gravity
MotionSensorDataUnits mag_data;
MotionSensorDataUnits acc_data;
//MotionSensor Data Raw - Meaning that it gives in non 'Gs'
MotionSensorDataCounts mag_raw;
MotionSensorDataCounts acc_raw;

FILE *fp_log;
FILE *fp_;


int main()
{
	pc.printf("Initializing... \r\n");
	//Mount the filesystem
	sd.mount();
	acc.enable(); //Enable Motion Sensor

	pc.printf("Done Initializing \r\n");


	fp_log = fopen("/sd/log.txt", "w+");
	fp_= fopen("/sd/testing.csv", "a+");
	if (fp_ == NULL) {
		fprintf(fp_log,"Unable to write the file \n");
		pc.printf("FAILED FAILED FAILED \r\n");
		fclose(fp_log);
		fclose(fp_);
		for(;;){}
	}
	else{
		fprintf(fp_log,"opened log %d \n", rand());
		fclose(fp_log);

	}


	fp_= fopen("/sd/testing.csv", "a+");

	pc.printf("SD opened, running IMU now \r\n");
	int count = 0;
	for(int ii = 0; ii < 1000; ii++){


		acc.getAxis(acc_data);
		mag.getAxis(mag_data);
		fprintf(fp_,"%1.5f,%1.5f,%1.5f, ", acc_data.x, acc_data.y, acc_data.z);
		fprintf(fp_,"%4.3f, %4.3f, %4.3f\r\n", mag_data.x, mag_data.y, mag_data.z);
		pc.printf("loop count = %d", ii);
		wait(.01);
		ii = ii +1;
		}
		fclose(fp_);


		pc.printf("Done Done \r\n");
	//Unmount the filesystem
	sd.unmount();
}
