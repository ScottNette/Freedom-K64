#include <main.h>

/*
 *
 * osMutexWait(stdio_mutex, osWaitForever);
 * osMutexRelease(stdio_mutex);
 *
 *
 */

Timer timey;


void print_struct(sensorDataStruct *structIn){
    pc.printf("gx = %f, gy = %f, gz = %f \r\n", structIn->gx, structIn->gy, structIn->gz);
    pc.printf("ax = %f, ay = %f, az = %f \r\n", structIn->ax, structIn->ay, structIn->az);
    pc.printf("mx = %f, my = %f, mz = %f \r\n", structIn->mx, structIn->my, structIn->mz);

    pc.printf("----------\r\n\n\n");
}

void MPU9250Update(){
	imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
	 pc.printf("updated \r\n\n");
}

void sensors_thread() {
    int ii = 1;
    //sensorDataStruct *sensorData;
   // sensorData = mpool.alloc();

    int cur_time;
     int previous;
    //timey.start();
    pc.printf("started sensors \r\n\n");
    while (true)
    {



#if 0
        sensorData->gx = (ii*.1)*11;
        sensorData->gy = (ii*.1)*22;
        sensorData->gz = (ii*.1)*33;

        sensorData->ax = (ii*1)*11;
        sensorData->ay = (ii*1)*22;
        sensorData->az = (ii*1)*33;

        sensorData->mx = (ii*10)*11;
        sensorData->my = (ii*10)*22;
        sensorData->mz = (ii*10)*33;


        pc.printf("Got sensor data!-ddd-");
        print_struct(sensorData);
        ii++;
        queue.put(sensorData);
#endif

			// Check for new data in the FIFO
		if ( imu.fifoAvailable() )
		{

			// Use dmpUpdateFifo to update the ax, gx, mx, etc. values
			if ( imu.dmpUpdateFifo() == INV_SUCCESS)
			{
				 pc.printf("im available \r\n\n");
				// cur_time = timey.read_us();
				//	 pc.printf("Took %dus\r\n", cur_time - previous);
				//	 previous = cur_time;
			  // computeEulerAngles can be used -- after updating the
			  // quaternion values -- to estimate roll, pitch, and yaw
			 // /imu.computeEulerAngles();
			  //printIMUData();
			}
		}
		imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
		Thread:wait(200);
    }
}

//void IMU_ISR

void quatern_thread() {
    int count = 0;
    sensorDataStruct *QuadData_in;
    QuadData_in = mpool.alloc();

#if 1
    QuadData_in->gx = 0;
    QuadData_in->gy = 0;
    QuadData_in->gz = 0;

    QuadData_in->ax = 0;
    QuadData_in->ay = 0;
    QuadData_in->az = 0;

    QuadData_in->mx = 0;
    QuadData_in->my = 0;
    QuadData_in->mz = 0;
#endif

    while (true) {
        //osMutexWait(stdio_mutex, osWaitForever);
        // pc.printf("doin quads \r\n");
        // osMutexRelease(stdio_mutex);

         osEvent evt = queue.get(0);
         if (evt.status == osEventMessage) {
              QuadData_in = (sensorDataStruct*)evt.value.p;
         }

#if 0
        QuadData_in->gx = QuadData_in->gx +1; //* (rand() % 1000)/1000;
        QuadData_in->gy = QuadData_in->gy+1; // * (rand() % 1000)/1000;
        QuadData_in->gz = QuadData_in->gz+1; // * (rand() % 1000)/1000;

        QuadData_in->ax = QuadData_in->ax+1; // * (rand() % 1000)/1000;
        QuadData_in->ay = QuadData_in->ay+1; // * (rand() % 1000)/1000;
        QuadData_in->az = QuadData_in->az+1; // * (rand() % 1000)/1000;

        QuadData_in->mx = QuadData_in->mx+1; // * (rand() % 1000)/1000;
        QuadData_in->my = QuadData_in->my +1; //* (rand() % 1000)/1000;
        QuadData_in->mz = QuadData_in->mz+1; // * (rand() % 1000)/1000;

        if (count >= 5)
        {
            pc.printf("count to %i -----------------------------------------\r\n", count);
            print_struct(QuadData_in);
            count = 0;
            mpool.free(QuadData_in);
        }
        count++;
#endif
       // Thread::wait(1000);

   	 if (parseGPS == 1)
   	 {

   		 //pc.printf("%s",cDataBufferParse);
   		 //pc.printf("Parsed\r\n");
   		 parse(cDataBufferParse);
   		 parseGPS = 0;
   	 }
   	 Thread::wait(3000);
    }
}




void heartbeat() {
    while (true) {

//      led1 = !led1;
//      led3 = !led3;
        led2 = !led2;
        Thread::wait(500);
    }
}

void deadTimer() {
    deadt.start();
    while (true) {

        Thread::wait(5000);
        pc.printf("thread t = %f------------------------------------------------------------", deadt.read());

    }
}


int main()
{
int count = 0;


    main_init();
    thread3.start(callback(heartbeat));         thread3.set_priority(osPriorityAboveNormal);
    wait(3);

#if 1
    thread1.start(callback(sensors_thread));    thread1.set_priority(osPriorityRealtime);
   // thread2.start(callback(quatern_thread));    thread2.set_priority(osPriorityRealtime);



#endif
    #if 0
    deadt2.start();
    thread4.start(callback(deadTimer));         thread4.set_priority(osPriorityNormal);

    wait(10);

    pc.printf("main thread t = %f,  \r\n  thread t = %f", deadt2.read(), deadt.read());
#endif

    //RtosTimer heatbeat_timer(heartbeat, osTimerPeriodic, (void *)0);
    //RtosTimer sensor_timer(sensors_thread2, osTimerPeriodic,  (void *)0);

    //heatbeat_timer.start(500);
    //sensor_timer.start(3000);

    //IMU_interrupt.rise(&MPU9250Update);
#if 1
    pc.printf("In loop\r\n");
     while(1){
    };

#endif
}


void printIMUData(void)
{  
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
  //float q0 = imu.calcQuat(imu.qw);
  //float q1 = imu.calcQuat(imu.qx);
  //float q2 = imu.calcQuat(imu.qy);
  //float q3 = imu.calcQuat(imu.qz);

  //osMutexWait(stdio_mutex, osWaitForever);

  //pc.printf("Q: q0 = %f, q1 = %f, q2 = %f, q3 = %f\r\n" , q0,q1,q2,q3);
  pc.printf("R/P/Y: %f, %f, %f\r\n" , imu.roll, imu.pitch, imu.yaw);
  //pc.printf("Time: %f ms\r\n\n", imu.time);
  
  
  float accelX = imu.calcAccel(imu.ax);
  float accelY = imu.calcAccel(imu.ay);
  float accelZ = imu.calcAccel(imu.az);
  //float gyroX = imu.calcGyro(imu.gx);
  //float gyroY = imu.calcGyro(imu.gy);
  //float gyroZ = imu.calcGyro(imu.gz);
  //float magX = imu.calcMag(imu.mx);
  //float magY = imu.calcMag(imu.my);
  //float magZ = imu.calcMag(imu.mz);
  
  pc.printf("Accel: %f %f %f g\r\n", accelX, accelY, accelZ);
  //pc.printf("gyro: %f %f %f dps\r\n", gyroX, gyroY, gyroZ);
  //pc.printf("mag: %f %f %f uT\r\n\n", magX, magY, magZ);
  pc.printf("----------------------\r\n\n");
  //osMutexRelease(stdio_mutex);
  
}
void parse(char *cmd)
{

    char ns, ew, tf, status;
    int fq, nst, fix, date;                                     // fix quality, Number of satellites being tracked, 3D fix
    float latitude, longitude, timefix, speed, altitude;

    osMutexWait(stdio_mutex, osWaitForever);
    // Global Positioning System Fix Data
    if(strncmp(cmd,"$GNGGA", 6) == 0)
    {

        sscanf(cmd, "$GNGGA,%f,%f,%c,%f,%c,%d,%d,%*f,%f", &timefix, &latitude, &ns, &longitude, &ew, &fq, &nst, &altitude);
        pc.printf("GNGGA Fix taken at: %f, Latitude: %f %c, Longitude: %f %c, Fix quality: %d, Number of sat: %d, Altitude: %f M\n", timefix, latitude, ns, longitude, ew, fq, nst, altitude);
    }

    // Satellite status
    if(strncmp(cmd,"$GNGSA", 6) == 0)
    {
        sscanf(cmd, "$GNGSA,%c,%d,%d", &tf, &fix, &nst);
        pc.printf("GNGSA Type fix: %c, 3D fix: %d, number of sat: %d\r\n", tf, fix, nst);
    }

    // Geographic position, Latitude and Longitude
    if(strncmp(cmd,"$GNGLL", 6) == 0)
    {
        sscanf(cmd, "$GNGLL,%f,%c,%f,%c,%f", &latitude, &ns, &longitude, &ew, &timefix);
        pc.printf("GNGLL Latitude: %f %c, Longitude: %f %c, Fix taken at: %f\n", latitude, ns, longitude, ew, timefix);
    }

    // Geographic position, Latitude and Longitude
    if(strncmp(cmd,"$GNRMC", 6) == 0)
    {
        sscanf(cmd, "$GNRMC,%f,%c,%f,%c,%f,%c,%f,,%d", &timefix, &status, &latitude, &ns, &longitude, &ew, &speed, &date);
        pc.printf("GNRMC Fix taken at: %f, Status: %c, Latitude: %f %c, Longitude: %f %c, Speed: %f, Date: %d\n", timefix, status, latitude, ns, longitude, ew, speed, date);
    }
    osMutexRelease(stdio_mutex);
}


void GpsSerialIsr(void)
{/**
 *\brief Interrupt handler for serial Rx
         set the event for the serial task
*/
	char c;
	 c = gpsSerial.getc();

	if((c == '$')  || (buff_flag == 1));           // wait a $
	{
		cDataBuffer[rx_in] = c;
		rx_in++;
		buff_flag = 1;

		if( c == '\n' )
		{
			cDataBuffer[rx_in] = '\0';
			strncpy(cDataBufferParse, cDataBuffer, GPS_BUFFER_SIZE);
			buff_flag = 0;
			parseGPS = 1;
			rx_in = 0 ;
			//pc.printf("Dont interrupt me \r\n");
		}
	}

	//led3 = !led3;
}

void main_init()
{
    led1 = 1;
    led2 = 1;
    led3 = 1;


    // Create mutex for stdio
    stdio_mutex = osMutexCreate(osMutex(stdio_mutex));
    pc.baud (115200);
    wait(1);
    pc.printf("Starting program \r\n\n");

    //gpsSerial.attach(&GpsSerialIsr, Serial::RxIrq);  // Start serial interrupt
    //gpsSerial.attach(&GpsSerialIsr, Serial::RxIrq);  // Start serial interrupt



#if 1
     // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      pc.printf("Unable to communicate with MPU-9250");

    }
  }
  pc.printf("success\r\n");
  //imu.begin();

   imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              100); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
    
    
  // Enable all sensors, and set sample rates to 4Hz.
  // (Slow so we can see the interrupt work.)
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  // The sample rate of the accel/gyro can be set using
    // setSampleRate. Acceptable values range from 4Hz to 1kHz
  imu.setSampleRate(300); // Set accel/gyro sample rate to 4Hz


  // Likewise, the compass (magnetometer) sample rate can be
  // set using the setCompassSampleRate() function.
  // This value can range between: 1-100Hz
  imu.setCompassSampleRate(25); // Set mag rate


  // Use enableInterrupt() to configure the MPU-9250's 
  // interrupt output as a "data ready" indicator.
  imu.enableInterrupt();

  // The interrupt level can either be active-high or low.
  // Configure as active-low, since we'll be using the pin's
  // internal pull-up resistor.
  // Options are INT_ACTIVE_LOW or INT_ACTIVE_HIGH
 imu.setIntLevel(INT_ACTIVE_LOW);

  // The interrupt can be set to latch until data has
  // been read, or to work as a 50us pulse.
  // Use latching method -- we'll read from the sensor
  // as soon as we see the pin go LOW.
  // Options are INT_LATCHED or INT_50US_PULSE
  imu.setIntLatched(INT_LATCHED);
  
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(250); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(4); // Set accel to +/-2g
  // Note: the MPU-9250's magnetometer FSR is set at 
  // +/- 4912 uT (micro-tesla's)

  // setLPF() can be used to set the digital low-pass filter
  // of the accelerometer and gyroscope.
  // Can be any of the following: 188, 98, 42, 20, 10, 5
  // (values are in Hz).
  imu.setLPF(10); // Set LPF corner frequency to 5Hz


  pc.printf("connected \r\n\n");

#endif


}
