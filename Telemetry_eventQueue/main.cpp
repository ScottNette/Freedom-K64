#include "main.h"



void updateIMU_IRQ()
{
    newIMUData = true;
    queueMain.event(&serviceIMU);
}

void updateGPS_IRQ()
{
    queueMain.event(&serviceGPS);
}


//-----------------------------------
void serviceIMU()
{
    if(newIMUData)
    {
        newIMUData = false;
        gpioIMU_D9 = 1;
        mpu9250.readAccelData();  // Read the x/y/z adc values
        mpu9250.readGyroData();  // Read the x/y/z adc values
        mpu9250.readMagData();  // Read the x/y/z adc values

   // Now we'll calculate the accleration value into actual g's
        mpu9250.ax = ((float)mpu9250.accelCount[0])*mpu9250.aRes - mpu9250.accelBias[0]; // get actual g value, this depends on scale being set
        mpu9250.ay = ((float)mpu9250.accelCount[1])*mpu9250.aRes - mpu9250.accelBias[1];
        mpu9250.az = ((float)mpu9250.accelCount[2])*mpu9250.aRes - mpu9250.accelBias[2];

        // Calculate the gyro value into actual degrees per second
        mpu9250.gx = ((float)mpu9250.gyroCount[0])*mpu9250.gRes - mpu9250.gyroBias[0];  // get actual gyro value, this depends on scale being set
        mpu9250.gy = ((float)mpu9250.gyroCount[1])*mpu9250.gRes - mpu9250.gyroBias[1];
        mpu9250.gz = ((float)mpu9250.gyroCount[2])*mpu9250.gRes - mpu9250.gyroBias[2];

        // Calculate the magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental corrections
        mpu9250.mx = ((float)mpu9250.magCount[0])*mpu9250.mRes*mpu9250.magCalibration[0] - mpu9250.magbias[0];  // get actual magnetometer value, this depends on scale being set
        mpu9250.my = ((float)mpu9250.magCount[1])*mpu9250.mRes*mpu9250.magCalibration[1] - mpu9250.magbias[1];
        mpu9250.mz = ((float)mpu9250.magCount[2])*mpu9250.mRes*mpu9250.magCalibration[2] - mpu9250.magbias[2];

        gpioIMU_D9 = 0;
        
    }
    
}
//-----------------------------------
void serivceGPS()
{
    gpioGPS_D10 = 1;
    while(gpsSerial.readable())
    {
        gps.parse(gpsSerial.getc());
    }
    gpioGPS_D10 = 0;
}
//-----------------------------------    
void serviceQuat()
{
    gpioQuat_D11 = 1;
    mpu9250.MadgwickQuaternionUpdate(mpu9250.ax, mpu9250.ay, mpu9250.az, mpu9250.gx*PI/180.0f, mpu9250.gy*PI/180.0f, mpu9250.gz*PI/180.0f,  mpu9250.my,  mpu9250.mx, mpu9250.mz);
    // mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);
    gpioQuat_D11 = 0;
    
}

//-----------------------------------
void servicePrint()
{
    gpioPrint_D12 = 1;
    pc.printf(gps.print("ALL"));
    
    pc.printf("ax = %f", 1000.0*mpu9250.ax);
    pc.printf(" ay = %f", 1000.0*mpu9250.ay);
    pc.printf(" az = %f  mg\n\r", 1000.0*mpu9250.az);

    pc.printf("gx = %f", mpu9250.gx);
    pc.printf(" gy = %f",mpu9250. gy);
    pc.printf(" gz = %f  deg/s\n\r", mpu9250.gz);

//  pc.printf("gx = %f", mx);
//  pc.printf(" gy = %f", my);
//  pc.printf(" gz = %f  mG\n\r", mz);

    pc.printf(" temperature = %f  C\n\r", mpu9250.readTempData());  // Read the adc values
    
    pc.printf("Yaw, Pitch, Roll: %f %f %f\n\r", mpu9250.yaw, mpu9250.pitch, mpu9250.roll);
                        
    gpioPrint_D12 = 0;
}

void serviceHeartbeat()
{
    //      led1 = !led1;
//      led3 = !led3;
        led2 = !led2;
}

void serviceSD()
{
    
    gpioSD_D13 = 1;
    sd.mount();
    
    
    fp_log = fopen("/sd/log.txt", "w+");
    fp_= fopen("/sd/testing.csv", "a+");
    if (fp_ == NULL) {
        fprintf(fp_log,"Unable to write the file \n");
        pc.printf("FAILED FAILED FAILED \r\n");
        fclose(fp_log);
        for(;;){}
    }
    else{
        fprintf(fp_log,"opened log %d \n", rand());
        fclose(fp_log);

    }


    pc.printf("SD opened, running IMU now \r\n");
    
        fprintf(fp_,"printed to SD Card");
        wait(.1);
        fclose(fp_);


        pc.printf("Done Done \r\n");
    //Unmount the filesystem
    sd.unmount();
        dataStructIdx = 0;
    
        gpioSD_D13 = 0;
    
        
        //queuePrint.event(&servicePrint);
    
}

void saveData()
{
    /*
    sensorDataStruct[dataStructIdx].ax = mpu9250.ax* float2int16;
    sensorDataStruct[dataStructIdx].ay = mpu9250.ay* float2int16;
    sensorDataStruct[dataStructIdx].az = mpu9250.az* float2int16;
    
    sensorDataStruct[dataStructIdx].mx = mpu9250.mx* float2int16;
    sensorDataStruct[dataStructIdx].my = mpu9250.my* float2int16;
    sensorDataStruct[dataStructIdx].mz = mpu9250.mz* float2int16;
    
    sensorDataStruct[dataStructIdx].roll = mpu9250.roll* float2int16;
    sensorDataStruct[dataStructIdx].pitch = mpu9250.pitch* float2int16;
    sensorDataStruct[dataStructIdx].yaw = mpu9250.yaw* float2int16;
    
    sensorDataStruct[dataStructIdx].gpsLat = gps.gpsLat* float2int16;
    sensorDataStruct[dataStructIdx].gpsLong = gps.gpsLong* float2int16;
    sensorDataStruct[dataStructIdx].gpsSpeed = gps.gpsSpeed* float2int16;
    sensorDataStruct[dataStructIdx].gpsHeading = gps.gpsHeading* float2int16;
     */       

    
//    sampledData[dataStructIdx][] = sprintf("%1.6f,%1.6f,%1.6f,%1.6f,%1.6f,%1.6f,%3.2f,%3.2f,%3.2f,%3.5,%3.5,%3.2f,%3.2f",mpu9250.ax,mpu9250.ay ...
//            mpu9250.az, mpu9250.mx, mpu9250.my, mpu9250.mz, mpu9250.roll, mpu9250.pitch, mpu9250.yaw, gps.gpsLat, gps.gpsLong, gps.gpsSpeed, gps.gpsHeading);
    
    dataStructIdx++;
    
    
}


//void queueIMU()
//{
//
//}
//
//void queueGPS()
//{
//
//}
//
//void queuePrint()
//{
//
//}
//
//void queueQuat()
//{
//
//}





int main()
{
    Initmain();
    
#if measureTimingFalse
    
    IMU_interrupt.rise(&updateIMU_IRQ);
    
    //setup GPS ISR
    //IMU_interrupt.rise(&updateIMU_IRQ);
    
    
    queueThread.start(callback(&queueMain, &EventQueue::dispatch_forever));
    printThread.start(callback(&queuePrint, &EventQueue::dispatch_forever));
    
    //sysTick.attach_us(queueMain.event(&serviceIMU), 100.0f);
    sysTick.attach_us(queueMain.event(&serviceGPS), 100.0f);
    sysTick.attach_us(queueMain.event(&serviceQuat), 100.0f);
    sysTick.attach(queueMain.event(&serviceHeartbeat), 0.5);
    sysTick.attach(queueMain.event(&saveData), 0.05f);
    //sysTick.attach(queuePrint.event(&servicePrint), 2);
    sysTick.attach(queueMain.event(&servicePrint), 2);
    
    osThreadSetPriority(osThreadGetId(), osPriorityIdle);
#else
    wait(1);
    
    serviceIMU();
    serviceGPS();
    serviceQuat();
    servicePrint();
    
    saveData();
    
    pc.printf("Done\r\n");
    
    
#endif

    
    while (true)
    {
        
        //serviceGPS();
        
        //sysCount++;
    }
    
    
 // wait(osWaitForever);

}



    
    
// the address of the object, member function, and interval
    //sysTick.attach(callback(&f, &Flipper::flip), 2.0); 
