/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ubloxM8N.h
 * Author: scott
 *
 * Created on March 16, 2018, 7:53 AM
 */

#ifndef UBLOXM8N_H
#define UBLOXM8N_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class MPU9250 {
 
    protected:
 


    public:
        char ns, ew, tf, status;
        int fq, nst, fix, date;                                     // fix quality, Number of satellites being tracked, 3D fix
        float latitude, longitude, timefix, speed, altitude;
        
        //void parse(char *cmd);
        //void print();
    
    
    void parse(char *cmd)
    {
        
        // Global Positioning System Fix Data
        if(strncmp(cmd,"$GNGGA", 6) == 0)
        {

            sscanf(cmd, "$GNGGA,%f,%f,%c,%f,%c,%d,%d,%*f,%f", &timefix, &latitude, &ns, &longitude, &ew, &fq, &nst, &altitude);
         }

        // Satellite status
        if(strncmp(cmd,"$GNGSA", 6) == 0)
        {
            sscanf(cmd, "$GNGSA,%c,%d,%d", &tf, &fix, &nst);

        }

        // Geographic position, Latitude and Longitude
        if(strncmp(cmd,"$GNGLL", 6) == 0)
        {
            sscanf(cmd, "$GNGLL,%f,%c,%f,%c,%f", &latitude, &ns, &longitude, &ew, &timefix);

        }

        // Geographic position, Latitude and Longitude
        if(strncmp(cmd,"$GNRMC", 6) == 0)
        {
            sscanf(cmd, "$GNRMC,%f,%c,%f,%c,%f,%c,%f,,%d", &timefix, &status, &latitude, &ns, &longitude, &ew, &speed, &date);
        }

    }
};

#if 0
char print(char *cmd)
{
    
    char strOut[150];
    
    char latitudeStr[7];
    char longitudeStr[7];
    char speedStr[7];
    char altitudeStr[7];
    char timefixStr[7];



    ftoa(latitudeStr, latitude, 4 );
    ftoa(longitudeStr, longitude, 4 );
    ftoa(speedStr, speed, 4 );
    ftoa(altitudeStr, altitude, 4 );
    ftoa(timefixStr, timefix, 4 );


    if(strncmp(cmd,"ALL", 3) == 0)
    {
    	//dtostrf( latitude, 3, 4, latitudeStr );
    	strOut = sprintf("%i, %s, %s, %s, %s, %s, %s \r\n",fix,latitudeStr,longitudeStr, speedStr,altitudeStr,timefixStr);
    }

    
    if(strncmp(cmd,"$GNGGA", 6) == 0)
    {
       //$GNGGA
       strOut = sprintf("GNGGA Fix taken at: %f, Latitude: %f %c, Longitude: %f %c, Fix quality: %d, Number of sat: %d, Altitude: %f M\n", timefix, latitude, ns, longitude, ew, fq, nst, altitude);
    }
    
     if(strncmp(cmd,"$GNGSA", 6) == 0)
    {
        //GNGSA
        strOut = sprintf("GNGSA Type fix: %c, 3D fix: %d, number of sat: %d\r\n", tf, fix, nst);
    }
    
    if(strncmp(cmd,"$GNGLL", 6) == 0)
    {
        //GNGLL
         strOut = sprintf("GNGLL Latitude: %f %c, Longitude: %f %c, Fix taken at: %f\n", latitude, ns, longitude, ew, timefix);
    }
    
     if(strncmp(cmd,"$GNRMC", 6) == 0)
    {
        // GNRMC
        strOut = sprintf("GNRMC Fix taken at: %f, Status: %c, Latitude: %f %c, Longitude: %f %c, Speed: %f, Date: %d\n", timefix, status, latitude, ns, longitude, ew, speed, date);
     }

    
    return strOut;

}

char *ftoa(char *a, double f, int precision)
{
 long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};

 char *ret = a;
 long heiltal = (long)f;
 itoa(heiltal, a);
 while (*a != '\0') a++;
 *a++ = '.';
 long desimal = abs((long)((f - heiltal) * p[precision]));
 itoa(desimal, a, 10);
 return ret;
}

void itoa(int n, char s[])
{
    int i, sign;

    if ((sign = n) < 0)  /* record sign */
        n = -n;          /* make n positive */
    i = 0;
    do {       /* generate digits in reverse order */
        s[i++] = n % 10 + '0';   /* get next digit */
    } while ((n /= 10) > 0);     /* delete it */
    if (sign < 0)
        s[i++] = '-';
    s[i] = '\0';
    reverse(s);
}

void reverse(char s[])
{
    int i, j;
    char c;

    for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
        c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
}

#endif
#endif /* UBLOXM8N_H */

