#include "mbed.h"
Serial pc(USBTX, USBRX);

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
InterruptIn mybutton(BUTTON2);

volatile int lightOn[3] = {0,1,1};

void button_pressed() {

	int temp ;
	temp = lightOn[0];
	lightOn[0] = lightOn[2];
	lightOn[2] = lightOn[1];
	lightOn[1] = temp;


	pc.printf("led1 = %d\r\n",lightOn[0]);
	pc.printf("led2 = %d\r\n",lightOn[1]);
	pc.printf("led3 = %d\r\n",lightOn[2]);
	pc.printf("\r\n");
}


// main() runs in its own thread in the OS
int main() {

	led1 = 0;
	led2 = 0;
	led3 = 0;
	mybutton.fall(&button_pressed);

    while (true) {


        led1 = lightOn[0];
        led2 = lightOn[1];
        led3 = lightOn[2];
        wait(0.5);
    }
}

