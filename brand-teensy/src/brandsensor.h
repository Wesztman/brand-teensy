#ifndef BRANDSENSOR_H         // To make sure you don't declare the function more than once by including the header multiple times.
#define BRANDSENSOR_H

#include <Arduino.h>
#include <Adafruit_AMG88xx.h>


/*
Triggers Ultrasonic sensor and measures the travel time  of the returning sound wave.
Returns the distance in mm. Min 20mm, Max 2000mm.
Returns 0 if no pulse is found.
Max can be increased to 4000 if timeout is increased.
Input: triggerpin and echopin for the sensor
*/
int readUltraDist(int trig, int echo);

//Serial prints pixel values from IR Camera
void printIRCamera(float pixels[]);

//Return value from line sensor 0-1024
int readLineSensor(int linePin);

class Button_OP
{
   public:
	 Button_OP(int buttonPin);
	 
	 void init();
	 void update();
	 bool getState();
	 bool isPressed();


   private:
   	 int _buttonPin;
	 bool buttonState;
     bool lastButtonState;
     unsigned long lastDebounceTime = 0;
     unsigned long debounceDelay = 50;
};

//-----------Encoder description--------------
//                           _______         _______       
//               Pin1 ______|       |_______|       |______ Pin1
// negative <---         _______         _______         __      --> positive
//               Pin2 __|       |_______|       |_______|   Pin2

		//	new	new	old	old
		//	pin2	pin1	pin2	pin1	Result
		//	----	----	----	----	------
		//	0	0	0	0	no movement
		//	0	0	0	1	+1
		//	0	0	1	0	-1
		//	0	0	1	1	+2  (assume pin1 edges only)
		//	0	1	0	0	-1
		//	0	1	0	1	no movement
		//	0	1	1	0	-2  (assume pin1 edges only)
		//	0	1	1	1	+1
		//	1	0	0	0	+1
		//	1	0	0	1	-2  (assume pin1 edges only)
		//	1	0	1	0	no movement
		//	1	0	1	1	-1
		//	1	1	0	0	+2  (assume pin1 edges only)
		//	1	1	0	1	-1
		//	1	1	1	0	+1
		//	1	1	1	1	no movement
/*
	// Simple, easy-to-read "documentation" version :-)
	//
	void update(void) {
		uint8_t s = state & 3;
		if (digitalRead(pin1)) s |= 4;
		if (digitalRead(pin2)) s |= 8;
		switch (s) {
			case 0: case 5: case 10: case 15:
				break;
			case 1: case 7: case 8: case 14:
				position++; break;
			case 2: case 4: case 11: case 13:
				position--; break;
			case 3: case 12:
				position += 2; break;
			default:
				position -= 2; break;
		}
		state = (s >> 2);
	}
*/


#endif