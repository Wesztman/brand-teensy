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


#endif