#ifndef TESTS_H         // To make sure you don't declare the function more than once by including the header multiple times.
#define TESTS_H

#include <Arduino.h>
#include <Wire.h>
#include <LOLIN_I2C_MOTOR.h>

//Simple test for the motor driver.
//Actives both motor channels in both directions at different speed.
void motorTest(LOLIN_I2C_MOTOR motor);

void simpleFollow(LOLIN_I2C_MOTOR motor, int distLeft, int distRight);

//Blinks the onboard LED on Teensy
void blinkTest();

#endif