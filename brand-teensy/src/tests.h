#ifndef TESTS_H         // To make sure you don't declare the function more than once by including the header multiple times.
#define TESTS_H

#include <Arduino.h>
#include <Wire.h>
#include <LOLIN_I2C_MOTOR.h>

void motorTest(LOLIN_I2C_MOTOR motor);

#endif