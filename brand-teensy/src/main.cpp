/*
  Team Bränd - Main Software for Teensy

  Created:
  2020-11-28

  Description:
  Main Teensy software for the firefighter robot for team Bränd.

  Contributers:
  Oskar Persson

*/

//=================================================================
//===                       LIBRARIES                          ====
//=================================================================

#include <Arduino.h>
#include <Wire.h>
#include <LOLIN_I2C_MOTOR.h>
#include "tests.h"

//=================================================================
//===                       PIN DEFINITION                     ====
//=================================================================

int echoPin1 = 2; // attach pin D2 Teensy to pin Echo of HC-SR04
int trigPin1 = 3; //attach pin D3 Teensy to pin Trig of HC-SR04
int echoPin2 = 4; // attach pin D4 Teensy to pin Echo of HC-SR04
int trigPin2 = 5; //attach pin D5 Teensy to pin Trig of HC-SR04

//=================================================================
//===                       SYSTEM DEFINTION                   ====
//=================================================================

//# Motor Driver #
LOLIN_I2C_MOTOR motor; //I2C address 0x30



//=================================================================
//===                       VARIABLES                          ====
//=================================================================

long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement
int ultraDist1; //  stores the value of ultrasonic sensor 1
int ultraDist2; //  stores the value of ultrasonic sensor 2



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  Serial.begin(115200);
  
  while (motor.PRODUCT_ID != PRODUCT_ID_I2C_MOTOR) //wait motor shield ready.
  {
    motor.getInfo();
  }
  motor.changeFreq(MOTOR_CH_BOTH, 1000); //Change A & B 's Frequency to 1000Hz.

  pinMode(trigPin1, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin1, INPUT); // Sets the echoPin as an INPUT
  pinMode(trigPin2, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin2, INPUT); // Sets the echoPin as an INPUT
  

}

// ================================================================
// ===                 ---- PROTOTYPES ----                     ===
// ================================================================
int readUltraDist(int trig, int echo);

// ================================================================
// ===                    ---- MAIN ----                        ===
// ================================================================

void loop()
{

  ultraDist1 = readUltraDist(trigPin1, echoPin1);
  //ultraDist2 = readUltraDist(trigPin2, echoPin2);
  Serial.println(ultraDist1);
  //Serial.print(" ");
  //Serial.println(ultraDist2);
  delay(10);

}

//Triggers Ultrasonic sensor and measures the travel time  of the returning sound wave.
//Returns the distance in mm.
//Input: triggerpin and echopin for the sensor
int readUltraDist(int trig, int echo){
   // Clears the trigPin condition
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echo, HIGH, 11600);
  // Calculating the distance
  distance = duration * 0.343 / 2; // Speed of sound wave divided by 2 (go and back)
  // Return distance
 return distance;
}