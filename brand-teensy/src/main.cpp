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



//=================================================================
//===                       SYSTEM DEFINTION                   ====
//=================================================================

//# Motor Driver #
LOLIN_I2C_MOTOR motor; //I2C address 0x30
// LOLIN_I2C_MOTOR motor(DEFAULT_I2C_MOTOR_ADDRESS); //I2C address 0x30
// LOLIN_I2C_MOTOR motor(your_address); //using customize I2C address


//=================================================================
//===                       VARIABLES                          ====
//=================================================================


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
}

// ================================================================
// ===                    ---- MAIN ----                        ===
// ================================================================

void loop()
{
  motorTest(motor);
}
