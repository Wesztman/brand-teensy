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
#include <Adafruit_VL53L0X.h>


//=================================================================
//===                       PIN DEFINITION                     ====
//=================================================================

int echoPin1 = 34; // Echo of HC-SR04 Left
int trigPin1 = 33; // Trig of HC-SR04 Left
int echoPin2 = 32; // Echo of HC-SR04 Right
int trigPin2 = 31; // Trig of HC-SR04 Right
int xShut1 = 14;  // xShut Distance sensor 1
int xShut2 = 39;  // xShut Distance sensor 2
int xShut3 = 36;  // xShut Distance sensor 3
int xShut4 = 35;  // xShut Distance sensor 4
int xShut5 = 30;  // xShut Distance sensor 5
int xShut6 = 29;  // xShut Distance sensor 6
int xShut7 = 28;  // xShut Distance sensor 7
int xShut8 = 27;  // xShut Distance sensor 8
int xShut9 = 26;  // xShut Distance sensor 9

//=================================================================
//===                       SYSTEM DEFINTION                   ====
//=================================================================

//### Motor Driver ###
LOLIN_I2C_MOTOR motor; //I2C address 0x30

//### Distance Sensors ###
#define SENSOR1_WIRE Wire
#define SENSOR2_WIRE Wire
#define SENSOR3_WIRE Wire
#define SENSOR4_WIRE Wire
#define SENSOR5_WIRE Wire
#define SENSOR6_WIRE Wire
#define SENSOR7_WIRE Wire
#define SENSOR8_WIRE Wire
#define SENSOR9_WIRE Wire

typedef enum {
  RUN_MODE_DEFAULT = 1,
  RUN_MODE_ASYNC,
  RUN_MODE_GPIO,
  RUN_MODE_CONT
} runmode_t;

runmode_t run_mode = RUN_MODE_DEFAULT;
uint8_t show_command_list = 1;

typedef struct {
  Adafruit_VL53L0X *psensor; // pointer to object
  TwoWire *pwire;
  int id;            // id for the sensor
  int shutdown_pin;  // which pin for shutdown;
  int interrupt_pin; // which pin to use for interrupts.
  Adafruit_VL53L0X::VL53L0X_Sense_config_t
      sensor_config;     // options for how to use the sensor
  uint16_t range;        // range value used in continuous mode stuff.
  uint8_t sensor_status; // status from last ranging in continous.
} sensorList_t;

// Actual object, could probalby include in structure above61
Adafruit_VL53L0X sensor1;
Adafruit_VL53L0X sensor2;
Adafruit_VL53L0X sensor3;
Adafruit_VL53L0X sensor4;
Adafruit_VL53L0X sensor5;
Adafruit_VL53L0X sensor6;
Adafruit_VL53L0X sensor7;
Adafruit_VL53L0X sensor8;
Adafruit_VL53L0X sensor9;

// Setup for 9 sensors
// Set pin 23 as interupt but we wont use interupt
sensorList_t sensors[] = {
  {&sensor1, &SENSOR1_WIRE, 0x31, xShut1, 23,
   Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
  {&sensor2, &SENSOR2_WIRE, 0x32, xShut2, 23,
   Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
  {&sensor3, &SENSOR3_WIRE, 0x33, xShut3, 23,
   Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
  {&sensor4, &SENSOR4_WIRE, 0x34, xShut4, 23,
   Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
  {&sensor5, &SENSOR5_WIRE, 0x35, xShut5, 23,
   Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
  {&sensor6, &SENSOR6_WIRE, 0x36, xShut6, 23,
   Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
  {&sensor7, &SENSOR7_WIRE, 0x37, xShut7, 23,
   Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
  {&sensor8, &SENSOR8_WIRE, 0x38, xShut8, 23,
   Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
  {&sensor9, &SENSOR9_WIRE, 0x39, xShut9, 23,
   Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0}

};

const int COUNT_SENSORS = sizeof(sensors) / sizeof(sensors[0]);

const uint16_t ALL_SENSORS_PENDING = ((1 << COUNT_SENSORS) - 1);
uint16_t sensors_pending = ALL_SENSORS_PENDING;
uint32_t sensor_last_cycle_time;

//### GY-521 IMU ###

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data
char tmp_str[7]; // temporary variable used in convert function
char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}



//=================================================================
//===                       VARIABLES                          ====
//=================================================================

float duration; // variable for the duration of sound wave travel
float distance; // variable for the distance measurement
int ultraDist1 = 100; //  stores the value of ultrasonic sensor 1
int prevDist1 = 100;  //  stores the old value of ultrasonic sensor 1
int ultraDist2 = 100; //  stores the value of ultrasonic sensor 2
int prevDist2 = 100;  //  stores the old value of ultrasonic sensor 2

uint16_t distances_mm[9]; //Stores all ToF distance sensor data

//---------GY-521 IMU--------------
//Acceleration data correction
int AcXoff = 0; //set when board is in place
int AcYoff = 0;//set when board is in place
int AcZoff = 0;//set when board is in place

//Temperature correction
int toff = 0;

//Gyro correction
int GyXoff = 480;
int GyYoff = 0;
int GyZoff = 150;

float angleZdeg;
float angleZrad;
double angleZ;
float angularRate;
long deltaT = 0;
long tLast = 0;
float gyroGain = 70;    //Value from the LSM6DS33 Datasheet page 15. The gain depends on the angular rate sensitivity. Default 8.75
float angleZRel;
float angleZRelDeg;
//-----------------------------------------------------------


// ================================================================
// ===                 ---- PROTOTYPES ----                     ===
// ================================================================
int readUltraDist(int trig, int echo);
void Initialize_sensors();
void read_sensors();
void start_continuous_range(uint16_t cycle_time);
void Process_continuous_range();
void timed_async_read_sensors();
void readIMU();


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  //-----Setup GY-521------
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  //-----------------------
  
/*   while (motor.PRODUCT_ID != PRODUCT_ID_I2C_MOTOR) //wait motor shield ready.
  {
    motor.getInfo();
  } */
  motor.changeFreq(MOTOR_CH_BOTH, 1000); //Change A & B 's Frequency to 1000Hz.
 
  pinMode(trigPin1, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin1, INPUT); // Sets the echoPin as an INPUT
  pinMode(trigPin2, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin2, INPUT); // Sets the echoPin as an INPUT
  
  // wait until serial port opens ... For 5 seconds max
  while (!Serial && millis() < 5000)
    ;

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // initialize all of the pins.
    //VL53LOX_multi start, initialize IO pins
    for (int i = 0; i < COUNT_SENSORS; i++) {
    pinMode(sensors[i].shutdown_pin, OUTPUT);
    digitalWrite(sensors[i].shutdown_pin, LOW);

    if (sensors[i].interrupt_pin >= 0)
      pinMode(sensors[i].interrupt_pin, INPUT_PULLUP);
  }
  Initialize_sensors();

}


// ================================================================
// ===                    ---- MAIN ----                        ===
// ================================================================

void loop()
{

  //motorTest(motor);

  /* ultraDist1 = readUltraDist(trigPin1, echoPin1);
  ultraDist2 = readUltraDist(trigPin2, echoPin2);  
  Serial.print(ultraDist1);
  Serial.print(" ");
  Serial.println(ultraDist2);

  if(ultraDist1 == 0){
    ultraDist1 = prevDist1;
  }
  if(ultraDist2 == 0){
    ultraDist2 = prevDist2;
  }
  prevDist1 = ultraDist1;
  prevDist2 = ultraDist2;
  
  simpleFollow(motor, ultraDist1, ultraDist2);
  delay(10);
 */

  //read_sensors();
 //timed_async_read_sensors();

 readIMU();
 // print out data
  Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
  Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53);
  Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x));
  Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y));
  Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z));
  Serial.println();
  
  // delay
  delay(200);

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
 return (int)distance;
}

/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then
   set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but
   0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its
   XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but
   0x29 and whatever you set the first sensor to
*/
void Initialize_sensors() {
  bool found_any_sensors = false;
  // Set all shutdown pins low to shutdown sensors
  for (int i = 0; i < COUNT_SENSORS; i++)
    digitalWrite(sensors[i].shutdown_pin, LOW);
  delay(10);

  for (int i = 0; i < COUNT_SENSORS; i++) {
    // one by one enable sensors and set their ID
    digitalWrite(sensors[i].shutdown_pin, HIGH);
    delay(10); // give time to wake up.
    if (sensors[i].psensor->begin(sensors[i].id, false, sensors[i].pwire,
                                  sensors[i].sensor_config)) {
      found_any_sensors = true;
    } else {
      Serial.print(i, DEC);
      Serial.print(F(": failed to start\n"));
    }
  }
  if (!found_any_sensors) {
    Serial.println("No valid sensors found");
    while (1)
      ;
  }
}

//====================================================================
// Simple Sync read sensors.
//====================================================================
void read_sensors() {
  // First use simple function
  uint16_t ranges_mm[COUNT_SENSORS];
  bool timeouts[COUNT_SENSORS];
  uint32_t stop_times[COUNT_SENSORS];

  digitalWrite(13, HIGH);
  uint32_t start_time = millis();
  for (int i = 0; i < COUNT_SENSORS; i++) {
    ranges_mm[i] = sensors[i].psensor->readRange();
    timeouts[i] = sensors[i].psensor->timeoutOccurred();
    stop_times[i] = millis();
  }
  uint32_t delta_time = millis() - start_time;
  digitalWrite(13, LOW);

  Serial.print(delta_time, DEC);
  Serial.print(F(" "));
  for (int i = 0; i < COUNT_SENSORS; i++) {
    Serial.print(i, DEC);
    Serial.print(F(":"));
    Serial.print(ranges_mm[i], DEC);
    Serial.print(F(" "));
    Serial.print(stop_times[i] - start_time, DEC);
    if (timeouts[i])
      Serial.print(F("(TIMEOUT) "));
    else
      Serial.print(F("          "));
    start_time = stop_times[i];
  }
  Serial.println();
}

//===============================================================
// Continuous range test code
//===============================================================

void start_continuous_range(uint16_t cycle_time) {
  if (cycle_time == 0)
    cycle_time = 100;
  Serial.print(F("start Continuous range mode cycle time: "));
  Serial.println(cycle_time, DEC);
  for (uint8_t i = 0; i < COUNT_SENSORS; i++) {
    sensors[i].psensor->startRangeContinuous(cycle_time); // do 100ms cycle
  }
  sensors_pending = ALL_SENSORS_PENDING;
  sensor_last_cycle_time = millis();
}

void stop_continuous_range() {
  Serial.println(F("Stop Continuous range mode"));
  for (uint8_t i = 0; i < COUNT_SENSORS; i++) {
    sensors[i].psensor->stopRangeContinuous();
  }
  delay(100); // give time for it to complete.
}

void Process_continuous_range() {

  uint16_t mask = 1;
  for (uint8_t i = 0; i < COUNT_SENSORS; i++) {
    bool range_complete = false;
    if (sensors_pending & mask) {
      if (sensors[i].interrupt_pin >= 0)
        range_complete = !digitalRead(sensors[i].interrupt_pin);
      else
        range_complete = sensors[i].psensor->isRangeComplete();
      if (range_complete) {
        sensors[i].range = sensors[i].psensor->readRangeResult();
        sensors[i].sensor_status = sensors[i].psensor->readRangeStatus();
        sensors_pending ^= mask;
      }
    }
    mask <<= 1; // setup to test next one
  }
  // See if we have all of our sensors read OK
  uint32_t delta_time = millis() - sensor_last_cycle_time;
  if (!sensors_pending || (delta_time > 1000)) {
    digitalWrite(13, !digitalRead(13));
    Serial.print(delta_time, DEC);
    Serial.print(F("("));
    Serial.print(sensors_pending, HEX);
    Serial.print(F(")"));
    mask = 1;
    for (uint8_t i = 0; i < COUNT_SENSORS; i++) {
      Serial.print(F(" : "));
      if (sensors_pending & mask)
        Serial.print(F("TTT")); // show timeout in this one
      else {
        Serial.print(sensors[i].range, DEC);
        if (sensors[i].sensor_status == VL53L0X_ERROR_NONE)
          Serial.print(F("  "));
        else {
          Serial.print(F("#"));
          Serial.print(sensors[i].sensor_status, DEC);
        }
      }
    }
    // setup for next pass
    Serial.println();
    sensor_last_cycle_time = millis();
    sensors_pending = ALL_SENSORS_PENDING;
  }
}

//====================================================================
// ASync read sensors.
//====================================================================
void timed_async_read_sensors() {
  // First use simple function
  uint16_t ranges_mm[COUNT_SENSORS];
  bool timeouts[COUNT_SENSORS];
  uint32_t stop_times[COUNT_SENSORS];

  digitalWrite(13, HIGH);
  uint32_t start_time = millis();

  // Tell all sensors to start.
  for (int i = 0; i < COUNT_SENSORS; i++) {
    ranges_mm[i] = sensors[i].psensor->startRange();
  }
  // We could call to see if done, but this version the readRange will wait
  // until ready
  for (int i = 0; i < COUNT_SENSORS; i++) {
    ranges_mm[i] = sensors[i].psensor->readRangeResult();
    timeouts[i] = sensors[i].psensor->timeoutOccurred();
    stop_times[i] = millis();
    distances_mm[i] = ranges_mm[i];
  }
  uint32_t delta_time = millis() - start_time;
  digitalWrite(13, LOW);

  Serial.print(delta_time, DEC);
  Serial.print(F(" "));
  for (int i = 0; i < COUNT_SENSORS; i++) {
    Serial.print(i, DEC);
    Serial.print(F(":"));
    Serial.print(ranges_mm[i], DEC);
    Serial.print(F(" "));
    Serial.print(stop_times[i] - start_time, DEC);
    if (timeouts[i])
      Serial.print(F("(TIMEOUT) "));
    else
      Serial.print(F("          "));
    start_time = stop_times[i];
  }
  Serial.println();
}

void readIMU()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  gyro_x = gyro_x + GyXoff; //Add correction
  gyro_y = gyro_y + GyYoff; //Add correction
  gyro_z = gyro_z + GyZoff; //Add correction
}

/* float calcAngle(){
  deltaT = micros() - tLast;
  tLast = micros();
  //Negative to make the rotation follow the unit circle
  angularRate = -1 * gyro_z * gyroGain;
  angleZ = angleZ + (angularRate * deltaT) / 1000000 - estGyroDrift;
  angleZRel = angleZRel + (angularRate * deltaT) / 1000000 - estGyroDrift;

  angleZdeg = angleZ / 1000.0;
  angleZRelDeg = angleZRel / 1000.0;

  //Limits the Angle to between 0-360
  if (angleZdeg < 0)
  {
    angleZdeg = 360 - angleZdeg;
    angleZ = 360000 - angleZ;
  }else if (angleZdeg > 360)
  {
    angleZdeg = angleZdeg - 360;
    angleZ = angleZ - 360000;
  }
  
  angleZrad = (angleZdeg *71) / 4068;
} */