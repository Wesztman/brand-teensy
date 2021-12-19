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
#include <LSM303.h>
#include <L3G.h>
#include <Adafruit_AMG88xx.h>
#include <Encoder.h>
#include "brandsensor.h"
#include <PID_OP.h>

#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/String.h"


//=================================================================
//===                       PIN DEFINITION                     ====
//=================================================================

int echoPin = 34; // Echo of HC-SR04 
int trigPin = 33; // Trig of HC-SR04 
int xShut1 = 14;  // xShut Distance sensor 1
int xShut2 = 39;  // xShut Distance sensor 2
int xShut3 = 36;  // xShut Distance sensor 3
int xShut4 = 35;  // xShut Distance sensor 4
int xShut5 = 30;  // xShut Distance sensor 5
int xShut6 = 29;  // xShut Distance sensor 6
int xShut7 = 28;  // xShut Distance sensor 7
int xShut8 = 27;  // xShut Distance sensor 8
int xShut9 = 26;  // xShut Distance sensor 9
int ENC_A1 = 6;   // Right encoder A-channel
int ENC_B1 = 7;   // Right encoder B-channel
int ENC_A2 = 8;   // Left encoder A-channel
int ENC_B2 = 9;   // Left encoder B-channel
int leftLine = A12; // Left Line sensor
int Rightline = A13; // Right Line sensor

//=================================================================
//===                       SYSTEM DEFINTION                   ====
//=================================================================

//### Motor Driver ###
LOLIN_I2C_MOTOR motor; //I2C address 0x30

//### Encoder ###
Encoder RightEnc(ENC_A1, ENC_B1);
Encoder LeftEnc(ENC_A2, ENC_B2);

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

//### LSM303 L3G IMU ###

LSM303 compass;
L3G gyro;
/*
  When given no arguments, the heading() function returns the angular
  difference in the horizontal plane between a default vector and
  north, in degrees.
  
  The default vector is chosen by the library to point along the
  surface of the PCB, in the direction of the top of the text on the
  silkscreen. This is the +X axis on the Pololu LSM303D carrier and
  the -Y axis on the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH
  carriers.
  
  To use a different vector as a reference, use the version of heading()
  that takes a vector argument; for example, use
  
    compass.heading((LSM303::vector<int>){0, 0, 1});
  
  to use the +Z axis as a reference.
  */

//### AMG8833 IR Camera ###

Adafruit_AMG88xx amg;

float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

//### ROS ###

float x;  //Linear velocity
float z;  //angular velocity

ros::NodeHandle nh;

void velCallback( const geometry_msgs::Twist& vel){
  x = vel.linear.x;
  z = vel.angular.z;
}

//std_msgs::String test_msg;
//ros::Publisher test_topic("test", &test_msg);
sensor_msgs::Range VL53L0X_1;


ros::Subscriber<geometry_msgs::Twist> drive("cmd_vel", velCallback);

//=================================================================
//===                       VARIABLES                          ====
//=================================================================


int ultraDist = 100; //  stores the value of ultrasonic sensor

uint16_t distances_mm[9]; //Stores all ToF distance sensor data

unsigned long serialDelay = 100; // Delay in ms between each Serial print.
unsigned long lastSerial = 0; // Store time of last Serial print

long rightEncPos = 0;
long oldRightEncPos = 0; //old -999
long rightEncTime = 0;
long rightEncStartTime = 0;
float rightEncPulsesPerSec = 0;
long leftEncPos = 0;
long oldLeftEncPos = 0; //old -999
long leftEncTime = 0;
long leftEncStartTime = 0;
float leftEncPulsesPerSec = 0;


int leftLineValue = 0;
int rightLineValue = 0;


//---------LSM303 L3G IMU--------------
float angleZdeg;
float angleZrad;
double angleZ;
float angularRate;
long deltaT = 0;
long tLast = 0;
float gyroGain = 8.75;    // The gain depends on the angular rate sensitivity. Default 8.75
float estGyroDrift = -18;  //Is dependent on temperature and how often the angle is calculated
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
void calcAngle();
void irCameraTest();
void RunMotors(float velocity, float angular);


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  //Serial.begin(115200);
  Serial1.begin(57600);
  Wire.begin();
  pinMode(13, OUTPUT);

 
  //-----Setup LSM303 L3G IMU--------------------
  compass.init();
  compass.enableDefault();
  
  if (!gyro.init())
  {
   // Serial.println("Failed to autodetect gyro type!");
    while (1);
  }
  gyro.enableDefault();
  
  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  compass.m_min = (LSM303::vector<int16_t>){-378, -151, +451};
  compass.m_max = (LSM303::vector<int16_t>){-26, +193, +477};
  //---------------------------------
  
  //------- Setup Motor Driver ------------
  while (motor.PRODUCT_ID != PRODUCT_ID_I2C_MOTOR) //wait motor shield ready.
  {
    motor.getInfo();
  } 
  motor.changeFreq(MOTOR_CH_BOTH, 1000); //Change A & B 's Frequency to 1000Hz.
  //-------------------------------------
  
  //--------- Setup Ultrasonic Sensor ------------
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  //----------------------------------------------
  
  //-------------- Setup TOF Distance Sensors --------
  // initialize all of the pins.
  //VL53LOX_multi start, initialize IO pins
  for (int i = 0; i < COUNT_SENSORS; i++) {
  pinMode(sensors[i].shutdown_pin, OUTPUT);
  digitalWrite(sensors[i].shutdown_pin, LOW);

  if (sensors[i].interrupt_pin >= 0)
    pinMode(sensors[i].interrupt_pin, INPUT_PULLUP);
  }
  Initialize_sensors();
  //--------------------------------------------------
  
  
  //------------Setup AMG8833 IR Camera ----------------
  bool status;
  
  // Soldered the backside of the sensor to get address 0x68 since the gyro had address 0x69
  status = amg.begin(0x68);
  if (!status) {
    //Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
    while (1);
  }
  
  delay(100); // let sensor boot up
  //----------------------------------------------------------
  

  //---------------------- ROS -------------------------------
  
  nh.initNode();
  //nh.advertise(test_topic);
  nh.subscribe(drive);
  
  //----------------------------------------------------------
  
}


// ================================================================
// ===                    ---- MAIN ----                        ===
// ================================================================

void loop()
{
  nh.spinOnce();
  //motorTest(motor);
  //blinkTest();
  //amg.readPixels(pixels);
  
  ultraDist = readUltraDist(trigPin, echoPin);  

  //timed_async_read_sensors();
  //simpleFollow(motor, distances_mm[3], distances_mm[5]);

  compass.read();
  gyro.read();
  float heading = compass.heading();
  calcAngle();

  rightEncPos = RightEnc.read();
  leftEncPos = LeftEnc.read();
  if (rightEncPos != oldRightEncPos){
    rightEncTime = micros() - rightEncStartTime;
    rightEncStartTime = micros();
    rightEncPulsesPerSec = 1000000.0 / rightEncTime;
  }
  if (leftEncPos != oldLeftEncPos){
    leftEncTime = micros() - leftEncStartTime;
    leftEncStartTime = micros();
    leftEncPulsesPerSec = 1000000.0 / leftEncTime;
  }
  if (rightEncPos != oldRightEncPos || leftEncPos != oldLeftEncPos){
    oldRightEncPos = rightEncPos;
    oldLeftEncPos = leftEncPos;
  }

  leftLineValue = readLineSensor(leftLine);
  rightLineValue = readLineSensor(Rightline);
  
  
  if (millis() - lastSerial > serialDelay)
   {
  //   Serial.print(ultraDist);
  //   Serial.print("Heading: ");
  //   Serial.print(heading);
  //   Serial.print(" Gyro RAW Z: ");
  //   Serial.print((int)gyro.g.z);
  //   Serial.print(" Vinkel: ");
  //   Serial.println(angleZdeg);
    //printIRCamera(pixels);
    //Serial.print(leftLineValue);
   // Serial.print(" ");
   // Serial.println(rightLineValue);
   Serial.print("Right: ");
   Serial.print(rightEncPos);
   Serial.print(" Left: ");
   Serial.print(leftEncPos);
   Serial.print(" Right p/s: ");
   Serial.print(rightEncPulsesPerSec);
   Serial.print(" Left p/s: ");
   Serial.println(leftEncPulsesPerSec);
    

    lastSerial = millis();
  }
  

  //test_msg.data ="hej hej";
  //test_topic.publish(&test_msg);

  if (x > 0)
  {
    digitalWrite(13, HIGH);
  }else if (x < 0)
  {
    digitalWrite(13, LOW);
  }

  //###TEST###
  x = 0.30;
  
  //#########

  RunMotors(x, z);

  //delay(10);

  /*
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }
  */


  
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

/*
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
  */
}

//===============================================================
// Continuous range test code
//===============================================================

void start_continuous_range(uint16_t cycle_time) {
  if (cycle_time == 0)
    cycle_time = 100;
  //Serial.print(F("start Continuous range mode cycle time: "));
  //Serial.println(cycle_time, DEC);
  for (uint8_t i = 0; i < COUNT_SENSORS; i++) {
    sensors[i].psensor->startRangeContinuous(cycle_time); // do 100ms cycle
  }
  sensors_pending = ALL_SENSORS_PENDING;
  sensor_last_cycle_time = millis();
}

void stop_continuous_range() {
  //Serial.println(F("Stop Continuous range mode"));
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
  /*
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
  */
}

//====================================================================
// ASync read sensors.
//====================================================================
void timed_async_read_sensors() {
  // First use simple function
  uint16_t ranges_mm[COUNT_SENSORS];
  bool timeouts[COUNT_SENSORS];
  uint32_t stop_times[COUNT_SENSORS];

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
 
 /*
  uint32_t delta_time = millis() - start_time;
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
  */
}


 void calcAngle()
 {
  deltaT = micros() - tLast;
  tLast = micros();
  //Negative to make the rotation follow the unit circle
  angularRate = -1 * (int)gyro.g.z * gyroGain;
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
} 

void RunMotors(float velocity, float angular){
  float duty;
  
  if (velocity >= 0)
  {
    motor.changeStatus(MOTOR_CH_BOTH, MOTOR_STATUS_CCW);
    duty = map(velocity, 0.0, 2.0, 0.0, 100.0);
    motor.changeDuty(MOTOR_CH_BOTH, duty);
  }
  if (velocity < 0)
  {
    motor.changeStatus(MOTOR_CH_BOTH, MOTOR_STATUS_CW);

    duty = map(abs(velocity), 0.0, 2.0, 0.0, 100.0);
    motor.changeDuty(MOTOR_CH_BOTH, duty);
  }

}