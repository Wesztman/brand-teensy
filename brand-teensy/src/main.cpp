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
//#include <Encoder.h>
#include "brandsensor.h"
#include <PID_OP.h>

#include "ros.h"
#include "ros/time.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

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
int rightLine = A13; // Right Line sensor
int startPin = 4;  // Start Button
int stopPin = 5;  // Stop Button

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

//### LSM303 L3G IMU ###

LSM303 compass; //The sensor is a LSM303DLHC
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

//### PID Controller ###

float Kp = 0.04;
float Ki = 0.09;
float Kd = 0.001;
float uMax = 100.0;
float uMin = -100.0;
//Tau is chosen as Kd/Kp/N, with N in the range of 2 to 20.
float tau = 0.01; //old 0.02
float samplingTime = 0.01;

unsigned long PIDSampletime_ms = 10; // samlingTime * 1000
unsigned long lastSample = 0;

PID_OP turnPID(Kp, Ki, Kd, tau, uMax, uMin, samplingTime);

//Tried to use PID controll for the motors but the encdoer signal oscillating too
//much and filter made it too slow.

//################# ROS ##################

float x = 0.0;  //Linear velocity
float z = 0.0;  //angular velocity
int zSign = 0;
int zSignOld = 0;
float kpt = 0.04;
float kit = 0.09;
float kdt = 0.001;
char debugString[20];
char tempString[10];

float posX = 0.0;
float posY = 0.0;
float th = 0.0;
float vx = 0.0;
float vy = 0.0;
float vth = 0.0;

float dt = 0.0;
float dPosX = 0.0;
float dPosY = 0.0;
float dTh = 0.0;

unsigned long lastRosTime = 0;
unsigned long rosDelay = 10;
char base_link_id[] = "/base_link";
char odom_id[] = "/odom";

//ros::Time current_time = ros::Time::now();
//ros::Time last_time = ros::Time::now();
unsigned long sampleTime = 0;
unsigned long sampleTimeOld = 0;

ros::NodeHandle nh;

void velCallback( const geometry_msgs::Twist& vel){
  x = vel.linear.x;
  z = vel.angular.z;
}

void pidVariables(const geometry_msgs::Point& kpid){
  kpt = kpid.x;
  kit = kpid.y;
  kdt = kpid.z;
  //Serial.println("ERROR");
}

sensor_msgs::Range VL53L0X_1;
std_msgs::Float32 angleRateDeg;
std_msgs::Float32 angleRateRad;
std_msgs::String debugData;
nav_msgs::Odometry odom;
tf::TransformBroadcaster odom_broadcaster;
geometry_msgs::Quaternion odom_quat;
geometry_msgs::TransformStamped odom_trans;

//ros::Publisher pub1("/angleRateDeg", &angleRateDeg);
//ros::Publisher pub2("/angleRateRad", &angleRateRad);
//ros::Publisher pub3("/debugData", &debugData);
ros::Publisher odom_pub("/odom", &odom);

ros::Subscriber<geometry_msgs::Twist> drive("cmd_vel", velCallback);
ros::Subscriber<geometry_msgs::Point> PIDpara("PIDpara", pidVariables);



//### Buttons### 

Button_OP startButton(startPin);
Button_OP stopButton(stopPin);
bool startFlag = LOW;
bool stopFlag = LOW;

//=================================================================
//===                       VARIABLES                          ====
//=================================================================


int ultraDist = 100; //  stores the value of ultrasonic sensor

uint16_t distances_mm[9]; //Stores all ToF distance sensor data

unsigned long serialDelay = 100; // Delay in ms between each Serial print.
unsigned long lastSerial = 0; // Store time of last Serial print

//----------------ENCODER------------------------------

float Afilt = 0.02; //filter weight for new values
float Bfilt = 0.98; //filter weight for new values
long encMax = 1000000; // 1 sec in us
unsigned long encDiffTime = 500; //0.5 sec in ms
unsigned long lastEncSample = 0; 
bool firstStartFlag = HIGH;


volatile uint8_t stateR;
volatile uint8_t newStateR;
volatile long rightEncPosRaw = 0;
long rightEncPos = 0;
long rightEncTimeRaw = 0;
long rightEncStartTime = 0;
long rightEncTime = 0;
long rightEncTimeOld = 0;
bool clockFlagR = true; 
float rightEncTimeFilt = 0.0;
float rightEncTimeSec = 0.0;

volatile uint8_t stateL;
volatile uint8_t newStateL;
volatile long leftEncPosRaw = 0;
long leftEncPos = 0;
long leftEncTimeRaw = 0;
long leftEncStartTime = 0;
long leftEncTime = 0;
long leftEncTimeOld = 0;
bool clockFlagL = true;
float leftEncTimeFilt = 0.0;
float leftEncTimeSec = 0.0;

//calculated meter per pulse
//Wheel Di = 66 mm, circumference = 207,35 mm
//Encoder has 12 pulses per revulotion = 0.01728 m/p
float meterPerPulse = 0.01728;

float rightVelocity = 0.0; 
float rightVelocityFilt = 0.0; 
float leftVelocity = 0.0; 
float leftVelocityFilt = 0.0; 
float leftMotorSetpoint = 0.0;
float rightMotorSetpoint = 0.0;
float turnControlSignal = 0.0;
float dutyLeft = 0.0;
float dutyRight = 0.0;

float Aturn = 0.1;
float Bturn = 0.9;
float filtAngularRateRad = 0.0;

float testVel = 17.28;
long test2Vel = 17280;
float testF = 0.0;
long leftTEST = 0;
long left2Test = 0;

//----------------LINE SENSOR-------------------
int leftLineValue = 0;
int rightLineValue = 0;


//---------LSM303 L3G IMU--------------
float angleZdeg;
float angleZrad;
double angleZ;
float angularRate;  //unit mdps
float angularRateRad; // unit mrps
long deltaT = 0;
long tLast = 0;
float gyroGain = 8.75;    // The gain depends on the angular rate sensitivity. Default 8.75
float estGyroDrift = 0; //3.7;  //Is dependent on temperature and how often the angle is calculated
float angleZRel;
float angleZRelDeg;



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
void encRightISR();
void encLeftISR();


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
  compass.m_min = (LSM303::vector<int16_t>){278, -216, -4};
  compass.m_max = (LSM303::vector<int16_t>){579, +88, +4};
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

  //---------------------Encoder ------------------------------
  
  pinMode(ENC_A1, INPUT_PULLUP);
  pinMode(ENC_B1, INPUT_PULLUP);
  pinMode(ENC_A2, INPUT_PULLUP);
  pinMode(ENC_B2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A1), encRightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B1), encRightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), encLeftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B2), encLeftISR, CHANGE);
  

  //---------------------- ROS -------------------------------
  
  nh.initNode();
  odom_broadcaster.init(nh);

  nh.subscribe(drive);
  //nh.advertise(pub1);
  //nh.advertise(pub2);
  //nh.advertise(pub3);
  nh.subscribe(PIDpara);
  nh.advertise(odom_pub);
  
  //----------------------------------------------------------

  //---------------------- PID -------------------------------

  //Initiate PID
  turnPID.PIDInit();
    
}


// ================================================================
// ===                    ---- MAIN ----                        ===
// ================================================================

void loop()
{
  //nh.spinOnce();
  
  //################### Distance sensors ####################
  //ultraDist = readUltraDist(trigPin, echoPin);
  leftLineValue = readLineSensor(leftLine);
  rightLineValue = readLineSensor(rightLine);  
  //timed_async_read_sensors();
  

  //#################### IMU #####################################
  compass.read();
  gyro.read();
  float heading = compass.heading();
  calcAngle();
  filtAngularRateRad = Aturn * angularRateRad + Bturn * filtAngularRateRad;

  angleRateDeg.data = angularRate; // -1 because left turn i positive 
  angleRateRad.data = filtAngularRateRad;  // -1 because left turn i positive


  //################ ENCODER and Velocity #####################
  //Assign encoder values without interrupts to make sure 
  //the raw value wont be affected by interrupts  
  noInterrupts();
  rightEncPos = rightEncPosRaw;
  leftEncPos = leftEncPosRaw;
  rightEncTime = rightEncTimeRaw;
  leftEncTime = leftEncTimeRaw;
  interrupts();

  //Cap the encoder time value
  if (leftEncTime > encMax)
  {
    leftEncTime = encMax;
  }
  if (rightEncTime > encMax)
  {
    rightEncTime = encMax;
  }

  leftEncTimeSec = leftEncTime / 1000000.0;
  rightEncTimeSec = rightEncTime / 1000000.0;

  //Make sure that leftVelocity doesn't get inf
  if (leftEncTimeSec <= 0.00001)
  {
    leftVelocity = 0.0;
  } else {
    leftVelocity = meterPerPulse / leftEncTimeSec;
  }
//Make sure that rightVelocity doesn't get inf
  if (rightEncTimeSec <= 0.00001)
  {
    rightVelocity = 0.0;
  } else {
    rightVelocity = meterPerPulse / rightEncTimeSec;
  }

  leftVelocityFilt = Afilt * leftVelocity + Bfilt * leftVelocityFilt;
  rightVelocityFilt = Afilt * rightVelocity + Bfilt * rightVelocityFilt;

  //Enc reset - If encoder value is the same after period,
  //set all encoder values to zero
  if (millis() - lastEncSample > encDiffTime)
  {
    if (leftEncTimeOld == leftEncTime){
      leftEncTime = 0;
      leftEncTimeOld = 0;
      leftEncTimeFilt = 0;
      leftVelocityFilt = 0;
      noInterrupts();
      leftEncTimeRaw = 0;
      interrupts();
    }
    if (rightEncTimeOld == rightEncTime){
      rightEncTime = 0;
      rightEncTimeOld = 0;
      rightEncTimeFilt = 0;
      rightVelocityFilt = 0;
      noInterrupts();
      rightEncTimeRaw = 0;
      interrupts();
    }
    leftEncTimeOld = leftEncTime;
    rightEncTimeOld = rightEncTime;
    lastEncSample = millis();
  }  
   
   
  //############### Buttons ############################
  if (startButton.isPressed())
  {
    startFlag = HIGH;
    stopFlag = LOW;
  }
  
  if (stopButton.isPressed())
  {
    stopFlag = HIGH;
    startFlag = LOW;
  }
  //################# ROS ##########################
  //compute odometry in a typical way given the velocities of the robot
  
  sampleTime = micros() - sampleTimeOld;
  sampleTimeOld = micros();
  
  vx = (leftVelocityFilt + rightVelocityFilt)/2;
  
  dt = (float)sampleTime / 1000000.0;
  dPosX = (vx * cos(th)) * dt;
  dPosY = (vx * sin(th)) * dt;
  
  posX += dPosX;
  posY += dPosY;
   
  if( millis() - lastRosTime > rosDelay){

    //send the transform
    odom_trans.header.frame_id = odom_id;
    odom_trans.child_frame_id = base_link_id;

    odom_trans.transform.translation.x = posX; //posX
    odom_trans.transform.translation.y = posY;  //posY
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionFromYaw(angleZrad); //tf;
    odom_trans.header.stamp = nh.now();
    
    odom_broadcaster.sendTransform(odom_trans);
   
    odom.header.stamp = nh.now();
    odom.header.frame_id = odom_id;
    //set the position
    odom.pose.pose.position.x = posX;
    odom.pose.pose.position.y = posY;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionFromYaw(angleZrad);
    //set the velocity
    odom.child_frame_id = base_link_id;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = filtAngularRateRad;

    //pub1.publish(&angleRateDeg);  
    //pub2.publish(&angleRateRad);
    //pub3.publish(&debugData);
    odom_pub.publish(&odom);

    nh.spinOnce();
    
    lastRosTime = millis();
  }
  
  
  //################# Communication ######################
  if (millis() - lastSerial > serialDelay)
  {

    //sprintf(debugString,"Turn: %f", turnControlSignal);
    
    strcpy(debugString, "Turn: ");
    dtostrf(turnControlSignal, 4, 1, tempString);
    strcat(debugString, tempString);
    strcat(debugString, " VelR: ");
    dtostrf(rightVelocityFilt, 4, 3, tempString);
    strcat(debugString, tempString);
    strcat(debugString, " VelL: ");
    dtostrf(leftVelocityFilt, 4, 3, tempString);
    strcat(debugString, tempString);
    //strcat(debugString, " D: ");
    //dtostrf(turnPID.D, 4, 1, tempString);
    //strcat(debugString, tempString);
    
    debugData.data = debugString;

  //   Serial.print(ultraDist);
    
    /*
    Serial.print("R: ");
    Serial.print(dutyRight);
    Serial.print(" L: ");
    Serial.print(dutyLeft);
    Serial.print("VelR: ");
    Serial.print(rightEncTime);
    Serial.print(",");
    Serial.println(leftEncTime);
    */
    //Serial.print(testsample); 
    //Serial.print(",");
    //Serial.println(sampleTimeOld);
    //Serial.print(" Vinkelhastighet: ");
     
    //Serial.println(z);

    


  /*
   Serial.print("1: ");
   Serial.print(leftEncTime);
   Serial.print(" 2: ");
   Serial.print(leftEncTimeFilt, 5);
   Serial.print(" 3: ");
   Serial.print(leftVelocity, 5);
   Serial.print(" 4: ");
   Serial.print(leftEncPos, 5);
   Serial.print(" 5: ");
   Serial.print(LeftMotorPID.Ki);
   Serial.print(" 6: ");
   Serial.print(LeftMotorPID.error);
   Serial.print(" 7: ");
   Serial.print(LeftMotorPID.P);
   Serial.print(" 8: ");
   Serial.print(LeftMotorPID.I);
   Serial.print(" 9: ");
   Serial.print(leftEncTimeOld);
   Serial.print(" 10: ");
   Serial.print(leftVelocityFilt, 5);
   Serial.print(" 11: ");
   Serial.print(LeftMotorPID.measurement);
   Serial.print(" 12: ");
   Serial.print(LeftMotorPID.measurementOld);
   Serial.print(" 13: ");
   Serial.print(LeftMotorPID.tau);
   Serial.print(" 14: ");
   Serial.print(LeftMotorPID.D);
   Serial.print(" 15: ");
   Serial.println(leftVelocityFilt, 5);
   */
  

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
    

  //#########
  if (startFlag)
  {
    if (z > 0)
    {
      zSign = 1;
    }else if (z < 0)
    {
      zSign = -1;
    }else if (z == 0)
    {
      zSign = 0;
    }

    if (zSign != zSignOld)
    {
      turnPID.resetIntegral();
    }
    
    RunMotors(x, z); //x, z 
    zSignOld = zSign;
  } else if (stopFlag) {
    motor.changeStatus(MOTOR_CH_BOTH, MOTOR_STATUS_STOP);
  }

  //delay(10);
  //nh.spinOnce();
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

//Calculates the rotation angle from the gyro. Calculates both in deg and rad to the following variables:
//anlgeZdeg
//angleZrad
//Limits the Angle to between 0-360 deg
void calcAngle()
 {
  deltaT = micros() - tLast;
  tLast = micros();
  
  angularRate = (int)gyro.g.z * gyroGain + 506.812; //506.812 is the offset. unit is mpds
  angularRateRad = angularRate * 71 / 4068;
  angleZ = angleZ + (angularRate * deltaT) / 1000000 - estGyroDrift;
  angleZRel = angleZRel + (angularRate * deltaT) / 1000000 - estGyroDrift;

  angleZdeg = angleZ / 1000.0;
  angleZRelDeg = angleZRel / 1000.0;

  //Limits the Angle to between -180 - 180 degrees
  if (angleZdeg < -180)
  {
    angleZdeg = angleZdeg + 360;
    angleZ = angleZ + 360000;
  }else if (angleZdeg > 180)
  {
    angleZdeg = angleZdeg - 360;
    angleZ = angleZ - 360000;
  }
  
  angleZrad = (angleZdeg *71) / 4068;
} 

void RunMotors(float velocity, float angular)
{
  float dutyMax = 80.0;
  float dutyMin = -80.0;
  float dutyThreshold = 15.0;
  float dutyOffset = 3.29;
  if (velocity < 0)
  {
    dutyOffset = -3.29;
  }

  turnPID.Kp = kpt;
  turnPID.Ki = kit;
  turnPID.Kd = kdt;

  if (millis() - lastSample > PIDSampletime_ms)
  {
    turnControlSignal = turnPID.PIDUpdate(angular*1000.0, filtAngularRateRad);
    
    lastSample = millis();
    
  }

  dutyLeft = 51.0 * velocity + dutyOffset - turnControlSignal;
  dutyRight = 51.0 * velocity + dutyOffset + turnControlSignal;

  //Make sure duty doesn't exceed max value
  if (dutyLeft >= dutyMax)
  {
    dutyLeft = dutyMax;
  }
  if (dutyRight >= dutyMax)
  {
    dutyRight = dutyMax;
  }
  //Make sure duty doesn't exceed min value
  if (dutyLeft < dutyMin)
  {
    dutyLeft = dutyMin;
  }
  if (dutyRight <= dutyMin)
  {
    dutyRight = dutyMin;
  }

  //If duty is too low set to zero
  if (dutyLeft < dutyThreshold && dutyLeft > -dutyThreshold)
  {
    dutyLeft = 0.0;
  }
  if (dutyRight < dutyThreshold && dutyRight > -dutyThreshold)
  {
    dutyRight = 0;
  }

  if (dutyLeft >= 0)
  {
    motor.changeStatus(MOTOR_CH_A, MOTOR_STATUS_CCW);
    motor.changeDuty(MOTOR_CH_A, dutyLeft);
  }
  if (dutyLeft < 0)
  {
    motor.changeStatus(MOTOR_CH_A, MOTOR_STATUS_CW);
    motor.changeDuty(MOTOR_CH_A, abs(dutyLeft));
  }
  
  if (dutyRight >= 0)
  {
    motor.changeStatus(MOTOR_CH_B, MOTOR_STATUS_CCW);
    motor.changeDuty(MOTOR_CH_B, dutyRight);
  }
  if (dutyRight < 0)
  {
    motor.changeStatus(MOTOR_CH_B, MOTOR_STATUS_CW);
    motor.changeDuty(MOTOR_CH_B, abs(dutyRight));
  }

}

//See brandsensor.h for description
void encRightISR()
{
  newStateR = stateR & 3;
  if (digitalRead(ENC_A1)) newStateR |= 4;
	if (digitalRead(ENC_B1)) newStateR |= 8;
		switch (newStateR) {
			case 0: case 5: case 10: case 15:
				break;
			case 1: case 7: case 8: case 14:
				rightEncPosRaw++;
        if(clockFlagR){
          rightEncStartTime = micros();
          clockFlagR = false;
        }else {
          rightEncTimeRaw = micros() - rightEncStartTime;
          clockFlagR = true;
        }
        break;
			case 2: case 4: case 11: case 13:
				rightEncPosRaw--;
        if(clockFlagR){
          rightEncStartTime = micros();
          clockFlagR = false;
        }else {
          rightEncTimeRaw = micros() - rightEncStartTime;
          clockFlagR = true;
        }
         break;
			case 3: case 12:
				rightEncPosRaw += 2;
        if(clockFlagR){
          rightEncStartTime = micros();
          clockFlagR = false;
        }else {
          rightEncTimeRaw = (micros() - rightEncStartTime) / 2;
          clockFlagR = true;
        }
         break;
			default:
				rightEncPosRaw -= 2;
        if(clockFlagR){
          rightEncStartTime = micros();
          clockFlagR = false;
        }else {
          rightEncTimeRaw = (micros() - rightEncStartTime) / 2;
          clockFlagR = true;
        }
         break;
	  }
		stateR = (newStateR >> 2);
}

//See brandsensor.h for description
void encLeftISR()
{
  newStateL = stateL & 3;
  if (digitalRead(ENC_A2)) newStateL |= 4;
	if (digitalRead(ENC_B2)) newStateL |= 8;
		switch (newStateL) {
			case 0: case 5: case 10: case 15:
				break;
			case 1: case 7: case 8: case 14:
				leftEncPosRaw++;
        if(clockFlagL){
          leftEncStartTime = micros();
          clockFlagL = false;
        }else {
          leftEncTimeRaw = micros() - leftEncStartTime;
          clockFlagL = true;
        }
        break;
			case 2: case 4: case 11: case 13:
				leftEncPosRaw--; 
        if(clockFlagL){
          leftEncStartTime = micros();
          clockFlagL = false;
        }else {
          leftEncTimeRaw = micros() - leftEncStartTime;
          clockFlagL = true;
        }
        break;
			case 3: case 12:
				leftEncPosRaw += 2; 
        if(clockFlagL){
          leftEncStartTime = micros();
          clockFlagL = false;
        }else {
          leftEncTimeRaw = (micros() - leftEncStartTime) / 2;
          clockFlagL = true;
        }
        break;
			default:
				leftEncPosRaw -= 2; 
        if(clockFlagL){
          leftEncStartTime = micros();
          clockFlagL = false;
        }else {
          leftEncTimeRaw = (micros() - leftEncStartTime) / 2;
          clockFlagL = true;
        }
        break;
		}
		stateL = (newStateL >> 2);
}

