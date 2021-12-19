/*
 * PID_OP.h - Library for creating a PID Controller
 * Created by Oskar Persson, June 7, 2020
 */
 #ifndef PID_OP_h
 #define PID_OP_h

 #include "Arduino.h"

 class PID_OP
 {
  public:
    PID_OP(float _kp, float _ki, float _kd, float _tau, float _uMax, float _uMin, float _samplingTime);

    void PIDInit();
    float PIDUpdate(float _setpoint, float _measurement);

  private:
    float Kp; //Proportional gain
    float Ki; //Integral gain
    float Kd; //Derivative gain
    float P;  //Proportinal part
    float I;  //Inttegral part
    float D;  //Derivative part
    float tau;  //Derivative filter
    float setpoint;  //Setpoint
    float measurement;  //Measurement
    float uMax; //High output limit
    float uMin; //Low output limit
    float IMin;  //Low integral limit
    float IMax;  //High integral limit
    float error;    //Error
    float errorOld; //Previous error
    float measurementOld; //Previous measurement
    float samplingTime;  //Sampling time i seconds
    float u; //Output
    
 };


#endif
