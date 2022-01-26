#include "PID_OP.h"
#include "Arduino.h"

PID_OP::PID_OP(float _kp, float _ki, float _kd, float _tau, float _uMax, float _uMin, float _samplingTime)
{
  Kp = _kp;
  Ki = _ki;
  Kd = _kd;
  tau = _tau;
  uMax = _uMax;
  uMin = _uMin;
  samplingTime = _samplingTime;
}

void PID_OP::PIDInit()
{
  P = 0.0;
  I = 0.0;
  D = 0.0;
  errorOld = 0.0;
  measurementOld = 0.0;
  u = 0.0;
}

float PID_OP::PIDUpdate(float _setpoint, float _measurement)
{
  setpoint = _setpoint;
  measurement = _measurement;

  /*
    Error signal
  */
  error = setpoint - measurement;

  /*
    Proportional
  */
  P = Kp * error;

  /*
    Integral
  */
  I = I + 0.5 * Ki * samplingTime * (error + errorOld);

  /* Anti-wind-up via dynamic integrator clamping */
  /* Compute integrator limits */
  if (uMax > P) {

    IMax = uMax - P;

  } else {

    IMax = 0.0;

  }

  if (uMin < P) {

    IMin = uMin - P;

  } else {

    IMin = 0.0;

  }

  /* Clamp integrator */
  if (I > IMax) {

    I = IMax;

  } else if (I < IMin) {

    I = IMin;

  }


  /*
    Derivative (band-limited differentiator)
  */
  /* Note: derivative on measurement, therefore minus sign in front of equation! */
  D = -(2.0 * Kd * (measurement - measurementOld)
        + (2.0 * tau - samplingTime) * D)
      / (2.0 * tau + samplingTime);


  /*
    Compute output and apply limits
  */
  u = P + I + D;

  //If output is inf or nan reset PID
  if (isinf(u) || isnan(u))
  {
    P = 0.0;
    I = 0.0;
    D = 0.0;
    u = 0.0;
    //Serial.println("ERROR");
  }

  if (u > uMax) {

    u = uMax;

  } else if (u < uMin) {

    u = uMin;

  }

  /* Store error and measurement for later use */
  errorOld = error;
  measurementOld = measurement;

  /* Return controller output */
  return u;

}

void PID_OP::resetIntegral()
{
  this->I = 0.0;
}