#include "PIDController.h"

void PIDController::setpoint(double newSetpoint)
{
  setPoint = newSetpoint;
}

void PIDController::resetLastTime()
{
  lastTime = millis();
}

double PIDController::getP()
{
  return Kp * lastErr;
}

double PIDController::getI()
{
  return Ki * errSum;
}

double PIDController::getD()
{
  return Kd * dErr;
}

double PIDController::getTimeChange()
{
  return timeChange;
}

void PIDController::tune(double _Kp, double _Ki, double _Kd)
{
  if (_Kp < 0 || _Ki < 0 || _Kd < 0)
    return;
  Kp = _Kp;
  Ki = _Ki;
  Kd = _Kd;
}

void PIDController::limit(double min, double max)
{
  minOut = min;
  maxOut = max;
  doLimit = true;
}

double PIDController::compute(double sensor)
{
  unsigned long now = millis();
  timeChange = now - lastTime;

  double error = setPoint - sensor;
  if (Ki != 0)
  {
    errSum += error * timeChange;
    if (doLimit)
      errSum = constrain(errSum, minOut * 1.1 / Ki, maxOut * 1.1 / Ki);
  }

  dErr = 0;
  if (Kd != 0 && timeChange > 0)
    dErr = (error - lastErr) / timeChange;

  double output = Kp * error + Ki * errSum + Kd * dErr;

  if (doLimit)
    output = constrain(output, minOut, maxOut);

  lastErr = error;
  lastTime = now;

  return output;
}
