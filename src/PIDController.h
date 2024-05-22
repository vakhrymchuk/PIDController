#pragma once

#ifdef ARDUINO
#include "Arduino.h"
#endif

class PIDController
{
public:
  double compute(double input);

  void tune(double _Kp, double _Ki, double _Kd);
  void limit(double min, double max);
  void setpoint(double newSetpoint);
  void resetLastTime();
  double getP();
  double getI();
  double getD();
  double getTimeChange();

private:
  unsigned long lastTime = millis();
  unsigned long timeChange = 0;

  double lastErr = 0;

  double errSum = 0;
  double dErr = 0;

  bool doLimit = false;

  double Kp = 1;
  double Ki = 0;
  double Kd = 0;
  double minOut;
  double maxOut;
  double setPoint = 0;
};
