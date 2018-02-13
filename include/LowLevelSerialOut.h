/* Copyright 2013 Thomas Drage */
#ifndef CONTROL_SRC_LOWLEVELSERIALOUT_H_
#define CONTROL_SRC_LOWLEVELSERIALOUT_H_

#define LL_BAUDRATE 115200
// UNO #define LL_DEVICE "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_74134373733351609070-if00"
#define LL_DEVICE "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_85634313839351413271-if00"


#include <vector>
#include <string>
#include <boost/thread.hpp>
#include <cmath>
#include "AsyncSerial.h"
#include "Controller.h"
#include "TripStateMonitor.h"
//#include "Control.h"

class Logger;
class Controller;

class LowLevelSerialOut {
public:

  LowLevelSerialOut(Controller* CarController, Logger* Logger, TripStateMonitor* tripMon);
      LowLevelSerialOut(const LowLevelSerialOut& orig);
      virtual ~LowLevelSerialOut();

  bool Open();
  void Start();
  void Stop();
  bool Send(std::string Command);

  void GetOdometryVel(float*,float*);

  bool SerialState;
  std::vector<float> WheelSpeeds;
  int SteeringVal;
 
  //Some car dimensions
  const float L = 1.81;

  const float Lw = 1.29; //between rear wheels

  const float Lr = 0.66; //middle of rear wheels - Centre of grav. (IMU/GPS location) 
  const float Lf = 1.15; // Centre of grav. middle of front wheels 

  const float deltaMax = M_PI/6.0; //highest left turn, radians

  float Vx, Vy;
private:

  std::vector<char> RxBuffer;

  CallbackAsyncSerial* Serial;

  boost::thread m_Thread;
  boost::thread s_Thread;

  bool Run;

  void Receive(const char *data, unsigned int len);

  void ProcessMessage();

  void SendCurrent();

  void Monitor();

  double LastAckTime;

  Controller* CarControl;

  Logger* Log;

  TripStateMonitor* tripMon;
};

#endif // CONTROL_SRC_LOWLEVELSERIALOUT_H_
