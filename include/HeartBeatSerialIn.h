/* Copyright 2017 Thomas Drage */
#ifndef CONTROL_SRC_HEARTBEATSERIALIN_H_
#define CONTROL_SRC_HEARTBEATSERIALIN_H_

#define HB_INTERVAL 300

#define HB_BAUDRATE 9600
#define HB_DEVICE "/dev/ttyS0"

#include <vector>
#include <string>
#include <boost/thread.hpp>

#include "AsyncSerial.h"
#include "TripStateMonitor.h"


class Control;
class Logger;

class HeartBeatSerialIn {
public:

  HeartBeatSerialIn(Control* CarController, Logger* Logger, TripStateMonitor* tripMon);
      HeartBeatSerialIn(const HeartBeatSerialIn& orig);
      virtual ~HeartBeatSerialIn();

  bool Open();
  void Start();
  bool HeartBeatOn();
  void Stop();


  bool SerialState;
  int  rssi;

private:

  std::vector<char> RxBuffer;

  CallbackAsyncSerial* Serial;

  boost::thread m_Thread;

  bool Run;

  int monitorcount;
  bool heartbeaton;
  bool msgflag;
  bool rssiflag;

  void Receive(const char *data, unsigned int len);

  void ProcessMessage();

  void Monitor();

  timeval last,current;

  Control* CarControl;

  Logger* Log;

  TripStateMonitor* tripMon;

};

#endif  // CONTROL_SRC_HEARTBEATSERIALIN_H_
