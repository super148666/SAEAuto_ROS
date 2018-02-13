/* Copyright 2013 Thomas Drage */
#ifndef CONTROL_SRC_SAFETYSERIALOUT_H_
#define CONTROL_SRC_SAFETYSERIALOUT_H_

#define BAUDRATE 38400
#define DEVICE "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0"

#include <vector>
#include <string>
#include <boost/thread.hpp>

#include "AsyncSerial.h"
#include "TripStateMonitor.h"


class Control;
class Logger;

class SafetySerialOut {
 public:
  SafetySerialOut(Control* CarController, Logger* Logger, TripStateMonitor* tripMon);
      SafetySerialOut(const SafetySerialOut& orig);
      virtual ~SafetySerialOut();

  bool Open();
  void Start();
  void Stop();
  bool Send(char Command);

  bool SerialState;

 private:
  std::vector<char> RxBuffer;

  CallbackAsyncSerial* Serial;

  boost::thread m_Thread;
  boost::thread m_Thread_AckCount;

  std::string ExpectedAck;

  int WrongAckCount;

  bool Run;

  void Receive(const char *data, unsigned int len);

  void ProcessMessage();

  void SendHB();

  void MonitorAckCount();

  Control* CarControl;

  Logger* Log;

  TripStateMonitor* tripMon;
};

#endif  // CONTROL_SRC_SAFETYSERIALOUT_H_

