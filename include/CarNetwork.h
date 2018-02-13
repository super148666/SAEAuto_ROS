/* Copyright 2013 Thomas Drage
 * */
#ifndef CONTROL_SRC_CARNETWORK_H_
#define CONTROL_SRC_CARNETWORK_H_

#define LISTEN_PORT 1100

#include <string>

#include <boost/thread.hpp>

#include "Controller.h"
#include "TripStateMonitor.h"

class Control;
class Logger;

class CarNetwork {
public:

  CarNetwork(Control* CarController, Logger* Logger, Controller* Controller, TripStateMonitor* tripMon);
      CarNetwork(const CarNetwork& orig);
      virtual ~CarNetwork();

  bool IsConnected();
  void Disconnect();
  bool Open();
  void StartProcessMessages();

  bool HasConnection;

  std::string StatusString;


private:

  int SocketFD;

  bool Run;

  boost::thread m_Thread;

  void ProcessMessages();

  Control* CarControl;

  Controller* Ctrl;

  Logger* Log;

  TripStateMonitor* tripMon;

  timeval last,current;

};

#endif  // CONTROL_SRC_CARNETWORK_H_
