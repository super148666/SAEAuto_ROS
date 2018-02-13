/* Copyright 2013 Thomas Drage */
#ifndef CONTROL_SRC_IPC_H_
#define CONTROL_SRC_IPC_H_

#include <string>
#include <iostream>
#include <fstream>

#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include "TripStateMonitor.h"
#include "Control.h"
#include "Controller.h"

class Logger;

class IPC {
 public:
  IPC(Control* CarController, Logger* Logger, TripStateMonitor* tripMon, Controller* Ctrl);
      IPC(const IPC& orig);
      virtual ~IPC();

  void Open();
  void Start();
  void ProcessMessages();

 private:
  bool OpenState;

  bool Run;

  boost::thread m_Thread;
  FILE *RXpipe;

  Control* CarControl;

  Controller* Ctrl;

  Logger* Log;

  TripStateMonitor* tripMon;
};

#endif  // CONTROL_SRC_IPC_H_
