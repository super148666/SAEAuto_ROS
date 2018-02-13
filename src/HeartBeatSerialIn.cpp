/* Copyright 2017 Thomas Drage
 * File:   HeartBeatSerialIn.cpp
 * Author: T. Drage
 *
 * Created on 1/7/17
 */

#include "HeartBeatSerialIn.h"

#include <sys/time.h>

#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <exception>
#include <vector>
#include <string>
#include <algorithm>

#include "AsyncSerial.h"

#include "Logger.h"
#include "Control.h"

using std::exception;
using std::cerr;
using std::endl;

/**
 * Purpose: Creates a new instance of the HeartBeatSerialIn object.
 * Inputs : None.
 * Outputs: None.
 */
HeartBeatSerialIn::HeartBeatSerialIn(Control* CarController, Logger* Logger, TripStateMonitor* tripMon) {
  CarControl = CarController;
  Log = Logger;

  this->tripMon = tripMon;

  SerialState = false;
  Run = false;

  monitorcount = 0;
  heartbeaton = false;
  msgflag = false;
  rssiflag = false;
  rssi = 0;

  RxBuffer.reserve(10);
}

/**
 * Purpose: Creates a new instance of the HeartBeatSerialIn object.
 * Inputs : An HeartBeatSerialIn object.
 * Outputs: None.
 */
HeartBeatSerialIn::HeartBeatSerialIn(const HeartBeatSerialIn& orig) {
}

/**
 * Purpose: Destroys the instance of the HeatBeatSerial object.
 * Inputs : None.
 * Outputs: None.
 */
HeartBeatSerialIn::~HeartBeatSerialIn() {
}

bool HeartBeatSerialIn::Open() {
  try {
    Serial = new CallbackAsyncSerial(HB_DEVICE, HB_BAUDRATE);
    Serial->setCallback(boost::bind(&HeartBeatSerialIn::Receive, this, _1, _2));

    if (Serial->errorStatus() || Serial->isOpen() == false) {
      cerr << "Error: serial port unexpectedly closed" << endl;
      return false;
    } else {
      SerialState = true;
      return true;
    }
  }
  catch(std::exception& e) {
    std::string ExpString(e.what());
    Log->WriteLogLine("HeartBeatSerialIn - Caught exception when opening serial port: "
                      + ExpString);
    return false;
  }
}

/**
 * Purpose: Sets the run flag and creates the monitor thread.
 * Inputs : None.
 * Outputs: None.
 */
void HeartBeatSerialIn::Start() {
  if (SerialState) {
    Run = true;
    gettimeofday(&last, NULL);
    m_Thread = boost::thread(&HeartBeatSerialIn::Monitor, this);
    m_Thread.detach();
  }
}

/**
 * Purpose: Clears the run flag and closes the serialport.
 * Inputs : None.
 * Outputs: None.
 */
void HeartBeatSerialIn::Stop() {
  Run = false;

  sleep(1);

  if (SerialState) {
    Serial->close();
  }
}

/**
 * Purpose: Construct messages from data received from serialport on callback.
 * Inputs : Data character pointer, length to read.
 * Outputs: None.
 */
void HeartBeatSerialIn::Receive(const char *data, unsigned int len) {
  std::vector<char> RxdVector(data, data+len);

  while (RxdVector.size() > 0) {
    if (RxdVector.front() != '\n' && RxdVector.front() != '\r') {
      RxBuffer.push_back(RxdVector.front());
    } else {
      ProcessMessage();
    }

    RxdVector.erase(RxdVector.begin());
  }
}

/**
 * Purpose: Act on received heartbeat / RSSI message sequences.
 * Inputs : None.
 * Outputs: None.
 */
void HeartBeatSerialIn::ProcessMessage() {
    std::string MsgString(RxBuffer.begin(), RxBuffer.end());

    if(MsgString.length() > 0) {

      if (MsgString.compare(0, 3, "HBT") == 0) {
        if(! heartbeaton) { heartbeaton = true; }
        gettimeofday(&last, NULL);

        if (MsgString.compare(4, 1, "+") == 0) {
          CarControl->HeartbeatState = true;
        } else if (MsgString.compare(4, 1, "-") == 0) {
          CarControl->HeartbeatState = false;
        }
      } else if (rssiflag && std::all_of(MsgString.begin(), MsgString.end(), ::isdigit)) {
        rssi = boost::lexical_cast<int>(MsgString);
        rssiflag = false;
      } else if (MsgString.compare(0,5,"ESTOP") == 0) {
        tripMon->trip(1);
      } else if (MsgString.compare(0,4,"BATT") == 0) {
        Log->WriteLogLine("HBSerial Rx:" + MsgString);
      }


    }

    RxBuffer.clear();
}

/**
 * Purpose: Check timing of received heartbeat messages
 * and eriodically request the RSSI of received messages from XBee.
 * Inputs : None.
 * Outputs: None.
 */
void HeartBeatSerialIn::Monitor() {
  while (Run) {
    gettimeofday(&current, NULL);
    int ms_gap = (current.tv_usec - last.tv_usec)/1000 + (current.tv_sec - last.tv_sec)*1000;
    if (ms_gap > HB_INTERVAL*7/3 && heartbeaton) {  // Missed 2 heartbeats.
      if (! msgflag) {
        Log->WriteLogLine("HeartBeatSerialIn - Slow response on HB! "
                        + boost::lexical_cast<std::string>(ms_gap));
        msgflag = true;
      } else if (ms_gap > HB_INTERVAL*10/3 && heartbeaton) {  // Missed 3 heartbeats.
        if (CarControl->BrakeILOn) {
          tripMon->trip(11);
        }
        heartbeaton = false;
      }
    } else if(msgflag) {
      msgflag = false;
    }

    if (monitorcount > HB_INTERVAL) {
      Serial->writeString("+++");
      boost::this_thread::sleep(boost::posix_time::milliseconds(275));
      rssiflag = true;
      Serial->writeString("ATDB\r\n");
      boost::this_thread::sleep(boost::posix_time::milliseconds(20));
      Serial->writeString("ATCN\r\n");
      monitorcount = 0;
    }

    monitorcount++;
    boost::this_thread::sleep(boost::posix_time::milliseconds(HB_INTERVAL/6));
  }
}

bool HeartBeatSerialIn::HeartBeatOn() {
  return heartbeaton;
}