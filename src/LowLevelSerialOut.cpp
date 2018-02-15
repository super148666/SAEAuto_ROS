/* Copyright 2013 Thomas Drage
 * File:   LowLevelSerialOut.cpp
 * Author: T. Drage
 *
 * Created on 13/7/13
 */


#include <iostream>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <exception>
#include <vector>
#include <cmath>
#include <string>

#include "ros/ros.h"
#include "AsyncSerial.h"
#include <stdlib.h>

#include "LowLevelSerialOut.h"
#include "Logger.h"
#include "Controller.h"

using std::exception;
using std::cerr;
using std::endl;







/**
 * Purpose: Creates a new instance of the LowLevelSerialOut object.
 * Inputs : None.
 * Outputs: None.
 */
LowLevelSerialOut::LowLevelSerialOut(Controller* CarController, Logger* Logger, TripStateMonitor* tripMon) {

  CarControl = CarController;
  Log = Logger;

  this->tripMon = tripMon;

  SerialState = false;
  Run = false;

  LastAckTime = 0;
  WheelSpeeds.reserve(4);
  WheelSpeeds[0] = 0.0;
  WheelSpeeds[1] = 0.0;
  WheelSpeeds[2] = 0.0;
  WheelSpeeds[3] = 0.0;
  RxBuffer.reserve(10);
  Vx = 0.0;
  Vy = 0.0;
}

/**
 * Purpose: Creates a new instance of the LowLevelSerialOut object.
 * Inputs : An Network object.
 * Outputs: None.
 */
LowLevelSerialOut::LowLevelSerialOut(const LowLevelSerialOut& orig) {
}

/**
 * Purpose: Destroys the instance of the LowLevelSerialOut object.
 * Inputs : None.
 * Outputs: None.
 */
LowLevelSerialOut::~LowLevelSerialOut() {
}

bool LowLevelSerialOut::Open() {

  try {
	Log->WriteLogLine(LL_DEVICE);
    Serial = new CallbackAsyncSerial(LL_DEVICE,LL_BAUDRATE);
    Serial->setCallback(boost::bind(&LowLevelSerialOut::Receive,this, _1, _2));

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
    Log->WriteLogLine("LowLevelSerial - Caught exception when opening serial port: " + ExpString);
    return false;
  }
}

void LowLevelSerialOut::Start() {

  if (SerialState) {

    Run = true;
boost::this_thread::sleep(boost::posix_time::milliseconds(300));
    m_Thread = boost::thread(&LowLevelSerialOut::SendCurrent, this);
    m_Thread.detach();
    s_Thread = boost::thread(&LowLevelSerialOut::Monitor, this);
    s_Thread.detach();
  }
}

void LowLevelSerialOut::Stop() {

  Run = false;

  sleep(1);

  if (SerialState) {
    Serial->close();
  }
}


void LowLevelSerialOut::SendCurrent() {

  while (Run) {
    
    if (CarControl->CurrentSteeringSetPosn <= 127 && CarControl->CurrentSteeringSetPosn >= -128) {
      std::string SteeringCommand = "S" + boost::lexical_cast<std::string>(CarControl->CurrentSteeringSetPosn);
      Serial->writeString(SteeringCommand + '\n');
    } else {
      Log->WriteLogLine("LowLevelSerial - Steering set point out of range " + boost::lexical_cast<std::string>(CarControl->CurrentSteeringSetPosn));
      tripMon->trip(5);
    }

    if (CarControl->CurrentThrottleBrakeSetPosn <= 255 && CarControl->CurrentThrottleBrakeSetPosn >= -256) {

      std::string BrakeCommand = "B0";
      std::string ThrottleCommand = "A0";

      if (CarControl->CurrentThrottleBrakeSetPosn < 0) {
        BrakeCommand = "B" + boost::lexical_cast<std::string>(-1*CarControl->CurrentThrottleBrakeSetPosn);
      } else {
        ThrottleCommand = "A" + boost::lexical_cast<std::string>(CarControl->CurrentThrottleBrakeSetPosn);
      }

      Serial->writeString(BrakeCommand + '\n');
      Serial->writeString(ThrottleCommand + '\n');

    } else {
      Log->WriteLogLine("LowLevelSerial - Brake/throttle set point out of range " + boost::lexical_cast<std::string>(CarControl->CurrentThrottleBrakeSetPosn));
      tripMon->trip(5);
    }

    if (CarControl->TripState > 0)  { char E = 'E'; 
Serial->write(&E,1); }
    if (CarControl->ResetTrip == true) {char R = 'R'; 
Serial->write(&R,1); 
CarControl->ResetTrip = false; }

    if (Serial->errorStatus() || Serial->isOpen() == false) {
      Log->WriteLogLine("LowLevelSerial - Error: serial port unexpectedly closed");
      Run = false;
      SerialState = false;
      tripMon->trip(5);
    }
    boost::this_thread::sleep(boost::posix_time::milliseconds(50));

  }

}

void LowLevelSerialOut::Receive(const char *data, unsigned int len) {

  std::vector<char> RxdVector(data,data+len);
  while (RxdVector.size() > 0) {
    if (RxdVector.front() != '\n') {
      RxBuffer.push_back(RxdVector.front());
    } else {
      ProcessMessage();
    }

    RxdVector.erase(RxdVector.begin());

  }

}

bool LowLevelSerialOut::Send(std::string Command) {

  if (SerialState) { 
	Serial->writeString(Command); 
	return true; }
  else return false;

}

void LowLevelSerialOut::ProcessMessage() {

  std::string MsgString(RxBuffer.begin(), RxBuffer.end());
  if (MsgString.compare(0,3,"ACK") == 0) {
    LastAckTime = CarControl->TimeStamp();

  } else if (MsgString.compare(0,2,"ER") == 0) {

    Log->WriteLogLine("LowLevelSerial - Rxd Error! " + MsgString);
    if (MsgString.compare(2,1,"0") == 0) {
      Log->WriteLogLine("LowLevelSerial - Serial overflow!");
    } else if (MsgString.compare(2,1,"1") == 0) {
      Log->WriteLogLine("LowLevelSerial - Emergency brake engaged!");
    } else if (MsgString.compare(2,1,"2") == 0) {
      Log->WriteLogLine("LowLevelSerial - WDT Timeout!");
      tripMon->trip(10);
    } else if (MsgString.compare(2,1,"3") == 0) {
      Log->WriteLogLine("LowLevelSerial - Brake servo on for too long!");
    } else if (MsgString.compare(2,1,"4") == 0) {
      Log->WriteLogLine("LowLevelSerial - Steering control fault!");
    } else if (MsgString.compare(2,1,"5") == 0) {
      Log->WriteLogLine("LowLevelSerial - No new command in 300ms!");
      tripMon->trip(10);
    }   
    } else if (MsgString.compare(0,1,"W") == 0) { //Get Wheel Speeds
        if (MsgString.compare(1,1,"1" ) == 0) {  //Front Left Wheel
	  WheelSpeeds[0]=atof(MsgString.c_str()+3);
        } else if (MsgString.compare(1,1,"2") == 0) { //Front Right Wheel
	  WheelSpeeds[1]=atof(MsgString.c_str()+3);
        } else if (MsgString.compare(1,1,"3") == 0) { //Rear Left Wheel
	  WheelSpeeds[2]=atof(MsgString.c_str()+3);
        } else if (MsgString.compare(1,1,"4") == 0) { //Rear Right Wheel
	  WheelSpeeds[3]=atof(MsgString.c_str()+3);
        }
    } else if (MsgString.compare(0,2,"SV") == 0) { //Get steering angle (the value between -128 and 127)
        SteeringVal = atoi(MsgString.c_str()+3);
    }  else {
      //Log->WriteLogLine("LowLevelSerial - Received non-ack: " + MsgString);
  }

  RxBuffer.clear();

}


 void LowLevelSerialOut::GetOdometryVel(float* vx, float* vy) {
  float VMag = WheelSpeeds[0] + WheelSpeeds[1] + WheelSpeeds[2] + WheelSpeeds[3] / 4.0;

  //mapping steering sensor value to between +/- 30 degrees (or rad)
  float SAngleYaw = ((float)SteeringVal-881.0)*(137.0-881.0)/(2.0*deltaMax) -deltaMax;

  float RadCur = L/SAngleYaw;

  float PsiYaw = VMag/RadCur;

  float slip = atan(Lr*tan(SAngleYaw)/(Lf+Lr));

  *vx = VMag*cos(PsiYaw + slip);
  *vy = VMag*sin(PsiYaw + slip);
} 





void LowLevelSerialOut::Monitor() {


  while (Run) {

    if ((CarControl->TimeStamp() - LastAckTime > 0.3 && LastAckTime > 0) && CarControl->TripState != 10) {
      Log->WriteLogLine("LowLevelSerial - No acks in 300ms!");
      tripMon->trip(10);
    }

    boost::this_thread::sleep(boost::posix_time::milliseconds(400));

  }
}

