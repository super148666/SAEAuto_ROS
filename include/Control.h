/* Copyright 2013 Thomas Drage
 * File:   Control.h - Header file for Control.cpp
 * Author: Thomas Drage (20510505).
 *
 * Date: 2013
 *
 */


#ifndef CONTROL_SRC_CONTROL_H_
#define CONTROL_SRC_CONTROL_H_

#define EARTH_RADIUS 6371000
#define MAPPOINT_RADIUS 2.75
#define MAP_PATH "../front_end/Maps/maps/"

//Path planning constants
#define PATHESTIMATEGRANULARITY 1
#define GRANULARITY 0.5 //Set this with consideration to mappoint radius above
#define ROADEDGERADIUS 0.5 //Equal to Granularity
#define EPSILON 0.01

#define LIDAR_DATA_PATH "../Lidardata.obj" //this isn't a real file
#define LIDAR_DRIVING 1

#define OBJECTNOTIFYTIME 3 //Seconds between object type notifications

#define Pi 22/7

#include <vector>
#include <string>

#include <boost/thread.hpp>

#include "Controller.h"
#include "TripStateObs.h"
#include "TripStateMonitor.h"

class CarNetwork;
class Logger;
class SafetySerialOut;
class IPC;
class HeartBeatSerialIn;



class Control : public TripStateObs {
 public:

    double TwoPi;

    Control(std::string LogDir, bool ExtLog, TripStateMonitor* tripMon);
    Control(const Control& orig);
    virtual ~Control();

    void Setup();
    void Run();
    void Quit();
    void trip(int tripState);
    void untrip();
    void ToggleBrakeIL();
    void SendAlarm();

    bool HeartbeatState;
    int TripState;
    bool ManualOn;
    bool BrakeILOn; // If this is off we don't require a heartbeat or network connection for auto!

    std::string LogLevel;

    double DatumLat;
    double DatumLong;

    void AutoStart();

    double LatOffset;
    double LongOffset;

    std::string LogDir;
    bool ExtLogging;

    static double TimeStamp();
    Controller* Ctrl;
    Logger* Log;

    void WriteInfoFile();
 private:
    bool RunState;

    void UpdateTerminal();

    double DesiredBearing;
    double LastKnownAngle;

    CarNetwork* CarNetworkConnection;
    Logger* TimeLog;
    SafetySerialOut* SafetySerial;
    IPC* WebIPC;
    HeartBeatSerialIn* HeartBeatSerial;

    Logger* WebLogger;

    Logger* AutoLogger;

    TripStateMonitor* tripMon;

    boost::mutex PlanLock;

};



#endif  // CONTROL_SRC_CONTROL_H_
