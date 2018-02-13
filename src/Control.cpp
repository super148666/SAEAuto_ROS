/*
* Copyright 2013 Thomas Drage
* UWA Autonomous SAE Car Controller
*
* Created June 2013
*
* Permission is given for use of this software in derivative works within the
* UWA Robotics and Automation laboratory,
* however acknowledgement must be given in all derived works and publications.
*
*/

#include "Control.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <curses.h>
#include <sys/time.h>
#include <stdio.h>

#include "CarNetwork.h"
#include "SafetySerialOut.h"
#include "HeartBeatSerialIn.h"
#include "IPC.h"

/*
static timestamp_t get_timestamp() {
  struct timeval now;
  gettimeofday(&now, NULL);
  return now.tv_usec + (timestamp_t)now.tv_sec * 1000000;
}
*/

Control *SAECar;

/**
 * Purpose: Creates a new instance of the Control object.
 * Inputs : Name of directory logs are to be saved in.
 * Outputs: None.
 */
Control::Control(std::string LogDir, bool ExtLog, TripStateMonitor *tripMon) {
  TwoPi = 4 * acos(0);

  this->LogDir = LogDir;
  this->ExtLogging = ExtLog;

  tripMon->attach(this);

  this->tripMon = tripMon;

  // Create directories for logging.
  mode_t process_mask = umask(0);
  mkdir(this->LogDir.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
  mkdir((LogDir + "/luxscan").c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
  mkdir((LogDir + "/luxobj").c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
  umask(process_mask);

  // Initialise variables...
  HeartbeatState = false;
  TripState = 0;
  ManualOn = false;
  BrakeILOn = true;

  DatumLat = -31.980569;
  DatumLong = 115.817807;
  LatOffset = 0.0;
  LongOffset = 0.0;

  LogLevel = "Debug";

  // Create object instances...
  Log = new Logger(LogDir + "/mainlog.txt", 1);
  TimeLog = new Logger(LogDir + "/timelog.txt");
  Log->WriteLogLine("Control - Ext logging: " +
                    boost::lexical_cast<std::string>(ExtLogging) + " " +
                    LogDir);

  SafetySerial = new SafetySerialOut(this, Log, tripMon);
  Ctrl = new Controller(Log, ExtLog, tripMon);
  CarNetworkConnection = new CarNetwork(this, Log, Ctrl, tripMon);
  WebIPC = new IPC(this, Log, tripMon, Ctrl);
  WebLogger = new Logger("./ramdisk/weblog.txt");
  HeartBeatSerial = new HeartBeatSerialIn(this, Log, tripMon);
}

Control::~Control() {}

/**
 * Purpose: Runs set up routines on all sensors/interfaces.
 * Inputs : None.
 * Outputs: None.
 */
void Control::Setup() {
  WebIPC->Open();
  CarNetworkConnection->Open();
  SafetySerial->Open();
  Ctrl->Setup();
  HeartBeatSerial->Open();
}

/**
 * Purpose: Handles cleanly closing the program.
 * Inputs : None.
 * Outputs: None.
 */
void Control::Quit() {
  Log->WriteLogLine("Caught SIGINT...bye!");
  Log->CloseLog();
  CarNetworkConnection->Disconnect();
  SafetySerial->Stop();
  delete Ctrl;
  HeartBeatSerial->Stop();
  RunState = false;
}

/**
 * Purpose: Runs the main loop for Control class.
 * Inputs : None.
 * Outputs: None.
 */
void Control::Run() {
  RunState = true;

  // Start all sensors/interfaces.
  WebIPC->Start();
  CarNetworkConnection->StartProcessMessages();
  SafetySerial->Start();
  HeartBeatSerial->Start();

  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  SendAlarm();
  boost::this_thread::sleep(boost::posix_time::milliseconds(400));

  Ctrl->Start();

    UpdateTerminal();
    WriteInfoFile();

    boost::this_thread::sleep(boost::posix_time::milliseconds(250));

}

/**
 * Purpose: Updates the terminal display.
 * Inputs : None.
 * Outputs: None.
 */
void Control::UpdateTerminal() {
  mvprintw(1, 0, "Manual State: %i \n", this->ManualOn);
  mvprintw(2, 0, "Trip State: %i \n", this->TripState);
  mvprintw(3, 0, "HB State: %i \n", this->HeartbeatState);
  mvprintw(4, 0, "Car Net State: %s \n",
           this->CarNetworkConnection->StatusString.c_str());
  mvprintw(5, 0, "Safety Serial State: %i \n", this->SafetySerial->SerialState);
  mvprintw(6, 0, "LowLevel Serial State: %i \n", this->Ctrl->getSerialState());
  mvprintw(7, 0, "Brake IL Status: %i \n", this->BrakeILOn);

  mvprintw(9, 0, "Current Steering Posn: %i \n", Ctrl->CurrentSteeringSetPosn);
  mvprintw(10, 0, "Current Throttle/Brake Level: %i \n",
           Ctrl->CurrentThrottleBrakeSetPosn);

  int y, x;
  getmaxyx(stdscr, y, x);

  int TitleX = 0;

  if ((x / 2 - 28) > 0) {
    TitleX = x / 2 - 28;
  }

  mvprintw(0, TitleX,
           "UWA AUTONOMOUS SAE CAR CONTROLLER - THOMAS DRAGE 2013\n");

  int LogStartLine = 14;

  if (y > LogStartLine) {
    std::string *RecentLogLines = new std::string[y - LogStartLine];
    this->Log->GetLogLines(RecentLogLines, y - LogStartLine);

    for (int i = y; i > LogStartLine; i--) {
      mvprintw(i - 1, 0, "Log: %s", (RecentLogLines[y - i] + "\n").c_str());
    }
  }

  refresh();
}

/**
 * Purpose: Updates the weblog.txt info output file.
 * Inputs : None.
 * Outputs: None.
 */
void Control::WriteInfoFile() {
  WebLogger->WriteLock();
  WebLogger->ClearLog();

  WebLogger->WriteLogLine(
      "Man Sts|" + boost::lexical_cast<std::string>(this->ManualOn), true);
  WebLogger->WriteLogLine(
      "Auto On|" + boost::lexical_cast<std::string>(Ctrl->AutoOn), true);
  WebLogger->WriteLogLine(
      "Auto Run|" + boost::lexical_cast<std::string>(Ctrl->AutoRun), true);
  WebLogger->WriteLogLine(
      "Trip Sts|" + boost::lexical_cast<std::string>(this->TripState), true);
  WebLogger->WriteLogLine(
      "HB Sts|" + boost::lexical_cast<std::string>(this->HeartbeatState), true);
  WebLogger->WriteLogLine(
      "CarNet Sts|" + boost::lexical_cast<std::string>(
                          this->CarNetworkConnection->StatusString.c_str()),
      true);
  WebLogger->WriteLogLine(
      "SafetySerial Sts|" +
          boost::lexical_cast<std::string>(this->SafetySerial->SerialState),
      true);
  WebLogger->WriteLogLine(
      "LowLevelSerial Sts|" +
          boost::lexical_cast<std::string>(this->Ctrl->getSerialState()),
      true);

  WebLogger->WriteLogLine(
      "Safety Mode|" + boost::lexical_cast<std::string>(this->BrakeILOn), true);

  WebLogger->WriteLogLine("Steer Posn|" + boost::lexical_cast<std::string>(
                                              Ctrl->CurrentSteeringSetPosn),
                          true);
  WebLogger->WriteLogLine(
      "Throttle/Brake Lvl|" +
          boost::lexical_cast<std::string>(Ctrl->CurrentThrottleBrakeSetPosn),
      true);

  WebLogger->WriteLogLine(
      "Datum Lat|" + boost::lexical_cast<std::string>(this->DatumLat), true);
  WebLogger->WriteLogLine(
      "Datum Long|" + boost::lexical_cast<std::string>(this->DatumLong), true);

  WebLogger->WriteLogLine(
      "Offset Lat|" + boost::lexical_cast<std::string>(this->LatOffset), true);
  WebLogger->WriteLogLine(
      "Offset Long|" + boost::lexical_cast<std::string>(this->LongOffset),
      true);

  WebLogger->WriteLogLine("Desired Bearing|" + boost::lexical_cast<std::string>(
                                                   this->DesiredBearing),
                          true);

   WebLogger->WriteLogLine("Wheel Speed FL|" + boost::lexical_cast<std::string>(this->Ctrl->LowLevelSerial->WheelSpeeds[0]), true);
   WebLogger->WriteLogLine("Wheel Speed FR|" + boost::lexical_cast<std::string>(this->Ctrl->LowLevelSerial->WheelSpeeds[1]), true);
   WebLogger->WriteLogLine("Wheel Speed BL|" + boost::lexical_cast<std::string>(this->Ctrl->LowLevelSerial->WheelSpeeds[2]), true);
   WebLogger->WriteLogLine("Wheel Speed BR|" + boost::lexical_cast<std::string>(this->Ctrl->LowLevelSerial->WheelSpeeds[3]), true);

  WebLogger->WriteLogLine("HBSerial RSSI|" + boost::lexical_cast<std::string>(
                                                 HeartBeatSerial->rssi),
                          true);

  int y = 50;

  std::string *RecentLogLines = new std::string[y];
  this->Log->GetLogLines(RecentLogLines, y);

  for (int i = y; i > 0; i--) {
    WebLogger->WriteLogLine("Log|" + RecentLogLines[y - i], true);
  }

  WebLogger->ClearLock();
}

/**
 * Purpose: Actions trips.
 * Inputs : The new trip state (type of trip).
 * Outputs: None.
 */
void Control::trip(int tripState) {

  this->TripState = tripState;
  // Stop things from happening...

  Ctrl->BrakeILOn = this->BrakeILOn;
}

/**
 * Purpose: Sets the trip state back to zero and turns off the E-brake.
 * Does not reset hardware supervisor.
 * Inputs : None.
 * Outputs: None.
 */
void Control::untrip() {

  if (SafetySerial->SerialState == false) {
    SafetySerial->Open();
    SafetySerial->Start();
  }

  this->TripState = 0;

  SafetySerial->Send('R');
  Log->WriteLogLine("Control - trip state returned to zero.");
}

/**
 * Purpose: Toggle the safety mode (BrakeIL) from human driving
 * to no driver and vice-versa...
 * Inputs : None.
 * Outputs: None.
 */
void Control::ToggleBrakeIL() {
  bool Success;

  if (!this->BrakeILOn) {
    Success = SafetySerial->Send('B');
  } else {
    Success = SafetySerial->Send('H');
  }

  if (Success) {
    this->BrakeILOn = !this->BrakeILOn;
    Log->WriteLogLine("Control - Brake IL toggled " + this->BrakeILOn);
  } else {
    Log->WriteLogLine("Control - Brake IL toggle failed.");
  }
}


/** Purpose: Starts Autonomous driving.
 * Inputs : None.
 * Outputs: None.
 */
void Control::AutoStart() {
  if (ExtLogging) {
    AutoLogger = new Logger(LogDir + "/autolog.txt");
  }

  if ((!(CarNetworkConnection->HasConnection || HeartBeatSerial->HeartBeatOn()) && BrakeILOn)) {
    Log->WriteLogLine("Control - No base connection, can't start auto.");
    return;
  }
  ManualOn = false;

  Ctrl->AutoStart(AutoLogger);
}


/**
 * Purpose: Tell the safety serial box to beep.
 * Inputs : None.
 * Outputs: None.
 */
void Control::SendAlarm() {
  if (SafetySerial->SerialState) {
    SafetySerial->Send('A');
  }
}

// Utility functions below.

double Control::TimeStamp() {
  timeval current;
  gettimeofday(&current, NULL);

  return current.tv_sec + static_cast<double>(current.tv_usec) / 1000000;
}

void TerminateHandler() {
  if (SAECar != NULL && SAECar->Log->IsOpen()){
    SAECar->Log->WriteLogLine("Control - Unhandled exception occurred. Exiting\n");
    SAECar->WriteInfoFile();
  } else {
    std::cout << "Control - Unhandled exception occurred. Exiting\n";
  }
  if (SAECar != NULL){
    SAECar->Quit();
  }
}
void HandleSeg(int param) {
  if (SAECar != NULL && SAECar->Log->IsOpen()){
    SAECar->Log->WriteLogLine("Control - Segmentation Fault occurred. Exiting\n");
    SAECar->Log->CloseLog();
    SAECar->WriteInfoFile();
  } else {
    std::cout << "Control - Segmentation Fault occurred. Exiting\n";
  }
  // Segmentation Fault considered non-recoverable
  exit(1);
}
// C-style non -member functions follow.

void HandleExit(int param) { SAECar->Quit(); }



/*
* Entry point to the program.
* Argument to executable is the name of the folder to save logs in for this run.
* Default folder name is '0'.
*/
int main(int argc, char *argv[]) {
  std::string LogDir;

  bool ExtLogging = false;

  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (argc > 1) {
    std::string Arg = boost::lexical_cast<std::string>(argv[1]);
    if (Arg.compare(0, 4, "null") == 0) {
      ExtLogging = false;
    } else {
      ExtLogging = true;
    }
    LogDir = "RunFiles/" + Arg;
  } else {
    system("rm -rf ./RunFiles/0");
    LogDir = "RunFiles/"+std::to_string((long)time(NULL));
  }
  FLAGS_log_dir=LogDir;
  google::InitGoogleLogging(argv[0]);

  // Set up the ncurses terminal display.
  initscr();
  noecho();
  nodelay(stdscr, 1);
  keypad(stdscr, 1);

  TripStateMonitor *tripMon = new TripStateMonitor();

  SAECar = new Control(LogDir, ExtLogging, tripMon);


  // Setup exit handlers
  std::signal(SIGINT, HandleExit);
  std::signal(SIGSEGV, HandleSeg);
  std::set_terminate(&TerminateHandler);

  try {
    SAECar->Setup();
  } catch(const std::exception& e){
      if (SAECar != NULL && SAECar->Log->IsOpen()){
        SAECar->Log->WriteLogLine("Control - Unhandled exception occurred setting up control.\n");
        SAECar->Log->WriteLogLine(e.what());
        SAECar->Log->WriteLogLine("\nExiting\n");
        SAECar->Log->CloseLog();
        SAECar->WriteInfoFile();

      } else {
        std::cout << "Control - Unhandled exception occurred setting up control\n" << e.what() << "\nExiting\n";
      }
      exit(1);
      
  }

  try {
    SAECar->Run(); // Execution happens in this function.
  } catch (const std::exception& e){
      if (SAECar != NULL && SAECar->Log->IsOpen()){
        SAECar->Log->WriteLogLine("Control - Unhandled exception occurred running control.\n");
        SAECar->Log->WriteLogLine(e.what());
        SAECar->Log->WriteLogLine("\nExiting\n");
        SAECar->Log->CloseLog();
        SAECar->WriteInfoFile();
      } else {
        std::cout << "Control - Unhandled exception occurred running control\n" << e.what() << "\nExiting\n";
      }
      exit(1);
  }
  std::cout << "Close.\n";

  echo();
  endwin();

  return 0;
}
