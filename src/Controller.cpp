/* Class that contains high level communication with drive-by-wire controller
 * 28 July 2017
 * Gabriel Meyer-Lee
 */

#include "Controller.h"
#include "PID.h"
#include "LowLevelSerialOut.h"

#include <cmath>
#include <sys/time.h>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <gflags/gflags.h>
DEFINE_double(desired_speed, 1.0, "The desired speed to drive at");

Controller::Controller(Logger* Log, bool ExtLogging, TripStateMonitor* tripMon ) {
    CurrentSteeringSetPosn = 0;
    CurrentThrottleBrakeSetPosn = 0;

    SpeedIncrement = 0;
    BrakeValue = 0;
    DesiredSpeed = 0;
    DesiredBearing = 0;

    ResetTrip = false;
    TripState = 0;

    tripMon->attach(this);

    this->ExtLogging = ExtLogging;
    this->Log = Log;

    BrakeILOn = true;
    AutoOn = false;

    LowLevelSerial = new LowLevelSerialOut(this, Log, tripMon);
}

/**
 * Purpose: Copy Constructor
 * Inputs : ptr to Controller
 * Outputs: None
 */
Controller::Controller(const Controller& orig) { }

/**
 * Purpose: Destructor
 * Inputs : None
 * Outputs: None
 */
Controller::~Controller() {
   LowLevelSerial->Stop();
}

/**
 * Purpose: Starts the connnection with lowelevelserial
 * Inputs : None
 * Outputs: None
 */
void Controller::Setup() {

    LowLevelSerial->Open();
}

/**
 * Purpose: Starts lowelevelserial
 * Inputs : None
 * Outputs: None
 */
void Controller::Start() {
   LowLevelSerial->Start();
}

/**
 * Purpose: Stops autonomous driving
 * Inputs : None
 * Outputs: None
 */
void Controller::AutoStop() {
   CurrentThrottleBrakeSetPosn = 0;
   CurrentSteeringSetPosn = 0;
   boost::thread brake_Thread = boost::thread(&Controller::TimedBrake, this);
   brake_Thread.detach();
   if (ExtLogging) {
    AutoLog->CloseLog();
   }

   AutoRun = false;
   AutoOn = false;
   Log->WriteLogLine("Controller - autonomous stop.");
}

/**
 * Purpose: Starts Autonomous Driving
 * Inputs : ptr to log for Auto stuff
 * Outputs: None
 */
void Controller::AutoStart (Logger* AutoLogger) {
    this->AutoLog = AutoLogger;

    ThrottleController = new PID(10.0, 3.0, 0, 0.1);
    ThrottleController->setInputLimits(0.0, 30);
    ThrottleController->setOutputLimits(-255, 255);
    ThrottleController->setMode(AUTO_MODE);

    BrakeController = new PID(20.0, 4.0, 0, 0.1);
    BrakeController->setInputLimits(0.0, 30);
    BrakeController->setOutputLimits(-255, 255);
    BrakeController->setMode(AUTO_MODE);

    SteeringController = new PID(6.5, 0.5, 0.0, 0.1);
    SteeringController->setInputLimits(-360, 720);
    SteeringController->setOutputLimits(-127, 127);
    SteeringController->setMode(AUTO_MODE);

    SteeringController->setSetPoint(0);
    ThrottleController->setSetPoint(0);

    AutoRun = true;
    AutoOn = true;
}

/**
 * Purpose: Temporarily suspend autonomous driving.
 * Inputs : None.
 * Outputs: None.
 */
void Controller::AutoPause() {
  AutoRun = false;
  boost::thread brake_Thread = boost::thread(&Controller::TimedBrake, this);
  brake_Thread.detach();

  Log->WriteLogLine("Control - autonomous pause.");
}

/**
 * Purpose: Resume autonomous driving.
 * Inputs : None.
 * Outputs: None.
 */
void Controller::AutoContinue() {
  AutoRun = true;

  Log->WriteLogLine("Control - autonomous continue.");
}

/**
 * Purpose: Updates controller set points when a new position is received during
 * autonomous driving.
 * Inputs : DesiredBearing
 * Outputs: None.
 */
void Controller::PosUpdate( double DesiredBearing) {
    SteeringController->setSetPoint(DesiredBearing);

    // Speed profile based on turn radius.
    //DesiredSpeed = ((fabs(CurrentSteeringSetPosn)) < 50 ? 2.5 : 1.0);
    DesiredSpeed = FLAGS_desired_speed;

    ThrottleController->setSetPoint(DesiredSpeed);
    BrakeController->setSetPoint(DesiredSpeed);

    if (ExtLogging) {
	std::string time = boost::lexical_cast<std::string>(TimeStamp());
	AutoLog->WriteLogLine("DB, "
		+ time
		+ ", "
		+ boost::lexical_cast<std::string>(DesiredBearing), true);
	AutoLog->WriteLogLine("DS, "
		+ time
		+ ", "
		+ boost::lexical_cast<std::string>(DesiredSpeed), true);
    }
    this->DesiredBearing = DesiredBearing;
}

/**
 * Purpose: Performs control loop actions when a new heading
 * is received during autonomous driving.
 * Inputs : New heading value.
 * Outputs: None.
 */
void Controller::TrackUpdate( double CurTrack) {
    if ((DesiredBearing<0 && CurTrack>180) || (DesiredBearing<180 && CurTrack>360)) {
	  DesiredBearing = DesiredBearing + 360;
	  SteeringController->setSetPoint(DesiredBearing);
    }
    if ((DesiredBearing>360 && CurTrack<180) || (DesiredBearing>180 && CurTrack<0)) {
	  DesiredBearing = DesiredBearing - 360;
	  SteeringController->setSetPoint(DesiredBearing);
    }

    SteeringController->setProcessValue(CurTrack);
    CurrentSteeringSetPosn = SteeringController->compute();

    if (ExtLogging) {
      AutoLog->WriteLogLine("SS, "
          + boost::lexical_cast<std::string>(TimeStamp())
          + ", "
          + boost::lexical_cast<std::string>(CurrentSteeringSetPosn), true);
    }

}

/**
 * Purpose: Performs control loop actions when
 * a new speed is received during autonomous driving.
 * Inputs : New speed value.
 * Outputs: None.
 */
void Controller::SpeedUpdate( double CurSpeed) {
    ThrottleController->setProcessValue(CurSpeed);
    BrakeController->setProcessValue(CurSpeed);

    SpeedIncrement = ThrottleController->compute();
    BrakeValue = BrakeController->compute();
    Log->WriteLogLine("BV:" + boost::lexical_cast<std::string>(BrakeValue));
    std::string time;

    double newSpeed = CurrentThrottleBrakeSetPosn + SpeedIncrement;

    if (TripState == 0) {
        if (newSpeed > 255) {
	    CurrentThrottleBrakeSetPosn = 255;
	} else if (newSpeed < 0 && BrakeValue < 0) {
	    if (BrakeValue > -255) {
	        CurrentThrottleBrakeSetPosn = BrakeValue;
	    } else {
		BrakeValue = -255;
	    }
	} else if (newSpeed > 0) {
	    CurrentThrottleBrakeSetPosn += SpeedIncrement;
	} else {
	    CurrentThrottleBrakeSetPosn = 0;
	}
    } else {
	CurrentThrottleBrakeSetPosn = (BrakeValue < 0) ? BrakeValue : 0;
    }

    if ( ExtLogging ) {
	    time = boost::lexical_cast<std::string>(TimeStamp());
	    AutoLog->WriteLogLine("SI, "
		+ time
		+ ", "
		+ boost::lexical_cast<std::string>(SpeedIncrement), true);
            AutoLog->WriteLogLine("BV, "
		+ time
		+ ", "
		+ boost::lexical_cast<std::string>(BrakeValue), true);
            AutoLog->WriteLogLine("TB, "
		+ time
		+ ", "
		+ boost::lexical_cast<std::string>(CurrentThrottleBrakeSetPosn), true);
    }

}

/**
 * Purpose: Returns state of lowlevelserial
 * Inputs : None
 * Outputs: bool state
 */
bool Controller::getSerialState() {
    return LowLevelSerial->SerialState;
}

/**
 * Purpose: Resets trip state, restarts low level serial
 * Inputs : None
 * Outputs: None
 */
void Controller::untrip() {
    CurrentThrottleBrakeSetPosn = 0;
    this->TripState = 0;
    this->ResetTrip = true;

    if (LowLevelSerial->SerialState == false){
	  LowLevelSerial->Open();
	  LowLevelSerial->Start();
    }
}

/**
 * Purpose: When tripped, brakes and publishes reason.
 * Inputs : int new trip state
 * Outputs: None.
 */
void Controller::trip(int tripState) {
    this->TripState = tripState;
    std::string TripReason;

    switch (TripState) {
        case 1 : TripReason = "Base ESTOP";
                 break;
        case 2 : TripReason = "Couldn't send HB";
                 break;
        case 3 : TripReason = "Safety supervisor initiated trip.";
                 break;
        case 4 : TripReason = "Safety supervisor msgs not ack'd.";
                 break;
        case 5 : TripReason = "Error sending low level commands.";
                 break;
        case 6 : TripReason = "Network error!";
                 break;
        case 7 : TripReason = "GPS Error";
                 break;
        case 8 : TripReason = "Autonomous issue";
                 break;
        case 9 : TripReason = "Web IPC estop";
                 break;
        case 10 : TripReason = "Low Level Error";
                  break;
        case 11 : TripReason = "HB (Serial) Timeout";
                  break;
    }

    AutoRun = false;
    AutoOn = false;

    if (BrakeILOn) {
      CurrentThrottleBrakeSetPosn = -256;
    } else {
      CurrentThrottleBrakeSetPosn = 0;
    }
    Log->WriteLogLine("Controller - changed to trip state " +
		    boost::lexical_cast<std::string>(TripState) +
		    " reason " + TripReason);
}

/**
 * Purpose: Record time stamp.
 * Inputs : None.
 * Outputs: Current time as double.
 */
double Controller::TimeStamp() {
    timeval current;
    gettimeofday(&current, NULL);

    return current.tv_sec + static_cast<double>(current.tv_usec)/1000000;
}


/**
 * Purpose: Brake for two seconds.
 * Needs to be run its own thread as to not block execution.
 * Inputs : None.
 * Outputs: None.
 */
void Controller::TimedBrake() {
  Log->WriteLogLine("Controller - Braking for 2s.");

  CurrentThrottleBrakeSetPosn = -255;
  boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
  CurrentThrottleBrakeSetPosn = 0;
}
