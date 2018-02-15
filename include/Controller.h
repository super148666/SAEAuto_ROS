/* High level controller for drive by wire
 * 28 July 2017
 * Gabriel Meyer-Lee
 */

#ifndef CONTROL_SRC_CONTROLLER_H_
#define CONTROL_SRC_CONTROLLER_H_


#include "PID.h"
#include "LowLevelSerialOut.h"
#include "Logger.h"
#include "TripStateObs.h"
#include "TripStateMonitor.h"

class LowLevelSerialOut;

class Controller : public TripStateObs {
    public:

	Controller(Logger* Log, bool ExtLogging, TripStateMonitor* tripMon);
	Controller(const Controller& orig);
	virtual ~Controller();

	void Setup();
	void Start();
	void trip(int tripState);
	void untrip();
	int TripState;
	bool BrakeILOn;
    bool ResetTrip;

	void PosUpdate(double DesiredBearing);
	void SpeedUpdate(double CurSpeed);
	void TrackUpdate(double CurTrack);

	void AutoStart(Logger* AutoLogger);
	void AutoStop();
    void AutoPause();
    void AutoContinue();
    bool AutoRun;
    bool AutoOn;

	bool getSerialState();

	int CurrentSteeringSetPosn;
	int CurrentThrottleBrakeSetPosn;

	static double TimeStamp();

	LowLevelSerialOut* LowLevelSerial;
	
    private:

	double DesiredSpeed;

	PID* ThrottleController;
	PID* BrakeController;
	PID* SteeringController;

	double BrakeValue;
	double SpeedIncrement;
    double DesiredBearing;
	bool ExtLogging;


    void TimedBrake();

	Logger* Log;
	Logger* AutoLog;
};

#endif // CONTROL_SRC_CONTROLLER_H_
