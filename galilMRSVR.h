#ifndef _galilMRSVR_h
#define _galilMRSVR_h

#include "Galil.h"
#include <iostream>
#include <stdexcept>
#include <string>

//#include "dmccom.h" //this is the Galil dmc communication module. 

#define NUM_ACTUATORS 2
#define NUM_ENCODERS 2
#define MAX_AXES 2

class galilMRSVR
{
public:
	enum AXES {X,Y};

private:
	Galil *g; ///handle to Galil controller
	double speed, accel, decel; ///each axis variable
	bool isInit; ///if the robot has been initialized				
	bool homed; ///if the robot has been homed

	float cnts_per_mm[MAX_AXES]; ///encoder scale factors
	float axis_home[MAX_AXES]; ///axis home position
	float axis_ulim[MAX_AXES]; ///axis upper limit 
	float axis_llim[MAX_AXES]; ///axis lower limit

	///PID control gains for initializations
	float pid_kp[MAX_AXES];
	float pid_ki[MAX_AXES];
	float pid_kd[MAX_AXES];

	float curlim_avg[MAX_AXES]; /// average current (torque) limit for each axis

	///US motor specific parameters
	float deadBandOff[MAX_AXES];
	float deadBandOn[MAX_AXES];
	float antiFricPos[MAX_AXES];
	float antiFricNeg[MAX_AXES];
	double time_threshold;

	/// Returns true if goal is inside joint limit for specified axis
	bool InsideLimits(AXES axis, double goal) const
	{return ((goal >= axis_llim[axis]) && (goal <= axis_ulim[axis])); }

public:
	
	galilMRSVR(const std::string& address ="");/// throw();
	virtual ~galilMRSVR();
	
	///returns true if Galil contorller is present and initialized
	bool IsInit() const {return isInit;}

	//Enable/disable motor power
	double EnableMotorPower(); 
	double DisableMotorPower();
	
	///Stop robot without disabling the power
	void Stop();
	///Home robot
	bool Home();
	//Unhome robot
	void UnHome();
	///Return true if robot is homed
	inline bool IsHomed() const throw() {return homed;}

	///Return Axes limits
	///float GetAxesUpperLimits()[2] const throw();
	///float GetAxesLowerLimits()[2] const throw();

	///Set Axes limits
	///void SetAxesUpperLimits(const &limit) throw();
	///void SetAxesLowerLimits(const &limit) throw();
	void MoveAxes(double x, double y) const;
	void MoveAxesInct(double x, double y) const;
	double GetPositionX() const;
	double GetPositionY() const;
	double GetAccel() const;
	double GetDecel()const ;
	double GetSpeed() const;
	void SetSpeed(double spd) const;
	void SetAccel(double x) const ;
	void SetDecel(double x) const;
	void WaitMotion();
	void InitMotor(); /// Initialize (stablish communication with the controller) and configure the robot
	void StopMotor();

	/// GetStatus:  Return the general (summary) status
	///    1  -- 1 if any axis in motion, MOTOR_MOVING_MASK
	///    0  -- 1 if any motor is off, MOTOR_OFF_MASK
	///    This list could be expanded.
	enum { MOTOR_OFF_MASK=0x01, MOTOR_MOVING_MASK=0x02 };
	unsigned short  GetStatus();
	bool ReadFootPedal();
};

#endif




	




