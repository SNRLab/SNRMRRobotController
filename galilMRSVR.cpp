#include "galilMRSVR.h"
#include <math.h>
#include <iostream>
#include <sstream> 
#include <cstdio>
#include <time.h>

using namespace std;

inline double dtor(double deg){return deg*3.141592/180.0;}
inline string dtostr(double value){
  std::ostringstream strs;
  strs << value;
  std::string str = strs.str();
  return str;}

// Consturctor 

galilMRSVR::galilMRSVR(const string& address)
{
  g = new Galil(address);
  speed = 0;
  accel = 0;
  decel = 0;
  isInit = false;
  homed = false;
  time_threshold = 60; // 1min time threshold for motion complete

  for (int i=0; i < MAX_AXES; i++) {
    cnts_per_mm[i] = 1000.0;
    axis_home[i] = 0.0;
    axis_ulim[i] = 0.0;
    axis_llim[i] = 0.0;
    pid_kp[i] = 6;
    pid_ki[i] = 1;
    pid_kd[i] = 64;
    curlim_avg[i] = 9.99;
    deadBandOff[i] = 5; //US motor specific parameter
    deadBandOn[i] = 100; //US motor specific parameter
    antiFricPos[i] = 0.28; //US motor specific parameter
    antiFricNeg[i] = 0.28; //US motor specific parameter
  }
}

galilMRSVR::~galilMRSVR()
{
    DisableMotorPower();
    delete g;
}
double galilMRSVR::EnableMotorPower()
{
  double mon = g->commandValue("SH");
  cout<<"motor on return value:::"<<mon<<endl;
  g->command("TM500");
}

double galilMRSVR::DisableMotorPower()
{
  double moff = g->commandValue("MO");
}

bool galilMRSVR::Home()
{
  //avoid homing robot if it is already homed - TODO

  //Make sure robot is unhomed before homing it
  //UnHome();

  //set a small speed and accel/decel for homing
  // SetSpeed(5.0);
  // SetAccel(10.0);
  // SetDecel(10.0);

  return 1;

}
void galilMRSVR::MoveAxes(double x, double y) const
{
  double mx = x*cnts_per_mm[X];
  double my = y*cnts_per_mm[Y];
  string cmd = "PA "+dtostr(mx);//+dtostr(my);
  g->command(cmd);
  g->command("BGA");
}
void galilMRSVR::MoveAxesInct(double x, double y)const
{
  double ix = x*cnts_per_mm[X];
  double iy = y*cnts_per_mm[Y];
  string cmd = "IP "+dtostr(ix); //+dtostr(iy)
  g->command(cmd);
}
void galilMRSVR::SetSpeed(double spd) const
{
  if (spd< 0.0)
    cerr<<"Speed is less than zero"<<endl;
  double mspd = spd*cnts_per_mm[X];
  string cmd = "SP"+dtostr(mspd);
  g->command(cmd);
}
void galilMRSVR::SetAccel(double x) const
{
  double mx = x*cnts_per_mm[X];
  string cmd = "AC"+dtostr(x)+dtostr(mx);
  g->command(cmd);
}
void galilMRSVR::SetDecel(double x) const
{
  double mx = x*cnts_per_mm[X];
  string cmd = "DC"+dtostr(x)+dtostr(mx);
  g->command(cmd);
}
double galilMRSVR::GetSpeed() const
{
  string str = "SP ?";
  double speed = g->commandValue(str);
  return speed;
}
double galilMRSVR::GetAccel() const
{
  string str = "AC ?";
  double accel = g->commandValue(str);
  return accel;
}
double galilMRSVR::GetDecel() const
{
  string str = "DC ?";
  double decel = g->commandValue(str);
  return decel;
}
double galilMRSVR::GetPositionX() const
{
  string str = "TPA";
  double posX = g->commandValue(str);
  return posX/cnts_per_mm[X];
}
double galilMRSVR::GetPositionY() const
{
  string str = "TPB";
  double posY = g->commandValue(str);
  return posY/cnts_per_mm[Y];
}
void galilMRSVR::WaitMotion()
{
  clock_t init, eTime;
  init = clock();

  //polling for _BGn : contains zero when motion is completed.
  while (true)
    {
      eTime = clock()-init;
      double elapsedTime = double(eTime)/((double)CLOCKS_PER_SEC);
      if (!g->commandValue("MG _BGX"))
	{
	  //Motion completed
	  break;
	}
      else if(elapsedTime > time_threshold)
	{
	  //time passed
	  cout<<"motion did not complete in:"<<elapsedTime<<"seconds"<<endl;
	  StopMotor();
	  break;
	}
    }
}

void galilMRSVR::InitMotor()
{
  // This is initialization for ultrasound motor
  string cmd = "KP"+dtostr(pid_kp[X])+","+dtostr(pid_kp[Y]);
  cmd += ";KD"+dtostr(pid_kd[X])+","+dtostr(pid_kd[Y]);
  cmd += ";KI"+dtostr(pid_ki[X])+","+dtostr(pid_ki[Y]);
  cmd += ";DS"+dtostr(deadBandOff[X])+","+dtostr(deadBandOff[Y]);
  cmd += ";DB"+dtostr(deadBandOn[X])+","+dtostr(deadBandOn[Y]);
  cmd += ";ZN"+dtostr(antiFricNeg[X])+","+dtostr(antiFricNeg[Y]);
  cmd += ";ZP"+dtostr(antiFricPos[X])+","+dtostr(antiFricPos[Y]);
  g->command(cmd);
}

void galilMRSVR::StopMotor()
{
  string cmd = "ST";
  g->command(cmd);
}

unsigned short galilMRSVR::GetStatus()
{
  unsigned short status = 0;
  int sw[MAX_AXES]={0,0};
  string rtnstr = g->command("TS");
  char* rtn=(char*) rtnstr.c_str();

  if (sscanf(rtn,"%d,%d", &sw[0], &sw[1]) != 2){
    cout<<"GetStatus: error scanning TS:"<<rtn<<endl;
  }

  //Axis is in motion if bit 7 is high.
  //Axis motor is off if bit 5 is high.
  for (int i=0; i<MAX_AXES; i++){
    if ( sw[i]&0x0080 )
      status |= MOTOR_MOVING_MASK;
    if ( sw[i]&0x0020 )
      status |= MOTOR_OFF_MASK;
  }
  return status;
}

bool galilMRSVR::ReadFootPedal()
{
  bool state = g->commandValue("MG@IN[1]");
  return state;
}
