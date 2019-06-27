#ifndef THRUST_MASTER
#define THRUST_MASTER

#include <ros/ros.h>
#include <vector>
#include <std_msgs/Float64.h>
#include "thruster_control/Thrust.h"
#include "thruster_control/JHPWMPCA9685.h"
#include <string>
#include <unordered_map>
#include <iostream>
#include <ros/package.h>
#include <fstream>


class Thrusters{

public:

  Thrusters();
  ~Thrusters();
  ros::NodeHandle nh_;
  ros::NodeHandle nhPriv_;

  //i2c interface
  PCA9685* pca9685;
   //thruster config
  std::unordered_map<std::string, int> thrusterConfig_;



  void move();
  std::vector<std::string> thrusterNames_;


  int getPWM(double);
  int getPWM(std::string);
  void loadConfig();

  //naming: H/F [Horizontal/Vertical]; F/B [Front/Back]; R/L [Right/Left]
  double *HFL, *HFR, *HBL, *HBR, *VFL, *VFR, *VBL, *VBR;


  // declare the subscribers
  ros::Subscriber surgeSub, swaySub, heaveSub, pitchSub, rollSub, yawSub, enableSub;
  ros::Publisher thrustPub;



private:







  double controlEffortSurge, controlEffortSway, controlEffortHeave,
         controlEffortRoll, controlEffortPitch, controlEffortYaw;

	bool enable;
  double maxThrust;

  inline void surgeCallback(const std_msgs::Float64& msg){ controlEffortSurge = msg.data;}
  inline void swayCallback(const std_msgs::Float64& msg){ controlEffortSway = msg.data;}
  inline void heaveCallback(const std_msgs::Float64& msg){ controlEffortHeave = msg.data;}
  inline void pitchCallback(const std_msgs::Float64& msg){ controlEffortPitch = msg.data;}
  inline void rollCallback(const std_msgs::Float64& msg){ controlEffortRoll = msg.data;}
  inline void yawCallback(const std_msgs::Float64& msg){ controlEffortYaw = msg.data;}
	inline void enableCallback(const std_msgs::Bool& msg){enable = msg.data;}


  std::vector<double> thrusterVals_;









};









#endif
