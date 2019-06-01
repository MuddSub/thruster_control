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

class Thrusters{
	
public:

  Thrusters();
  ros::NodeHandle nh_;

  //i2c interface
  PCA9685* pca9685;
   //thruster config
  std::unordered_map<std::string, std::vector<short>> thrusterConfig_;
  
  
  
  void move();
  std::vector<std::string> thrusterNames_;
  
 
  int getPWM(double);
  int getPWM(std::string);

  

private:




  // declare the subscribers
  ros::Subscriber surgeSub, swaySub, heaveSub, pitchSub, rollSub, yawSub;
  ros::Publisher thrustPub;
  
  
  
  double controlEffortSurge, controlEffortSway, controlEffortHeave,
         controlEffortRoll, controlEffortPitch, controlEffortYaw;

  void surgeCallback(const std_msgs::Float64& msg){ controlEffortSurge = msg.data;}
  void swayCallback(const std_msgs::Float64& msg){ controlEffortSway = msg.data;}
  void heaveCallback(const std_msgs::Float64& msg){ controlEffortHeave = msg.data;}
  void pitchCallback(const std_msgs::Float64& msg){ controlEffortPitch = msg.data;}
  void rollCallback(const std_msgs::Float64& msg){ controlEffortRoll = msg.data;}
  void yawCallback(const std_msgs::Float64& msg){ controlEffortYaw = msg.data;}



  std::vector<double> thrusterVals_;


  //naming: H/F [Horizontal/Vertical]; F/B [Front/Back]; R/L [Right/Left]
  double* HFL;
  double* HFR;
  double* VFL;
  double* VFR;
  double* HBL;
  double* HBR;
  double* VBL;
  double* VBR;
  

  
  void loadConfig();

  
  
};









#endif
