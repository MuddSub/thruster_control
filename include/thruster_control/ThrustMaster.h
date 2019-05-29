#ifndef THRUST_MASTER
#define THRUST_MASTER

#include <ros/ros.h>
#include <vector>
#include <std_msgs/Float64.h>
#include "thruster_control/Thrust.h"
#include "thruster_control/JHPWMPCA9685.h"
#include <string>
#include <unordered_map>


namespace Thrusters{

  ros::nodeHandle nh_;

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



  std::vector<double> thrusters(8);


  //naming: H/F [Horizontal/Vertical]; F/B [Front/Back]; R/L [Right/Left]
  double& HFL = thrusters[0];
  double& HFR = thrusters[1];
  double& VFL = thrusters[2];
  double& VFR = thrusters[3];
  double& HBL = thrusters[4];
  double& HBR = thrusters[5];
  double& VBL = thrusters[6];
  double& VBR = thrusters[7];
  
  
  //thruster config
  std::unordered_map<std::string, std::vector<short>> thrusterConfig_;
  
  
  void move();
  void loadConfig();
  
  
  
  
}









#endif
