#ifndef THRUST_MASTER
#define THRUST_MASTER

#include <ros/ros.h>
#include <vector>
#include <std_msgs/Float64.h>
#include "thruster_control/Thrust.h"
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

   //thruster config
  std::unordered_map<std::string, int> thrusterConfig_;



  void move();
  std::vector<std::string> thrusterNames_;


  int getPWM(double);
  int getPWM(std::string);
  void loadConfig();

  //naming: H/F [Horizontal/Vertical]; F/B [Front/Back]; R/L [Right/Left]
  double* HFL;
  double* HFR;
  double* VFL;
  double* VFR;
  double* HBL;
  double* HBR;
  double* VBL;
  double* VBR;


  // declare the subscribers
  ros::Subscriber surgeSub, swaySub, heaveSub, pitchSub, rollSub, yawSub;
  ros::Publisher thrustPub;



private:







  double controlEffortSurge, controlEffortSway, controlEffortHeave,
         controlEffortRoll, controlEffortPitch, controlEffortYaw;

  void surgeCallback(const std_msgs::Float64& msg){ controlEffortSurge = msg.data;}
  void swayCallback(const std_msgs::Float64& msg){ controlEffortSway = msg.data;}
  void heaveCallback(const std_msgs::Float64& msg){ controlEffortHeave = msg.data;}
  void pitchCallback(const std_msgs::Float64& msg){ controlEffortPitch = msg.data;}
  void rollCallback(const std_msgs::Float64& msg){ controlEffortRoll = msg.data;}
  void yawCallback(const std_msgs::Float64& msg){ controlEffortYaw = msg.data;}



  std::vector<double> thrusterVals_;









};









#endif
