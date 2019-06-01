/*

    THRUSTERS

 HFL        HFR
 VFL        VFR



 HBL        HBR
 VBL        VBR


*/

#include "thruster_control/ThrustMaster.hpp"

Thrusters::Thrusters(){
  nh_ = ros::NodeHandle();

  thrusterNames_  = {"HFL", "HFR", "HBL", "HBR", "VFL", "VFR", "VBL", "VBR"};
  thrusterVals_ = std::vector<double>(8);
  
  HFL = &thrusterVals_[0];
  HFR = &thrusterVals_[1];
  VFL = &thrusterVals_[2];
  VFR = &thrusterVals_[3];
  HBL = &thrusterVals_[4];
  VBR = &thrusterVals_[5];
  VBL = &thrusterVals_[6];
  VBR = &thrusterVals_[7];
  
	  // declare the publisher
  thrustPub = nh_.advertise<thruster_control::Thrust>("thrusterValues", 1);

  surgeSub = nh_.subscribe("surgeControlEffort", 0, &Thrusters::surgeCallback, this);
  swaySub = nh_.subscribe("swayControlEffort", 0, &Thrusters::swayCallback, this);
  heaveSub = nh_.subscribe("heaveControlEffort", 0, &Thrusters::heaveCallback, this);
  pitchSub = nh_.subscribe("pitchControlEffort", 0, &Thrusters::pitchCallback, this);
  rollSub = nh_.subscribe("rollControlEffort", 0, &Thrusters::rollCallback, this);
  Thrusters::yawSub = nh_.subscribe("yawControlEffort", 0, &Thrusters::yawCallback, this);


  //set up PWM nonsense
  pca9685 = new PCA9685() ;
  
  int err = pca9685->openPCA9685();
  if (err < 0){
    printf("Error: %d", pca9685->error);
	} else {
	  printf("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
	  pca9685->setAllPWM(0,0);
	  pca9685->reset();
	  pca9685->setPWMFrequency(60);
  }
}

Thrusters::~Thrusters(){
  pca9685->closePCA9685();
  
  //write to file
  std::ofstream configFile;
  std::string filePath = ros::package::getPath("thruster_control") + "/include/thruster_control/ThrusterConfig.yaml";
  configFile.open(filePath);

  configFile << "#Don't edit this file unless you're sure!" << std::endl;

  for(auto i : thrusterNames_){
    configFile << i << ":";
    configFile << "[" << thrusterConfig_[i][0] << ", ";
    configFile << thrusterConfig_[i][1] << "] \n";
  }

  configFile.close();

  
}
void Thrusters::move() {

  //right-rotate
  double swayRotated = 0.7071 * (controlEffortSurge + controlEffortSway);
  double surgeRotated = -0.7071 * (controlEffortSway - controlEffortSurge);

  *HFL = surgeRotated - controlEffortYaw;
  *HFR = swayRotated + controlEffortYaw;
  *HBL = swayRotated - controlEffortYaw;
  *HBR = surgeRotated + controlEffortYaw;

  *VFL = controlEffortHeave - controlEffortPitch + controlEffortRoll;
  *VFR = controlEffortHeave - controlEffortPitch - controlEffortRoll;
  *VBL = controlEffortHeave + controlEffortPitch + controlEffortRoll;
  *VBR = controlEffortHeave + controlEffortPitch - controlEffortRoll;

  ROS_INFO("surgeRotated %f, swayRotated %f", surgeRotated, swayRotated);

  double max = 0.;
  for (double thruster : thrusterVals_) if (thruster > max) max = thruster;
  for (double thruster : thrusterVals_) thruster /= max;
  //ifnan
  for (double thruster : thrusterVals_) if(thruster != thruster) thruster = 0;

}


void Thrusters::loadConfig(){
	
	std::vector<double> params;
	std::vector<short> paramsShort;
	for(std::vector<std::string>::iterator it = thrusterNames_.begin(); it != thrusterNames_.end(); ++it){
		//params = [thruster number, sign (fwd/bkwd)
		nh_.getParam(*it, params);
	
		//if the thruster has already been included
		if(thrusterConfig_.find(*it) != thrusterConfig_.end()){
			ROS_WARN("Thruster %s has already been inserted", (*it).c_str());
			ROS_WARN("Please try again");
			--it;
			continue;
		}
	
		paramsShort.push_back((short)params[0]);
		paramsShort.push_back((short)params[1]);
		
		
		std::pair<std::string, std::vector<short>> configPair(*it, paramsShort);
		thrusterConfig_.insert(configPair);
		
		
	}
}

int Thrusters::getPWM(double percentThrust){
	//scales the -1 -> 1 thruster value to 1100->1900
	return (percentThrust * 400) + 1500;	
}

int Thrusters::getPWM(std::string thruster){
	//scales the -1 -> 1 thruster value to 1100->1900
	if(thruster == "HFL") return getPWM(*HFL);
	if(thruster == "HFR") return getPWM(*HFR);
	if(thruster == "HBL") return getPWM(*HBL);
	if(thruster == "HBR") return getPWM(*HBR);
	if(thruster == "VFL") return getPWM(*VFL);
	if(thruster == "VFR") return getPWM(*VFR);
	if(thruster == "VBL") return getPWM(*VBL);
	if(thruster == "VBR") return getPWM(*VBR);			
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "ThrustMaster");
  
  Thrusters* thrust = new Thrusters();

  ros::Rate loopRate(10);
  
  thrust->loadConfig();

  #if THRUSTER_CONFIG
  
  for(int i = 0; i < 8; ++i){
	ROS_INFO("Running thruster %d", i);
	thrust->pca9685->setPWM(i, 0, 1600);
	ros::Duration(0.5).sleep();	  
  	thrust->pca9685->setPWM(i, 0, 0);
	ROS_INFO("Enter thruster, or \"r\" to run again");
	
	std::string inText;
	std::cin >> inText;
	ROS_WARN(inText.c_str());

	if(inText == "r"){
		--i;
		continue;
	}
	else if(thrust->thrusterConfig_.find(inText) != thrust->thrusterConfig_.end()){
		thrust->thrusterConfig_[inText].at(0) = i;
	}
	else{
		ROS_INFO("Unable to find provided thruster. Please try again");
		--i;
		continue;
	}
	ROS_INFO("Which way did the thruster spin? 1 = fwd, -1 = bkwd");
	std::cin >> thrust->thrusterConfig_[inText].at(1);

  }
  delete thrust;
  return 0;
  
  #endif

  while(ros::ok && thrust->pca9685->error >= 0) {
    thrust->move();

	#if DEBUG_THRUST
    // pack the publisher
    thruster_control::Thrust msg;
    msg.HFL = *thrust->HFL;
    msg.HFR = *thrust->HFR;
    msg.VFL = *thrust->VFL;
    msg.VFR = *thrust->VFR;
    msg.HBL = *thrust->HBL;
    msg.HBR = *thrust->HBR;
    msg.VBL = *thrust->VBL;
    msg.VBR = *thrust->VBR;

    // publish the values
    thrust->thrustPub.publish(msg);
    #endif
      

	for(auto thruster : thrust->thrusterNames_){
		std::vector<short> params = thrust->thrusterConfig_[thruster];
		//write to i2c. says write to the correct thruster number (given
		//by params.at(0) with an off-value of 0, and an on value of
		//the mapped PWM value multiplied by params.at(1) which is +/- 1
		thrust->pca9685->setPWM(params.at(0), 0, params.at(1) * thrust->getPWM(thruster));
	}

    ros::spinOnce();
    loopRate.sleep();
  }
  
}
