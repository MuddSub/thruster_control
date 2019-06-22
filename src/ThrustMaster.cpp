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
  nhPriv_ = ros::NodeHandle("~");
  thrusterNames_  = {"HFL", "HFR", "HBL", "HBR", "VFL", "VFR", "VBL", "VBR"};
  thrusterVals_ = std::vector<double>(8);
  
  HFL = &thrusterVals_[0];
  HFR = &thrusterVals_[1];
  VFL = &thrusterVals_[2];
  VFR = &thrusterVals_[3];
  HBL = &thrusterVals_[4];
  HBR = &thrusterVals_[5];
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
  pca9685 = new PCA9685(0x53) ;
  
  int err = pca9685->openPCA9685();
  ROS_WARN("err %d", err);
  if (err < 0){
    printf("Error: %d", pca9685->error);
	} else {
	  printf("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
	  pca9685->setAllPWM(0,0);
	  pca9685->reset();
	  pca9685->setPWMFrequency(300);
  }
  
  for(int i = 1; i <= 8; ++i){
	  pca9685->setPWM(i, 1500);
  }
  ros::Duration(2).sleep();

  ROS_INFO("Thruster manager initialized");
}

Thrusters::~Thrusters(){

  for(int i = 1; i <= 8; ++i)
    this->pca9685->setPWM(i, 1500);

  pca9685->closePCA9685();
  
  //write to file
  std::ofstream configFile;
  std::string filePath = ros::package::getPath("thruster_control") + "/include/thruster_control/ThrusterConfig.yaml";
  configFile.open(filePath);

  configFile << "#Don't edit this file unless you're sure!" << std::endl;

  for(auto i : thrusterNames_){
    configFile << i << ": ";
    configFile << thrusterConfig_[i] << std::endl;
  }

  configFile.close();

  
}
void Thrusters::move() {

  //right-rotate
  double swayRotated = 0.7071 * (controlEffortSurge + controlEffortSway);
  double surgeRotated = -0.7071 * (controlEffortSway - controlEffortSurge);


  *HBR = surgeRotated + controlEffortYaw;
  *HFL = surgeRotated - controlEffortYaw;
  *HFR = swayRotated + controlEffortYaw;
  *HBL = swayRotated - controlEffortYaw;
  *VFL = controlEffortHeave - controlEffortPitch + controlEffortRoll;
  *VFR = controlEffortHeave - controlEffortPitch - controlEffortRoll;
  *VBL = controlEffortHeave + controlEffortPitch + controlEffortRoll;
  *VBR = controlEffortHeave + controlEffortPitch - controlEffortRoll;

  //ROS_INFO("surgeRotated %f, swayRotated %f", surgeRotated, swayRotated);

  double max = 0.;
  for (double thruster : thrusterVals_) if (thruster > max) max = thruster;
  for (double thruster : thrusterVals_) thruster /= max;
  //ifnan
  for (double thruster : thrusterVals_) if(thruster != thruster) thruster = 0;

}


void Thrusters::loadConfig(){
	
	int param = 0;
	for(std::vector<std::string>::iterator it = thrusterNames_.begin(); it != thrusterNames_.end(); ++it){
		
		ROS_INFO("Thruster %s", (*it).c_str());
		//params = [thruster number, sign] (fwd/bkwd)
		nh_.getParam((*it).c_str(), param);
	
		ROS_INFO("Params: %d", param);
		//if the thruster has already been included
		if(thrusterConfig_.find(*it) != thrusterConfig_.end()){
			ROS_WARN("Thruster %s has already been inserted", (*it).c_str());
			ROS_WARN("Please try again");
			--it;
			continue;
		}
	

		std::pair<std::string, int> configPair(*it, param);
		thrusterConfig_.insert(configPair);
                
              ROS_WARN("OUT %d", thrusterConfig_[*it]);
		
		
	}
}

int Thrusters::getPWM(double percentThrust){
	//scales the -1 -> 1 thruster value to 1100->1900
        float pwm =  (percentThrust * 400) + 1500;
        if(pwm > 1900) pwm = 1900;
	else if(pwm < 1100) pwm = 1100;
        return pwm;
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
  ROS_INFO("Thrust[HFL] %d", thrust->thrusterConfig_["HFL"]);


	for(auto thruster : thrust->thrusterNames_)
			  ROS_INFO("Thruster %d", thrust->thrusterConfig_[thruster]);



/*  
  bool notDone = true;
  while(notDone){
	  
	  
	  //port 0 stopped working
	  for(int i = 1; i <= 8; ++i){
		ROS_INFO("Running thruster %d", i);
		thrust->pca9685->setPWM(i, 1500);
		ros::Duration(2).sleep();	  
		thrust->pca9685->setPWM(i, 1700);
		ros::Duration(1).sleep();	  
		thrust->pca9685->setPWM(i, 1500);
	 
		ROS_INFO("Enter thruster, or \"r\" to run again");
		
		std::string inText;
		std::cin >> inText;
		ROS_WARN(inText.c_str());

		if(inText == "r"){
			--i;
			continue;
		}
		else if(thrust->thrusterConfig_.find(inText) != thrust->thrusterConfig_.end()){
			thrust->thrusterConfig_[inText] = i;
		}
		else{
			ROS_INFO("Unable to find provided thruster. Please try again");
			--i;
			continue;
		}


	  }
	  
	  
	  ROS_INFO("Running it back");
	  
	  
	  for(auto thruster : thrust->thrusterNames_){
		  ROS_INFO("Thruster %d", thrust->thrusterConfig_[thruster]);
		  thrust->pca9685->setPWM(thrust->thrusterConfig_[thruster], 1500);
     	  ros::Duration(.5).sleep();	  
		  thrust->pca9685->setPWM(thrust->thrusterConfig_[thruster],  1700);
		  ros::Duration(.5).sleep();
		  thrust->pca9685->setPWM(thrust->thrusterConfig_[thruster], 1500);
	  }
	  
	  ROS_INFO("Was this right? Enter 1 to repeat");
	  std::cin >> notDone;
	  
	}
  
  
  delete thrust;
  return 0;
*/
  int debugNow = 0;

  while(ros::ok && thrust->pca9685->error >= 0) {

    thrust->move();

  
    // pack the publisher
    thruster_control::Thrust msg;
  
 
    msg.HFL = *(thrust->HFL);
    msg.HFR = *(thrust->HFR);
    msg.VFL = *(thrust->VFL);
    msg.VFR = *(thrust->VFR);
    msg.HBL = *(thrust->HBL);
    msg.HBR = *(thrust->HBR);
    msg.VBL = *(thrust->VBL);
    msg.VBR = *(thrust->VBR);
    
    // publish the values
    thrust->thrustPub.publish(msg);
      

    for(auto thruster : {"HFL", "HFR", "HBL", "HBR", "VBL", "VBR", "VFL", "VFR"}){
	int thrusterNum = thrust->thrusterConfig_[thruster];
	thrust->pca9685->setPWM(thrusterNum, thrust->getPWM(thruster));
        if(debugNow % 1000 == 0)
           ROS_INFO("thrust num %d, pwm %d", thrusterNum, thrust->getPWM(thruster));  
  }
  
   debugNow++;

    ros::spinOnce();
    loopRate.sleep();
  }
 
}
