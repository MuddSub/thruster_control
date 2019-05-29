/*

    THRUSTERS

 HFL        HFR
 VFL        VFR



 HBL        HBR
 VBL        VBR


*/

#include "thruster_control/ThrustMaster.h"

void Thrusters:::move() {

  //right-rotate
  double swayRotated = 0.7071 * (controlEffortSurge + controlEffortSway);
  double surgeRotated = -0.7071 * (controlEffortSway - controlEffortSurge);

  HFL = surgeRotated - controlEffortYaw;
  HFR = swayRotated + controlEffortYaw;
  HBL = swayRotated - controlEffortYaw;
  HBR = surgeRotated + controlEffortYaw;

  VFL = controlEffortHeave - controlEffortPitch + controlEffortRoll;
  VFR = controlEffortHeave - controlEffortPitch - controlEffortRoll;
  VBL = controlEffortHeave + controlEffortPitch + controlEffortRoll;
  VBR = controlEffortHeave + controlEffortPitch - controlEffortRoll;

  ROS_INFO("surgeRotated %f, swayRotated %f", surgeRotated, swayRotated);

  double max = 0.;
  for (double thruster : thrusters) if (thruster > max) max = thruster;
  for (double thruster : thrusters) thruster /= max;
  //ifnan
  for (double thruster : thrusters) if(thruster != thruster) thruster = 0;

}


void Thrusters::loadConfig(){
	std::vector<std::string> thrusters = {"HFL", "HFR", "HBL", "HBR", "VFL", "VFR", "VBL", "VBR"};
	
	std::vector<short> params;
	for(auto thruster : thrusters){
		params = nh_.getParam(thruster);
		thrusterConfig_[thruster] = params;
	}
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "ThrustMaster");
  nh_ = ros::NodeHandle();
  ros::Rate loopRate(10);

  // declare the publisher
  Thrusters::thrustPub = nh_.advertise<mission_control::Thrust>("thrusterValues", 1);

  Thrusters::surgeSub = nh_.subscribe("surgeControlEffort", 0, Thrusters::surgeCallback);
  Thrusters::swaySub = nh_.subscribe("swayControlEffort", 0, Thrusters::swayCallback);
  Thrusters::heaveSub = nh_.subscribe("heaveControlEffort", 0, Thrusters::heaveCallback);
  Thrusters::pitchSub = nh_.subscribe("pitchControlEffort", 0, Thrusters::pitchCallback);
  Thrusters::rollSub = nh_.subscribe("rollControlEffort", 0, Thrusters::rollCallback);
  Thrusters::yawSub = nh_.subscribe("yawControlEffort", 0, Thrusters::yawCallback);


  //set up PWM nonsense
  PCA9685 *pca9685 = new PCA9685() ;
  int err = pca9685->openPCA9685();
  if (err < 0){
    printf("Error: %d", pca9685->error);
	} else {
	  printf("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
	  pca9685->setAllPWM(0,0) ;
	  pca9685->reset();
	  pca9685->setPWMFrequency(60) ;

  while(ros::ok && pca9685->error >= 0) {
    Thrusters::move();

	#if DEBUG
    // pack the publisher
    mission_control::Thrust msg;
    msg.HFL = Thrusters::HFL;
    msg.HFR = Thrusters::HFR;
    msg.VFL = Thrusters::VFL;
    msg.VFR = Thrusters::VFR;
    msg.HBL = Thrusters::HBL;
    msg.HBR = Thrusters::HBR;
    msg.VBL = Thrusters::VBL;
    msg.VBR = Thrusters::VBR;

    // publish the values
    Thrusters::thrustPub.publish(msg);
    #endif
      
    //now actually do it
    std::vector<char> planes = {H, V};
    std::vector<char> frontBacks = {F, B};
    std::vector<char> sides = {L, R};

	for(auto plane : planes)
		for(auto frontBack : frontBacks)
			for(auto side : sides){
				std::string thruster = std::string() + plane + frontBack + side;
				std::vector<short> params = Thrusters::thrusterConfig_[thruster];
				pca9685->setPWM(params.at(0), 0, params.at(1) * 0;
			}

    ros::spinOnce();
    loopRate.sleep();
  }
  pca9685->closePCA9685();
}
