#include "ros/ros.h"

float scale = 1.0;

// TODO: Skaliere Vx, Vy, Vz

// TODO: Skaliere Vx_Measure, Vy_Measure, Vz_Measure

// TODO: berechne Gelenkgeschwindigkeiten aus Phi/Theta, 

void getConstants(ros::NodeHandle &nh){
  	nh.getParam("scale", scale);
	ROS_INFO("SCALE: %f", scale);
}

int main(int argc, char **argv)
{
 	
	ros::init(argc, argv, "Interface");
	ros::NodeHandle nh("Interface");	

	ROS_INFO("Start Interface");

   	getConstants(nh);
	

	ros::spin();
	return 0;
}
