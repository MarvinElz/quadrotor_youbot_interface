#include "ros/ros.h"

float scale;

// TODO: Skaliere Vx, Vy, Vz

// TODO: Skaliere Vx_Measure, Vy_Measure, Vz_Measure

// TODO: berechne Gelenkgeschwindigkeiten aus Phi/Theta, 

int main(int argc, char **argv)
{
 	
	ros::init(argc, argv, "Interface");
	ros::NodeHandle nh("Interface");

	ROS_INFO("Start Interface");

	ros::spin();
	return 0;
}
