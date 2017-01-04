#include "ros/ros.h"

#include "quadrotor_control/kinematics.h"

double scale = 1.0;
double a1, a2, a3;
double d1, d5;
double k_Arm, k_Ell;


void callback_kin_model( const quadrotor_control::kinematics::Ptr& msg ){
	// Scale: Vx, Vy, Vz
	msg->vel.linear.x *= scale;
	msg->vel.linear.y *= scale;
	msg->vel.linear.z *= scale;
	msg->pose.position.x *= scale;
	msg->pose.position.y *= scale;
	msg->pose.position.z *= scale;

	// inverse Kinematik
	double g[5];	// Gelenkwinkel
	double z = msg->pose.position.z;
	ROS_INFO( "Z: %f", z );
	//Hilfsgrößen
	double x4 = k_Arm*( - d5*cos( msg->pose.orientation.x ) ) - a1;
	//ROS_INFO( "x4: %f", x4 );
	double y4 = z - d1 - d5 * sin( msg->pose.orientation.x );
	//ROS_INFO( "y4: %f", y4 );
	double a = acos( (a2*a2 + a3*a3 - x4*x4 - y4*y4)/(2*a2*a3) );
	//ROS_INFO( "a: %f", a );
	double b = acos( (x4*x4 + y4*y4 + a2*a2 - a3*a3)/(2*a2*sqrt(x4*x4+y4*y4)) );
	//ROS_INFO( "b: %f", b );

	g[0] = M_PI*(k_Arm-1)/(2);
	g[1] = atan2(y4, x4) + k_Arm*k_Ell * b - M_PI/2;
	g[2] = k_Arm * k_Ell * (a-M_PI);
	g[3] = k_Arm * ( msg->pose.orientation.x ) -g[1] - g[2];
	g[4] = msg->pose.orientation.z + (k_Arm -1)/(2)*M_PI;
	
	ROS_INFO( "Gelenkwinkel: %f, %f, %f, %f, %f", g[0],g[1],g[2],g[3],g[4] );	
	ROS_INFO( "Winkelsumme: %f", g[0]+g[1]+g[2]+g[3] );	

	// TODO: Vx, Vy direkt an Plattform senden
	// TODO: Psi direkt an Gelenk 5 senden
	// TODO: (Phi/Theta) und Z verrechnen
	
}

// TODO: Skaliere Vx, Vy, Vz

// TODO: Skaliere Vx_Measure, Vy_Measure, Vz_Measure

// TODO: berechne Gelenkgeschwindigkeiten aus Phi/Theta, 

void getConstants(ros::NodeHandle &nh){
  	nh.getParam("scale", scale);
	ROS_INFO("SCALE: %f", scale);
	nh.getParam("a1", a1);
	nh.getParam("a2", a2);
	nh.getParam("a3", a3);
	nh.getParam("d1", d1);
	nh.getParam("d5", d5);
	nh.getParam("k_Arm", k_Arm);
	nh.getParam("k_Ell", k_Ell);
}

int main(int argc, char **argv)
{
 	
	ros::init(argc, argv, "Interface");
	ros::NodeHandle nh("Interface");	

	ROS_INFO("Start Interface");

   	getConstants(nh);
	
	// for testing
	ros::Subscriber sub_kin = nh.subscribe("/kin_measure", 10, callback_kin_model);
	//ros::Subscriber sub_kin = nh.subscribe("/kin_model", 10, callback_kin_model);
		
	ros::spin();
	return 0;
}
