#include "ros/ros.h"
#include "brics_actuator/JointPositions.h"
#include "geometry_msgs/Twist.h"

#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "quadrotor_control/kinematics.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistWithCovariance.h"

#include "youbot_arm_model/youbot_joints.h"
#include "youbot_arm_model/values_urdf.h"
#include "youbot_arm_model/youbot_jacobi.h"

#include "quadrotor_control/kinematics.h"

#include <string>

#include <Eigen/Dense>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

double scaleV = 1.0;
double scaleR = 0.2;
double a1, a2, a3;
double d1, d5;
double k_Arm, k_Ell;

const double my_joint_offsets[5] = {2.9496, 1.13446, -2.635447, 1.98896, 2.87979};

// Überprüft, ob von beiden Quellen Informationen vorliegen
struct synchronizer{
	bool IMU_ready;
	bool joint_ready;
	bool base_ready;
}typedef synchronizer;
synchronizer synch = {true, false, false};

ros::Publisher pub_joint;
ros::Publisher pub_base;
ros::Publisher pub_kin_measure;
// Message in der die gemessenen Kin-Größen gespeichert werden
quadrotor_control::kinematics kin_measure_msg;

// Zum Testen der callback_JointState Funktion mittels Gazebo (Plattform muss überbrückt werden)
double VX = 0;
double VY = 0;
double VPhi = 0;
double VTheta = 0;

/*
	Wird aufgerufen, wenn Odometriedaten (des Youbots) aktualisiert werden
*/
void callback_odom( const nav_msgs::Odometry::Ptr& msg){
	
	/*
		Header header
		string child_frame_id
		geometry_msgs/PoseWithCovariance pose
		geometry_msgs/TwistWithCovariance twist
	*/

	kin_measure_msg.vel.linear.x = msg->twist.twist.linear.x / scaleV;
	kin_measure_msg.vel.linear.y = msg->twist.twist.linear.y / scaleV;

	synch.base_ready = true;
	if( synch.IMU_ready && synch.joint_ready ){
		pub_kin_measure.publish( kin_measure_msg );
		synch.joint_ready = false;
		synch.IMU_ready 	= true;
		synch.base_ready 	= false;
	}	
}

/*
	Wird aufgerufen, wenn Winkelinformationen (des Youbots) aktualisiert werden
	Errechnet: Vx, Vy, Vz; PSI
*/
void callback_JointState( const sensor_msgs::JointState::Ptr& msg){
	MatrixXd Jacobi(6,5);
	
	
	// ist hier eine Überprüfung msg->name notwendig, oder stimmt die Reihenfolge?
	KDL::JntArray JointVels(5);
	for(int i = 0; i <= 4; i++)		
		JointVels(i) = msg->velocity[i];

	KDL::JntArray Joints(5);
	for(int i = 0; i <= 4; i++)		
		Joints(i) = msg->position[i] - joint_offsets[i];

	getJacobi( Jacobi, Joints );

	VectorXd Vels(6);
	Vels = Jacobi * JointVels.data; // [0..2] v, [3..5] w
	
	// Vels skalieren
	Vels(0) /= scaleV;
	Vels(1) /= scaleV;
	Vels(2) /= scaleV;

	// PSI berechnen
	// RICHTIG???
	double psi = Joints(0);	
	
	// berechnete Werte in kin_measure_msg eintragen	
	//kin_measure_msg.pose.position.x = 0;			// don't care
	//kin_measure_msg.pose.position.y = 0;			// don't care
	//kin_measure_msg.pose.position.z = 0;			// don't care
	//kin_measure_msg.pose.orientation.x = 0;		// don't care
	//kin_measure_msg.pose.orientation.y = 0;		// don't care
	kin_measure_msg.pose.orientation.z = psi;
	//kin_measure_msg.vel.linear.x = 0;				
	//kin_measure_msg.vel.linear.y = 0;

	kin_measure_msg.vel.linear.x = VX;	// NUR ZUM TESTEN VON CALLBACK_JOINTSTATE			
	kin_measure_msg.vel.linear.y = VY;

	kin_measure_msg.vel.linear.z = - Vels(2);
	
	//	Berechnung von w auch möglich -> warum nicht?
	kin_measure_msg.vel.angular.x = VPhi;
	kin_measure_msg.vel.angular.y = - Vels(4) /scaleR;
	kin_measure_msg.vel.angular.z = - Vels(5);

	ROS_INFO( "Measure:" );
	ROS_INFO("PSI: % 06.4f, Vz: % 06.4f, VPhi: % 06.4f, VTheta: % 06.4f, VPsi: % 06.4f", 
		psi, 
		kin_measure_msg.vel.linear.z, 
		kin_measure_msg.vel.angular.x, 
		kin_measure_msg.vel.angular.y,
		kin_measure_msg.vel.angular.z);

	synch.joint_ready = true;
	if( synch.IMU_ready && synch.base_ready ){
		pub_kin_measure.publish( kin_measure_msg );
		synch.joint_ready = false;
		synch.IMU_ready 	= true;
		synch.base_ready 	= false;
	}	
}

/*
	Wird aufgerufen, wenn IMU neue Daten liefert
*/
void callback_imu( const sensor_msgs::Imu::Ptr& msg){
	// TODO: w einlesen (evtl. umrechnen?)
	kin_measure_msg.vel.angular.x = msg->angular_velocity.x / scaleR;
	kin_measure_msg.vel.angular.y = msg->angular_velocity.y / scaleR;
	kin_measure_msg.vel.angular.z = msg->angular_velocity.z;
	
	synch.IMU_ready = true;
	if( synch.joint_ready && synch.base_ready ){
		pub_kin_measure.publish( kin_measure_msg );
		synch.joint_ready = false;
		synch.IMU_ready 	= true;
		synch.base_ready 	= false;
	}	
}

void callback_kin_model( const quadrotor_control::kinematics::Ptr& msg ){
	VX = msg->vel.linear.x;
	VY = msg->vel.linear.y;
	VPhi = msg->vel.angular.x;

	// Scale: Vx, Vy, Vz
	msg->vel.linear.x *= scaleV;
	msg->vel.linear.y *= scaleV;
	msg->vel.linear.z *= scaleV;
	msg->pose.position.x *= scaleV;
	msg->pose.position.y *= scaleV;
	msg->pose.position.z *= scaleV;

	// Scale: Phi, Theta
	msg->pose.orientation.x *= scaleR;
	msg->pose.orientation.y *= scaleR;

	// inverse Kinematik
	// Aufteilung in X- und Y-Bewegung

	double g[5];	// Gelenkwinkel
	double z = - msg->pose.position.z;		// Achtung: Negation!
	//ROS_INFO( "Z: %f", z );
  if( z < 0.5 ){
		ROS_INFO("Unsichere Arbeitshoehe -> Bitte nach oben bewegen");
		return;
	}

	double psi = msg->pose.orientation.z;	// Rotation um Z-Achse des youBots
	//ROS_INFO("psi: %f", psi);

	double phi = M_PI/2 - msg->pose.orientation.y;	// Rotation um Y-Achse des UAV
	//ROS_INFO("phi: %f", phi);

	//Hilfsgrößen
	double x4 = k_Arm*( - d5*cos( phi ) ) - a1;
	//ROS_INFO( "x4: %f", x4 );
	double y4 = z - d1 - d5 * sin( phi );
	//ROS_INFO( "y4: %f", y4 );
	double a = acos( (a2*a2 + a3*a3 - x4*x4 - y4*y4)/(2*a2*a3) );
	//ROS_INFO( "a: %f", a );
	double b = acos( (x4*x4 + y4*y4 + a2*a2 - a3*a3)/(2*a2*sqrt(x4*x4+y4*y4)) );
	//ROS_INFO( "b: %f", b );
	
	g[0] = psi + M_PI*(k_Arm-1)/(2);
	g[1] = atan2(y4, x4) + k_Arm*k_Ell * b - M_PI/2;
	g[2] = k_Arm * k_Ell * (a-M_PI);
	g[3] = k_Arm * ( phi - M_PI/2 ) - g[1] - g[2];
	g[4] = 0; //msg->pose.orientation.z - psi + (k_Arm -1)/(2)*M_PI;
	
	// joint_offsets addieren
	for( int i = 0; i < 5; i++ )
		g[i] += joint_offsets[i];	

	//ROS_INFO( "Winkelsumme: %f", g[0]+g[1]+g[2]+g[3] );	

	brics_actuator::JointPositions msg_joints;
	geometry_msgs::Twist msg_base;

	msg_base.linear.x = msg->vel.linear.x;
	msg_base.linear.y = msg->vel.linear.y;
	msg_base.linear.z = 0.0f;

	msg_base.angular.x = 0.0f;
	msg_base.angular.y = 0.0f;
	msg_base.angular.z = 0.0f;
	pub_base.publish(msg_base);    

	ros::Time time_now = ros::Time::now();
	for( int i = 0; i <= 4; i++ ){
		brics_actuator::JointValue jv;
		jv.timeStamp = time_now;        
		jv.joint_uri = joint_names[i];
		jv.unit = "rad";
		jv.value = g[i];
		if( g[i] != g[i] ) // NaN
			return;
		msg_joints.positions.push_back(jv);
	}
        
	//ROS_INFO( "Gelenkwinkel: %f, %f, %f, %f, %f", g[0],g[1],g[2],g[3],g[4] );	

	brics_actuator::Poison poison;
	poison.originator   = "";   // what?
	poison.description  = "";   // what?
	poison.qos          = 1.0f; // right?

	msg_joints.poisonStamp = poison;
	pub_joint.publish( msg_joints );
}



void getConstants(ros::NodeHandle &nh){
	nh.getParam("scaleV", scaleV);
	ROS_INFO("SCALE V: %f", scaleV);
	nh.getParam("scaleR", scaleR);
	ROS_INFO("SCALE R: %f", scaleR);
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

	
	
	// Subscriber für Gelenkwinkel-änderungen
	ros::Subscriber sub_joint = nh.subscribe("/joint_states", 10, callback_JointState);
	// Subscriber für IMU
	ros::Subscriber sub_imu   = nh.subscribe("/imu/data_raw", 10, callback_imu);
	// Subscriber für Odometriedaten der Basis
	ros::Subscriber sub_odom	= nh.subscribe("/odom", 10, callback_odom);
  
	//pub_kin_measure = nh.advertise<quadrotor_control::kinematics>("/kin_measure", 10);
	pub_joint = nh.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command", 100);
  pub_base  = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
		
	ros::spin();
	return 0;
}
