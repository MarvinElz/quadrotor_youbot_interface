#include "ros/ros.h"
#include "brics_actuator/JointPositions.h"
#include "geometry_msgs/Twist.h"

#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "quadrotor_control/kinematics.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistWithCovariance.h"

#include "youbot_arm_model/youbot_joints.h"
#include "youbot_arm_model/values_internet.h"
#include "youbot_arm_model/youbot_jacobi.h"

#include "quadrotor_control/kinematics.h"

#include <string>

#include <Eigen/Dense>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

double scale = 1.0;
double a1, a2, a3;
double d1, d5;
double k_Arm, k_Ell;

// Überprüft, ob von beiden Quellen Informationen vorliegen
struct synchronizer{
	bool IMU_ready;
	bool joint_ready;
	bool base_ready;
}typedef synchronizer;
synchronizer synch = {false, false, false};

ros::Publisher pub_joint;
ros::Publisher pub_base;
ros::Publisher pub_kin_measure;
// Message in der die gemessenen Kin-Größen gespeichert werden
quadrotor_control::kinematics kin_measure_msg;

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

	kin_measure_msg.vel.linear.x = msg->twist.twist.linear.x;
	kin_measure_msg.vel.linear.y = msg->twist.twist.linear.y;

	synch.base_ready = true;
	if( synch.IMU_ready && synch.joint_ready ){
		pub_kin_measure.publish( kin_measure_msg );
		synch.joint_ready = false;
		synch.IMU_ready 	= false;
		synch.base_ready 	= false;
	}	
}

/*
	Wird aufgerufen, wenn Winkelinformationen (des Youbots) aktualisiert werden
	Errechnet: Vx, Vy, Vz; PSI
*/
void callback_JointState( const sensor_msgs::JointState::Ptr& msg){
	MatrixXd Jacobi(6,5);
	KDL::JntArray JointVels(5);
	
	// ist hier eine Überprüfung msg->name notwendig, oder stimmt die Reihenfolge?
	for(int i = 0; i <= 4; i++)		
		JointVels(i) = msg->velocity[i];
	KDL::JntArray Joints(5);
	for(int i = 0; i <= 4; i++)		
		JointVels(i) = msg->position[i];

	getJacobi( Jacobi, Joints );

	VectorXd Vels(6);
	Vels = Jacobi * JointVels.data; // [0..2] v, [3..5] w
	
	// Vels skalieren
	Vels(0) /= scale;
	Vels(1) /= scale;
	Vels(2) /= scale;

	// PSI berechnen
	// RICHTIG???
	double psi = Joints(4) - Joints(0);	
	
	// berechnete Werte in kin_measure_msg eintragen	
	//kin_measure_msg.pose.position.x = 0;			// don't care
	//kin_measure_msg.pose.position.y = 0;			// don't care
	//kin_measure_msg.pose.position.z = 0;			// don't care
	//kin_measure_msg.pose.orientation.x = 0;		// don't care
	//kin_measure_msg.pose.orientation.y = 0;		// don't care
	kin_measure_msg.pose.orientation.z = psi;
	//kin_measure_msg.vel.linear.x = 0;				
	//kin_measure_msg.vel.linear.y = 0;
	kin_measure_msg.vel.linear.z = Vels(2);
	
	//	Berechnung von w auch möglich -> warum nicht?
	kin_measure_msg.vel.angular.x = Vels(3);
	kin_measure_msg.vel.angular.y = Vels(4);
	kin_measure_msg.vel.angular.z = Vels(5);


	synch.joint_ready = true;
	if( synch.IMU_ready && synch.base_ready ){
		pub_kin_measure.publish( kin_measure_msg );
		synch.joint_ready = false;
		synch.IMU_ready 	= false;
		synch.base_ready 	= false;
	}	
}

/*
	Wird aufgerufen, wenn IMU neue Daten liefert
*/
void callback_imu( const sensor_msgs::Imu::Ptr& msg){
	// TODO: w einlesen (evtl. umrechnen?)
	kin_measure_msg.vel.angular.x = msg->angular_velocity.x;
	kin_measure_msg.vel.angular.y = msg->angular_velocity.y;
	kin_measure_msg.vel.angular.z = msg->angular_velocity.z;
	
	synch.IMU_ready = true;
	if( synch.joint_ready && synch.base_ready ){
		pub_kin_measure.publish( kin_measure_msg );
		synch.joint_ready = false;
		synch.IMU_ready 	= false;
		synch.base_ready 	= false;
	}	
}

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
	double alpha = atan2( sin(msg->pose.position.x), -cos(msg->pose.position.x)*sin(msg->pose.position.y) );
	double gamma = asin( cos(msg->pose.position.y)*cos(msg->pose.position.x) );

	g[0] = gamma + M_PI*(k_Arm-1)/(2);
	g[1] = atan2(y4, x4) + k_Arm*k_Ell * b - M_PI/2;
	g[2] = k_Arm * k_Ell * (a-M_PI);
	g[3] = k_Arm * ( msg->pose.orientation.x ) -g[1] - g[2];
	g[4] = msg->pose.orientation.z - gamma + (k_Arm -1)/(2)*M_PI;
	
	ROS_INFO( "Gelenkwinkel: %f, %f, %f, %f, %f", g[0],g[1],g[2],g[3],g[4] );	
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
		msg_joints.positions.push_back(jv);
	}
        
	brics_actuator::Poison poison;
	poison.originator   = "";   // what?
	poison.description  = "";   // what?
	poison.qos          = 1.0f; // right?

	msg_joints.poisonStamp = poison;
	pub_joint.publish( msg_joints );
}



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
	
	// Subscriber für Gelenkwinkel-änderungen
	ros::Subscriber sub_joint = nh.subscribe("/joint_states", 10, callback_JointState);
	// Subscriber für IMU
	ros::Subscriber sub_imu   = nh.subscribe("/imu/data_raw", 10, callback_imu);
	// Subscriber für Odometriedaten der Basis
	ros::Subscriber sub_odom	= nh.subscribe("/odom", 10, callback_odom);
  

	pub_joint = nh.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command", 100);
  pub_base  = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
		
	ros::spin();
	return 0;
}
