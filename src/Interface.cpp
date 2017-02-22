#include "ros/ros.h"
#include "brics_actuator/JointPositions.h"
#include "geometry_msgs/Twist.h"

#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Vector3Stamped.h"	// imu_Complementary_filter
#include "quadrotor_control/kinematics.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistWithCovariance.h"

#include "youbot_arm_model/youbot_joints.h"
#include "youbot_arm_model/values_urdf.h"
#include "youbot_arm_model/youbot_jacobi.h"

#include "quadrotor_control/kinematics.h"

#include <string>
#include <cstring>

#include <Eigen/Dense>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

#define X_ONLY true
#define Y_ONLY false

// Hier Einstellung, welche Bewegungsrichtung simuliert werden soll
#define MOV_ONLY X_ONLY

// Skalierung der Translationsgeschwindigkeit & Raum
double scaleT = 0.15;
// Skalierung der Winkelgeschwindigkeit
double scaleR = 0.2;

// Roboterarm-Parameter
double a1, a2, a3;
double d1, d5;
double k_Arm, k_Ell;


// Überprüft, ob von allen Quellen Informationen vorliegen
struct synchronizer{
	bool IMU_ready;
	bool joint_ready;
	bool base_ready;
}typedef synchronizer;
synchronizer synch = {false, false, false};

// Publisher für die Gelenkwinkel
ros::Publisher pub_joint;
// Publisher für die Geschwindigkeit der Plattform
ros::Publisher pub_base;

// Publisher für die gemessenen Kinematikdaten -> Regelung
ros::Publisher pub_kin_measure;

// Message in der die gemessenen Kin-Größen gespeichert werden
quadrotor_control::kinematics kin_measure_msg;

// für den Fall, dass der youBot die Pose nicht darstellen kann,
// oder die Pose unsicher wird (Phi, Theta > 20°) werden die
// kin_model-Daten direkt an die Regelung zurückgegeben
quadrotor_control::kinematics kin_model_save_msg;

bool safe = false;


// ----------------------------------------------------------------------

/*
	Wird aufgerufen, wenn Odometriedaten (des Youbots) aktualisiert werden
*/
void callback_odom( const nav_msgs::Odometry::Ptr& msg){
	
	if( safe == false ){
		//pub_kin_measure.publish( kin_model_save_msg );
		return;
	}

	/*
		Header header
		string child_frame_id
		geometry_msgs/PoseWithCovariance pose
		geometry_msgs/TwistWithCovariance twist
	*/

	// Transformation von U -> N + Skalierung
	kin_measure_msg.vel.linear.x = 				msg->twist.twist.linear.x / scaleT;
	kin_measure_msg.vel.linear.y = (-1) * msg->twist.twist.linear.x / scaleT;
	
	ROS_INFO("Measure: VX: % 06.4f, VY: % 06.4f", 
		kin_measure_msg.vel.linear.x, 
		kin_measure_msg.vel.linear.y);

	synch.base_ready = true;
	if( synch.IMU_ready && synch.joint_ready ){
		//pub_kin_measure.publish( kin_measure_msg );
		synch.joint_ready = false;
		synch.IMU_ready 	= false;
		synch.base_ready 	= false;
	}	
}

/*
	Wird aufgerufen, wenn Winkelinformationen (des Youbots) aktualisiert werden
	Errechnet: Vz, PSI, PHI, THETA, PHI'
*/
void callback_JointState( const sensor_msgs::JointState::Ptr& msg){

	if( safe == false ){
		//pub_kin_measure.publish( kin_model_save_msg );
		return;
	}

	MatrixXd Jacobi(6,5);	
	
	// Gelenkgeschwindigkeiten aus JointState übertragen
	KDL::JntArray JointVels(5);
	for(int i = 0; i < 5; i++)		
		JointVels(i) = msg->velocity[i];

	// Gelenkpositionen aus JointState übertragen
	KDL::JntArray Joints(5);
	for(int i = 0; i < 5; i++)		
		Joints(i) = msg->position[i] - joint_offsets[i];

	// Jacobi-Matrix aus Gelenkpositionen berechnen
	getJacobi( Jacobi, Joints );

	// TCP-Geschwindigkeiten berechnen (Transl. & Rotat.)
	VectorXd Vels(6);
	Vels = Jacobi * JointVels.data; // [0..2] v, [3..5] w
	
	// Vels skalieren
	Vels(0) /= scaleT;	// VX ~ 0
	Vels(1) /= scaleT;	// VY ~ 0
	Vels(2) /= scaleT;

	// PSI berechnen
	double psi = Joints(0);	
		
	// Neigungswinkel des obersten Gliedes
	double phi = ( Joints(1)+Joints(2)+Joints(3) );

	if( MOV_ONLY == X_ONLY ){
		kin_measure_msg.pose.orientation.z = psi;	
		kin_measure_msg.pose.orientation.y = - phi / scaleR;
		kin_measure_msg.pose.orientation.x = kin_model_save_msg.pose.orientation.x;
	}else{
		kin_measure_msg.pose.orientation.z = psi - M_PI/2;	
		kin_measure_msg.pose.orientation.x = phi / scaleR;
		kin_measure_msg.pose.orientation.y = kin_model_save_msg.pose.orientation.y;
	}

	// Transformation von U -> N
	//kin_measure_msg.vel.linear.z = (-1) * Vels(2);
	kin_measure_msg.vel.linear.z = (1) * Vels(2);

/*	
	// Winkelgeschwindigkeiten (vielleicht nicht möglich aufgrund Rauschen)
	// Drehung um X-Achse nicht möglich -> Modelldaten nehmen
	kin_measure_msg.vel.angular.x = kin_model_save_msg.vel.angular.x;
	kin_measure_msg.vel.angular.y = - Vels(4) /scaleR;
	kin_measure_msg.vel.angular.z = - Vels(5);
*/

	ROS_INFO( "Measure: Vz: % 06.4f, Phi: % 06.4f, Theta: % 06.4f, Psi: % 06.4f", 
		kin_measure_msg.vel.linear.z,
		kin_measure_msg.pose.orientation.x, 		 
		kin_measure_msg.pose.orientation.y,
		kin_measure_msg.pose.orientation.z
		);

	synch.joint_ready = true;
	if( synch.IMU_ready && synch.base_ready ){
		//pub_kin_measure.publish( kin_measure_msg );
		synch.joint_ready = false;
		synch.IMU_ready 	= false;
		synch.base_ready 	= false;
	}	
}

/*
	Wird aufgerufen, wenn IMU neue Daten liefert, Daten wofür???
*/
void callback_imu( const geometry_msgs::Vector3Stamped::Ptr& msg){
	if( safe == false ){
		//pub_kin_measure.publish( kin_model_save_msg );
		return;
	}

	if( MOV_ONLY == X_ONLY ){		
		kin_measure_msg.pose.orientation.y = - msg->vector.y / scaleR;
		kin_measure_msg.pose.orientation.x = kin_model_save_msg.pose.orientation.x;
	}else{
		kin_measure_msg.pose.orientation.x =   msg->vector.x / scaleR;
		kin_measure_msg.pose.orientation.y = kin_model_save_msg.pose.orientation.y;
	}

	synch.IMU_ready = true;
	if( synch.joint_ready && synch.base_ready ){
		//pub_kin_measure.publish( kin_measure_msg );
		synch.joint_ready = false;
		synch.IMU_ready 	= false;
		synch.base_ready 	= false;
	}	
}

/*
	Wird aufgerufen, wenn neue Kinematik-Daten aus der Simulation vorhanden sind
	-> Aufbereitung und Weitergabe an youBot
*/
void callback_kin_model( quadrotor_control::kinematics msg ){
	
	// Unschuldsbehauptung
	safe = true;

	// Kopiere die aktuellen Kinematics
	// für den Fall safe == false -> direkte Rückführung)
	std::memcpy( (void *)&kin_model_save_msg, (const void *)&msg, sizeof kin_model_save_msg );

	// Scale: Vx, Vy, Vz
	//tf::Vector3 
	msg.vel.linear.x *= scaleT;
	msg.vel.linear.y *= scaleT;
	msg.vel.linear.z *= scaleT;			// wird nicht benötigt
	msg.pose.position.x *= scaleT;	// wird nicht benötigt
	msg.pose.position.y *= scaleT;	// wird nicht benötigt
	msg.pose.position.z *= scaleT;

	// Scale: Phi, Theta
	msg.pose.orientation.x *= scaleR;
	msg.pose.orientation.y *= scaleR;

	double g[5];	// Gelenkwinkel
	
	// Transformation von N -> U
	double z = (-1) * msg.pose.position.z;

	double psi, phi, delta;
	
	if( MOV_ONLY == X_ONLY ){		
		psi = msg.pose.orientation.z;					
		phi = M_PI/2 - msg.pose.orientation.y;
		delta = 0;
	}else{		
		psi = msg.pose.orientation.z + M_PI/2;
		phi = M_PI/2 + msg.pose.orientation.x;
		delta = - M_PI/2;
	}
/*
	ROS_INFO( "psi: % 06.4f, phi: % 06.4f, delta: % 06.4f", 
		psi, 
		phi, 
		delta
		);
*/
	// Testen, ob übergebene Pose + Geschwindigkeiten sicher sind
  if( z < 0.5 ){
		ROS_INFO("Unsichere Arbeitshoehe -> Bitte nach oben bewegen");
		safe = false;
		return;
	}
	if( abs(msg.pose.orientation.x) > 0.35 ){
		ROS_INFO("Zu Steil! Rot(X)");
		safe = false;
		return;
	}
	if( abs(msg.pose.orientation.y) > 0.35 ){
		ROS_INFO("Zu Steil! Rot(Y)");
		safe = false;
		return;
	}
	

	// -----------   Inverse Kinematik   -------------------------

	//Hilfsgrößen
	double x4 = k_Arm*( - d5*cos( phi ) ) + a1;
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
	g[4] = delta + (k_Arm-1)/2*M_PI;
	
	// Min - Max überprüfen
	for( int i = 0; i < 5; i++ ){
		if( g[i] < joint_min_angles[i] || g[i] > joint_max_angles[i] || (g[i] != g[i]) ){
			ROS_INFO("Pose nicht anfahrbar. Gelenk %i : %f", (i+1), g[i]);
			safe = false;
			return;
		}
	}

	// joint_offsets addieren
	for( int i = 0; i < 5; i++ ){
		g[i] += joint_offsets[i];	
	}

	// -----------   Inverse Kinematik   ---------   ENDE  --------

	brics_actuator::JointPositions msg_joints;
	geometry_msgs::Twist msg_base;

	// Transformation von N -> U
	msg_base.linear.x =				 msg.vel.linear.x;
	msg_base.linear.y = (-1) * msg.vel.linear.y;
	msg_base.linear.z = 0.0f;
	//ROS_INFO( "BaseV: %f, %f", msg_base.linear.x, msg_base.linear.y );

	msg_base.linear.z  = 0.0f;
	msg_base.angular.x = 0.0f;
	msg_base.angular.y = 0.0f;
	msg_base.angular.z = 0.0f;   

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

	pub_base.publish(msg_base); 
	pub_joint.publish( msg_joints );
}



void getConstants(ros::NodeHandle &nh){
	nh.getParam("scaleT", scaleT);
	ROS_INFO("SCALE V: %f", scaleT);
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
	ros::Subscriber sub_imu   = nh.subscribe("/imu/rpy/filtered", 10, callback_imu);
	// Subscriber für Odometriedaten der Basis
	ros::Subscriber sub_odom	= nh.subscribe("/odom", 10, callback_odom);
  
	pub_kin_measure = nh.advertise<quadrotor_control::kinematics>("/kin_measure", 10);
	pub_joint = nh.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command", 100);
	pub_base  = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
		
	ros::spin();
	return 0;
}
