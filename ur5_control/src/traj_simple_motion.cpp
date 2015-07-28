#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <vector>
#include <algorithm>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <tf_conversions/tf_eigen.h>

#include <ur5_class/lab3.h>
#include <inverse_ur5/lab4.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

//**TO-DO**
//[1] - Implement a check to avoid moving through or close to singularities
//***************************************************************


// global list to hold the setposes. Not very thread safe but it's fine.
// **In ROS, Pose is a data structure composed of Point position and Quaternion
// orientation.
// Whehther list can acccept the Pose data structure is to be tested
// The answer seems to be : NO.
// Define lists for translation and rotation separately
std::list< geometry_msgs::Point > pointlist;
std::list< geometry_msgs::Quaternion > quaternionlist;
bool startUp = true;

// Compute and return the Jacobian of the robot given the current joint 
// positions
// input: the input joint state
// output: the 3x3 Jacobian (position only)
void Jacobian( const sensor_msgs::JointState& jointstate, double J[3][3] ){

	for( int r=0; r<3; r++ )
		for( int c=0; c<3; c++ )
			J[r][c] = 0.0;

	double q1 = jointstate.position[0];
	double q2 = jointstate.position[1];
	double q3 = jointstate.position[2];

	// Fill the values of the Jacobian matrix J
	J[0][0] = 0.4869*sin(q1)*sin(q2)*sin(q3) - 0.425*cos(q2)*sin(q1) - 0.19145*cos(q1) - 0.4869*cos(q2)*cos(q3)*sin(q1);
	J[0][1] = -0.0001*cos(q1)*(4869.0*sin(q2 + q3) + 4250.0*sin(q2));
	J[0][2] = -0.4869*sin(q2 + q3)*cos(q1);

	J[1][0] = 0.425*cos(q1)*cos(q2) - 0.19145*sin(q1) + 0.4869*cos(q1)*cos(q2)*cos(q3) - 0.4869*cos(q1)*sin(q2)*sin(q3);
	J[1][1] = -0.0001*sin(q1)*(4869.0*sin(q2 + q3) + 4250.0*sin(q2));
	J[1][2] = -0.4869*sin(q2 + q3)*sin(q1);

	J[2][0] = 0.0;
	J[2][1] = -0.4869*cos(q2 + q3) - 0.425*cos(q2);
	J[2][2] = -0.4869*cos(q2 + q3);

}

// Inverse a 3x3 matrix
// input: A 3x3 matrix
// output: A 3x3 matrix inverse
// return the determinant inverse
double Inverse( double A[3][3], double Ainverse[3][3] ){

	double determinant = (  A[0][0]*( A[1][1]*A[2][2]-A[2][1]*A[1][2] ) -
			A[0][1]*( A[1][0]*A[2][2]-A[1][2]*A[2][0] ) +
			A[0][2]*( A[1][0]*A[2][1]-A[1][1]*A[2][0] ) );

	double invdet = 1.0/determinant;

	Ainverse[0][0] =  ( A[1][1]*A[2][2] - A[2][1]*A[1][2] )*invdet;
	Ainverse[0][1] = -( A[0][1]*A[2][2] - A[0][2]*A[2][1] )*invdet;
	Ainverse[0][2] =  ( A[0][1]*A[1][2] - A[0][2]*A[1][1] )*invdet;

	Ainverse[1][0] = -( A[1][0]*A[2][2] - A[1][2]*A[2][0] )*invdet;
	Ainverse[1][1] =  ( A[0][0]*A[2][2] - A[0][2]*A[2][0] )*invdet;
	Ainverse[1][2] = -( A[0][0]*A[1][2] - A[1][0]*A[0][2] )*invdet;

	Ainverse[2][0] =  ( A[1][0]*A[2][1] - A[2][0]*A[1][1] )*invdet;
	Ainverse[2][1] = -( A[0][0]*A[2][1] - A[2][0]*A[0][1] )*invdet;
	Ainverse[2][2] =  ( A[0][0]*A[1][1] - A[1][0]*A[0][1] )*invdet;

	return determinant;

}

// This callback function is triggered each time that a set of pose is published.
// The only thing it does is to copy the received 6D pose to the list
// of poses 
// Input: a new set pose (6D pose)
void callback( const geometry_msgs::Pose& newpose ){
	geometry_msgs::Point newpoint;
	geometry_msgs::Quaternion newquaternion;

	newpoint.x = newpose.position.x;
	newpoint.y = newpose.position.y;
	newpoint.z = newpose.position.z;

	newquaternion.x = newpose.orientation.x;
	newquaternion.y = newpose.orientation.y;
	newquaternion.z = newpose.orientation.z;
	newquaternion.w = newpose.orientation.w;

	std::cout << "New point\n" << newpoint << std::endl;
	std::cout << "New quaternion\n" << newquaternion << std::endl;
	pointlist.push_back( newpoint );
	quaternionlist.push_back( newquaternion );
}

// This callback_joint_states function is triggered each loop during the spin
//We grab the initial pose of the robot and then ignore and update in the loop
sensor_msgs::JointState jointstate_init;
ros::Publisher spoof_first_move;
void callback_joint_states( const sensor_msgs::JointState& js ) {
	if (startUp) {
		jointstate_init = js;
		std_msgs::BoolPtr firstMove(new std_msgs::Bool);
		firstMove->data = true;
		spoof_first_move.publish(firstMove);
		std_msgs::BoolPtr firstMove_end(new std_msgs::Bool);
		firstMove_end->data = false;
		spoof_first_move.publish(firstMove_end);
		startUp = false;
	}
}


// actionlib
trajectory_msgs::JointTrajectory joint_trajectory;
void callback_move( const std_msgs::Bool& move ){

	static actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac( "follow_joint_trajectory", true );

	if( move.data ){

		control_msgs::FollowJointTrajectoryGoal goal;
		goal.trajectory = joint_trajectory;
		std::cout << goal << std::endl;

		ac.sendGoal( goal );
		joint_trajectory.points.clear();

	}

}

int main( int argc, char** argv ){

	// Initialize ROS node "trajectory"
	ros::init( argc, argv, "trajectory" );

	// Create a node handle
	ros::NodeHandle nh;
	ros::NodeHandle nm;

	// publishing joint trajectory
	std::string published_topic_name( "/joint_trajectory" );

	// Create a publisher that will publish joint states on the topic
	// /joint_states

	// The publisher will publish to /joint_trajectory at the end of
	// of the main function after completing all the computation.
	// The size of publishing queue is 1 which means it will only buffer
	// up 1 message. The old message will be thrown away immediately
	// if a new one is received
	ros::Publisher pub_jointstate;
	pub_jointstate = nh.advertise<sensor_msgs::JointState>( published_topic_name, 1 );

	//Experimental to get robot to move immediately

	spoof_first_move = nm.advertise<std_msgs::Bool>( "move", 1 );

	// Create a subscriber that will receive 6D setposes
	ros::Subscriber sub_setpose;
	sub_setpose = nh.subscribe( "setpose", 1, callback );

	// This is the joint state message coming from the robot
	// Get the initial joint state
	// sub_move is currently not used anywhere
	ros::Subscriber sub_move;
	sub_move = nh.subscribe( "move", 1, callback_move );

	// This is the joint state message coming from the robot
	// It is currently only being used to grab the initial pose    
	ros::Subscriber full_joint_states;
	full_joint_states = nh.subscribe( "joint_states", 1, callback_joint_states );

	//Object to publish desired joint positions of the robot
	// sensor_msgs::JointState jointstate;
	sensor_msgs::JointState des_jointstate;


	// To listen to the current position and orientation of the robot
	// wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29
	tf::TransformListener listener;

	// Rate (Hz) of the trajectory
	// http://wiki.ros.org/roscpp/Overview/Time
	ros::Rate rate( 75 );            // the trajectory rate
	double period = 2.0/150.0;        // the period
	double positionincrement = 1.0/100.0;
	ros::Duration time_from_start( 0.0 );

	bool readinitpose = true;                       // used to initialize setpose
	bool moving = false;
	tf::Pose setpose;                               // the destination setpose
	Eigen::VectorXf diff(6);
	Eigen::Matrix3f rot;
	bool solver;
	// This is the main trajectory loop
	// At each iteration it computes and publishes a new joint positions that
	// animate the motion of the robot
	// The loop exits when CTRL-C is pressed.
	while( nh.ok() ){

		// Read the current forward kinematics of the robot
		tf::Pose current_pose;

		try{

			// Change the name of the reference frame and target frame.
			// These names will be used by tf to return the position/orientation of
			// the last link of the robot with respect to the base of the robot
			std::string ref_frame( "base_link" );
			std::string tgt_frame( "ee_link" );

			//Wait for initial transform from initialization
			ros::Time t_zero = ros::Time(0);
			tf::StampedTransform transform;
			listener.waitForTransform( ref_frame, tgt_frame, t_zero, ros::Duration(0.8));
			listener.lookupTransform( ref_frame, tgt_frame, t_zero, transform );

			// convert tf into ROS message information
			// Note: tf is used to perform mathematical calculation
			// while ROS message is used for communication between publisher/subscriber
			// broadcaster etc.
			current_pose.setOrigin( transform.getOrigin() );
			current_pose.setRotation( transform.getRotation() );
			//Why doesn't RViz and therefore current_pose update after publishing new joint positions?r

			/* This is used to initialize the pose (a small hack)*/
			if( readinitpose ){
				setpose = current_pose;
				readinitpose = false;
			}

		}
		catch(tf::TransformException ex)
		{ std::cout << ex.what() << std::endl; }

		// If the list is not empty,
		// Read a setpose from the list and keep both the translation and rotation
		//If a new pose has not been sent, remain in the current pose
		if( !pointlist.empty() && !quaternionlist.empty() ){

			// set origin of the goal pose
			double x_f = pointlist.front().x;
			double y_f = pointlist.front().y;
			double z_f = pointlist.front().z;
			setpose.setOrigin( tf::Vector3(x_f, y_f, z_f) );
			pointlist.pop_front();

			// set orientation of the goal pose
			double q_xf = quaternionlist.front().x;
			double q_yf = quaternionlist.front().y;
			double q_zf = quaternionlist.front().z;
			double q_wf = quaternionlist.front().w;
			setpose.setRotation( tf::Quaternion(q_xf, q_yf, q_zf, q_wf));
			quaternionlist.pop_front();

			// Convert Pose to Affine3d to Affine3f to Matrix4f
			// You can see how complex it is even to convert Pose into
			// an element of SE3
			Eigen::Affine3d H0_6d;
			tf::poseTFToEigen( setpose, H0_6d);
			Eigen::Matrix4d H_M = H0_6d.matrix();
			Eigen::MatrixXf H_6 = H_M.cast <float> ();

			//Create rotation matrix for comparison with current pose	   
			Eigen::MatrixXf H_rot = H_6;
			//Rotation Matrix tranpose for inverse
			rot << H_rot(0,0), H_rot(1,0), H_rot(2,0),
			    H_rot(0,1), H_rot(1,1), H_rot(2,1),
			    H_rot(0,2), H_rot(1,2), H_rot(2,2);

			//Transform the pose into raw D-H parameters to generate the proper
			//inverse kinematic (IK) solutions
			Eigen::Matrix4f T_0;
			T_0 << -1.0, 0.0, 0.0, 0.0,
			    0.0, -1.0, 0.0, 0.0,
			    0.0, 0.0, 1.0, 0.0, 
			    0.0, 0.0, 0.0, 1.0;
			Eigen::Matrix4f T_f;
			T_f << 0.0, 0.0, 1.0, 0.0,
			    -1.0, 0.0, 0.0, 0.0,
			    0.0, -1.0, 0.0, 0.0, 
			    0.0, 0.0, 0.0, 1.0;
			H_6 = T_0*H_6*T_f;

			//Create memory for IK solutions
			double* q_sol[8];
			for (int i = 0; i < 8; ++i) {
				q_sol[i] = new double[6];
			}
			//Solve IK
			int num_sol = inverse(H_6, q_sol);

			//Ensure the pose is reachable
			if (num_sol == 0) {
				ROS_WARN("Desired pose is not reachable, choice ignored");
				des_jointstate = jointstate_init;
				continue;
			}
			//Compare the solutions to determine which requires the least amount of joint movement
			std::vector<double> ang_dist;
			double dists = 0;
			for (int i = 0; i < num_sol; i++) {	      
				for (int j = 0; j < 6; j++) {
					//add weight to first 3 joints?
					if (j == 0 || j == 1 || j == 2) {
						dists += (jointstate_init.position[j] - q_sol[i][j])*(jointstate_init.position[j] - q_sol[i][j]);
					}
					else {
						dists += (jointstate_init.position[j] - q_sol[i][j])*(jointstate_init.position[j] - q_sol[i][j]);
					}		
				}
				ang_dist.push_back(sqrt(dists));
				dists = 0;
			}

			//Return the iterator value for the smallest angular difference
			int angs = std::distance(ang_dist.begin(),std::min_element(ang_dist.begin(), ang_dist.end()));
			//std::vector<float> signed_diff;
			double signed_diff;
			//Update the joint positions
			for (int i = 0; i < 6; ++i)
			{	        
				des_jointstate.position[i] = q_sol[angs][i];
				diff(i) = des_jointstate.position[i] - jointstate_init.position[i];
				// signed_diff.push_back(fabs(diff(i)));	 
				signed_diff += diff(i)*diff(i);     
			}	   
			//diff /= *std::max_element(signed_diff.begin(),signed_diff.end())*200;
			diff /= sqrt(signed_diff)*100;
			//Defualt to jointspace method
			if ( ros::param::get("solver_method", solver) ) {   }
			else { ros::param::set("solver_method", true);  }

			//After pose is updated, change the initial position to current position
			moving = true;
			joint_trajectory.points.clear();
			time_from_start = ros::Duration(0.0);
			trajectory_msgs::JointTrajectoryPoint point;
			point.positions = jointstate_init.position;
			point.velocities = std::vector<double>( 6, 0.0 );
			point.time_from_start = time_from_start;
			time_from_start = time_from_start + ros::Duration( period );

			joint_trajectory.points.push_back( point );
			joint_trajectory.joint_names = jointstate_init.name;

			// ROS_INFO_STREAM("joints" << jointstate_init.position[0] << "   " <<  jointstate_init.position[1] <<"   " << jointstate_init.position[2] <<"   " << jointstate_init.position[3] <<"   " <<jointstate_init.position[4] <<"   " << jointstate_init.position[5]);

		}
		//Iterate over trajectory to generate motion
		else if (moving) {
			sensor_msgs::JointState jointstate;
			jointstate = jointstate_init;
			//If true use jointspace angle incrementing, if false use Jacobian based methods
			if (solver) {
				for (int i = 0; i < 6; i++) {
					jointstate.position[i] += diff(i); 
				}
			}
			//Jacobian-based update method
			else {
				tf::Point error = setpose.getOrigin() - current_pose.getOrigin();
				tf::Point v = ( error / error.length() ) * positionincrement;

				double J[3][3], Ji[3][3];

				// Compute the Jacobian
				Jacobian( jointstate, J );

				// Compute the inverse Jacobian. The inverse return the
				// value of the determinant.
				if( fabs(Inverse( J, Ji )) < 1e-09 )
				{ std::cout << "Jacobian is near singular." << std::endl; }


				// This is the inverse kinematics realization for the translation
				//   of UR5 by incrementing the joint postions

				// Compute the joint velocity by multiplying the (Ji v)
				double qd[3];
				qd[0] = Ji[0][0]*v[0] + Ji[0][1]*v[1] + Ji[0][2]*v[2];
				qd[1] = Ji[1][0]*v[0] + Ji[1][1]*v[1] + Ji[1][2]*v[2];
				qd[2] = Ji[2][0]*v[0] + Ji[2][1]*v[1] + Ji[2][2]*v[2];

				// increment the joint positions
				jointstate.position[0] += qd[0];
				jointstate.position[1] += qd[1];
				jointstate.position[2] += qd[2];

				//While waiting to derive full Jacobian, update last 3 joints by incrementing (need to fix timing issue)
				for (int i = 3; i < 6; i++) {
					jointstate.position[i] += diff(i); 
				}

			}

			trajectory_msgs::JointTrajectoryPoint point;
			point.positions = jointstate.position;
			point.velocities = std::vector<double>( 6, 0.0 );
			point.time_from_start = time_from_start; 
			time_from_start = time_from_start + ros::Duration( period );
			joint_trajectory.points.push_back( point ); 

			//If we reach desired pose, stop and wait for new pose
			tf::Point pos_error = current_pose.getOrigin() - setpose.getOrigin();
			Eigen::Affine3d H0_6r;
			tf::poseTFToEigen( current_pose, H0_6r);
			Eigen::Matrix4d H_R = H0_6r.matrix();
			Eigen::MatrixXf H_cur_rot = H_R.cast <float> ();
			Eigen::Matrix3f cur_rot;
			cur_rot << H_cur_rot(0,0), H_cur_rot(0,1), H_cur_rot(0,2), 
				H_cur_rot(1,0), H_cur_rot(1,1), H_cur_rot(1,2), 
				H_cur_rot(2,0), H_cur_rot(2,1), H_cur_rot(2,2);
			Eigen::Matrix3f rot_id;
			rot_id = rot*cur_rot;
			float rot_error = sqrt( (rot_id(0,0) - 1)*(rot_id(0,0) - 1) + (rot_id(1,1) - 1)*(rot_id(1,1) - 1) + (rot_id(2,2) - 1)*(rot_id(2,2) - 1) );
			if (pos_error.length() < 1.0/100.0 && rot_error < 0.1) { // 0.001 && rot_error < 0.05) {
				moving = false;
				if (solver) {
					point.positions = des_jointstate.position;
					point.velocities = std::vector<double>( 6, 0.0 );
					point.time_from_start = time_from_start;
					joint_trajectory.points.push_back( point ); 
					jointstate_init = des_jointstate;
				}
				continue;
			}
			jointstate_init = jointstate;
			}
			else {
				des_jointstate = jointstate_init;
				moving = false;
			}

			// publish the joint states
			jointstate_init.header.stamp = ros::Time::now();
			pub_jointstate.publish( jointstate_init );

			// Go through callback functions
			ros::spinOnce();

			// sleep
			rate.sleep();

		}

		return 0;

	}

