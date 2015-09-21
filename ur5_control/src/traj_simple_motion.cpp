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


#define PI 3.1415926

//**TO-DO**
//[1] - Implement a check to avoid moving through or close to singularities (may not be necessary)
//[2] - Self Collision Avoidance (easily implementable with a switch to moveit!
//[3] - Add joint limit checks
//***************************************************************

// Define lists for translation and rotation separately
std::list< geometry_msgs::Point > pointlist;
std::list< geometry_msgs::Quaternion > quaternionlist;
bool startUp = true;

// Compute and return the Jacobian of the robot given the current joint 
// positions
// input: the input joint state
// output: the 3x3 Jacobian (position only)

Eigen::MatrixXf invJacobian( const sensor_msgs::JointState& jointstate) {
	double q[6] = { jointstate.position[0], jointstate.position[1], jointstate.position[2], jointstate.position[3],
		jointstate.position[4], jointstate.position[5] };
	Eigen::MatrixXf J(6,6);
	Eigen::Matrix4f H06 = UR5::fwd(q);
	Eigen::Matrix4f H = Eigen::Matrix4f::Identity(4,4);
	Eigen::Matrix4f Rev;
	Rev << -1.0, 0.0, 0.0, 0.0,
	    0.0, -1.0, 0.0, 0.0,
	    0.0, 0.0, 1.0, 0.0, 
	    0.0, 0.0, 0.0, 1.0;

	H06 = Rev*H06;
	Eigen::Matrix4f H_temp;
	for (int i = 0; i < 6; i++) {
		H_temp = Rev*H;
		J.block<3,1>(0,i) = skew3(H_temp.block<3,1>(0,2))*(H06.block<3,1>(0,3) - H_temp.block<3,1>(0,3));
		J.block<3,1>(3,i) = H_temp.block<3,1>(0,2);
		H = H*UR5::dhf(UR5::alpha[i], UR5::a[i], UR5::d[i], q[i]);
	}

	/*
	   double c1 = cos(q[0]); double s1 = sin(q[0]); double c2 = cos(q[1]); double s2 = sin(q[1]);double c5 = cos(q[4]); 
	   double s5 = sin(q[4]); double c6 = cos(q[5]); double s6 = sin(q[5]);
	   double s23 = sin(q[1]+q[2]); double c23 = cos(q[1]+q[2]); double s234 = sin(q[1]+q[2]+q[3]);  double c234 = cos(q[1]+q[2]+q[3]); 
	//Fixed-Frame Vels
	double r11 = 0;
	double r12 = -s1;
	double  r13 = -s1;
	double  r14 = -s1;
	double  r15 = -c1*s234;
	double  r16 = (c1*c234*s5 - s1*c5);
	double  r21 = 0;
	double  r22 = c1;
	double  r23 = c1;
	double r24 = c1;
	double r25 = -s1*s234;
	double r26 = (c1*c5 + s1*c234*s5);
	double r31 = 1;
	double r32 = 0;
	double r33 = 0; 
	double r34 = 0;
	double r35 = -c234;
	double r36 = -s234*s5;

	double t11 = ((1893*s1*s234)/20 - (823*c1*c5)/10 - 425*s1*c2 - (2183*c1)/20 - (1569*s1*c23)/4 - (823*s1*c234*s5)/10)/1000;
	double t12 = -(c1*(1893*c234 + 7845*s23 + 8500*s2 + 1646*s234*s5))/20000;
	double t13 = -(c1*(1893*c234 + 7845*s23 + 1646*s234*s5))/20000;
	double t14 = -(c1*(1893*c234 + 1646*s234*s5))/20000;
	double t15 = ((823*s1*s5)/10 + (823*c1*c234*c5)/10)/1000; 
	double t16 = 0;
	double t21 = (425*c1*c2 - (2183*s1)/20 - (823*s1*c5)/10 - (1893*c1*s234)/20 + (1569*c1*c23)/4 + (823*c1*c234*s5)/10)/1000;
	double t22 = -(s1*(1893*c234 + 7845*s23 + 8500*s2 + 1646*s234*s5))/20000;
	double t23 = -(s1*(1893*c234 + 7845*s23 + 1646*s234*s5))/20000;
	double t24 = -(s1*(1893*c234 + 1646*s234*s5))/20000;
	double t25 = ((823*s1*c234*c5)/10 - (823*c1*s5)/10)/1000; 
	double t26 = 0;
	double t31 = 0;
	double t32 = (1893*s234)/20000 - (1569*c23)/4000 - (425*c2)/1000 - (823*c234*s5)/10000;
	double t33 = (1893*s234)/20000 - (1569*c23)/4000 - (823*c234*s5)/10000;
	double t34 = (1893*s234)/20000 - (823*c234*s5)/10000;
	double t35 =  -(823*s234*c5)/10000;
	double t36 = 0;

	//Eigen::MatrixXf J_R(6,6);
	/* J << t11, t12, t13, t14, t15, t16,
	t21, t22, t23, t24, t25, t26,
	t31, t32, t33, t34, t35, t36,
	r11, r12, r13, r14, r15, r16,
	r21, r22, r23, r24, r25, r26,
	r31, r32, r33, r34, r35, r36;
	*/
	//J1.block<6,6>(0,0) = J_R;

	/*
	   std::cout << "J" << J << std::endl;
	   std::cout << "    " << std::endl;
	   std::cout << "J1" << J1 << std::endl;
	   std::cout << "    " << std::endl;
	   */


	//Check to see if invJacobian is near singular
	if (fabs(J.determinant()) < 10e-9) {
		std::cout << "Jacobian is near singular." << std::endl;
		J += Eigen::MatrixXf::Identity(6,6)*0.0001;
	}

	return J.inverse();

}


Eigen::VectorXf lieAlgebra(Eigen::MatrixXf pose) {

	Eigen::Matrix3f R;
	Eigen::Vector3f omega, v;
	
	// test translation error only
	Eigen::VectorXf si(6);
	// Eigen::VectorXf si(6);
	R = pose.block<3,3>(0,0);
	Eigen::MatrixXf wl = R.log();
	Eigen::Vector3f w;
	w << wl(2,1), 
	  wl(0,2),
	  wl(1,0);
	v = pose.block<3,1>(0,3);
	
	si.head(3) = v;
	si.tail(3) = w;

	return si;

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
	// sensor_msgs::JointState joint_temp;


	// To listen to the current position and orientation of the robot
	// wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29
	tf::TransformListener listener;

	// Rate (Hz) of the trajectory
	// http://wiki.ros.org/roscpp/Overview/Time
	ros::Rate rate( 50 );            // the trajectory rate
	double period = 1.0/100.0;        // the period
	double positionincrement = 1.0/100.0;
	ros::Duration time_from_start( 0.0 );

	bool readinitpose = true;                       // used to initialize setpose
	bool moving = false;
	tf::Pose setpose;                               // the destination setpose
	Eigen::VectorXf diff(6);
	Eigen::Matrix3f rot;
	bool solver, robot_mover;
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

			//Check solver method (default to Jacobian method)
			if ( ros::param::get("solver_method", solver) ) {   }
			else { ros::param::set("solver_method", false);
				solver = false;  }

			//Check simulation or robot motion (default robot motion)
			if ( ros::param::get("robot_move", robot_mover) ) {   }
			else { ros::param::set("robot_move", true);
				robot_mover = true;  }



			if (solver) {
				//Transform the pose into raw D-H parameters to generate the proper
				//inverse kinematic (IK) solutions
				Eigen::Matrix4f T_0,T_f;
				T_0 << -1.0, 0.0, 0.0, 0.0,
				    0.0, -1.0, 0.0, 0.0,
				    0.0, 0.0, 1.0, 0.0, 
				    0.0, 0.0, 0.0, 1.0;

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
					if (fabs(diff(i)) > PI) diff(i) < 0 ? diff(i) += 2*PI: diff(i) -= 2*PI;
					// signed_diff.push_back(fabs(diff(i)));	 
					signed_diff += diff(i)*diff(i);     
				}	   
				//diff /= *std::max_element(signed_diff.begin(),signed_diff.end())*200;
				diff /= sqrt(signed_diff)*100;
			}

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

		}
		//Iterate over trajectory to generate motion
		else if (moving) {
			sensor_msgs::JointState jointstate;
			jointstate = jointstate_init;

			Eigen::VectorXf evel(6); 
			double scale;
			Eigen::Affine3d H_Cpd, H_Spd;
			tf::poseTFToEigen( setpose, H_Spd);
			tf::poseTFToEigen( current_pose, H_Cpd);
			Eigen::Matrix4d H_Sp = H_Spd.matrix();
			Eigen::Matrix4d H_Cp = H_Cpd.matrix();
			Eigen::MatrixXf Sp = H_Sp.cast <float> ();
			Eigen::MatrixXf Cp = H_Cp.cast <float> ();
			Eigen::MatrixXf g_error, temp;
			temp = Sp;
			Sp.block<3,3>(0,0) = temp.block<3,3>(0,0).transpose();
			Sp.block<3,1>(0,3) = -Sp.block<3,3>(0,0)*temp.block<3,1>(0,3);
			g_error = Cp*Sp;
			evel = lieAlgebra(g_error);	 

			//If true use jointspace angle incrementing, if false use Jacobian based methods
			if (solver) {
				for (int i = 0; i < 6; i++) {
					jointstate.position[i] += diff(i); 
				}
			}
			//Jacobian-based update method
			else { 
				// Compute the Jacobian
				Eigen::MatrixXf Ji = invJacobian(jointstate);
				Eigen::VectorXf evt(6);
				// Compute the joint velocity by multiplying Ji by v
				Eigen::VectorXf qd(6);
				//evel = (evel/(1+evel.norm()))*positionincrement; 
				evt = (evel/evel.norm())*positionincrement*0.025; // previously 0.05
				qd = Ji*evt;
				for (int j = 0; j < 6; j++) {
					jointstate.position[j] -= qd(j);
				} 

			}

			trajectory_msgs::JointTrajectoryPoint point;
			point.positions = jointstate.position;
			point.velocities = std::vector<double>( 6, 0.0 );
			point.time_from_start = time_from_start; 
			time_from_start = time_from_start + ros::Duration( period );
			joint_trajectory.points.push_back( point ); 

			//Set robot to move
			if (robot_mover) {
				std_msgs::BoolPtr firstMove_end(new std_msgs::Bool);
				firstMove_end->data = true;
				spoof_first_move.publish(firstMove_end);
			}

			!solver ? scale = 0.1 : scale = 0.5; // Previously 0.2
			std::cout << "Norm of the position error is " << evel.norm() << std::endl;
			if (evel.norm() <= positionincrement*scale) {	 
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

