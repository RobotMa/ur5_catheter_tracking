#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <vector>
#include <algorithm>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <tf_conversions/tf_eigen.h>

#include <rviz_plot/lab1.h>
#include <rviz_animate/lab2.h>
#include <ur5_class/lab3.h>
#include <inverse_ur5/lab4.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
//#include <ur_kinematics/ur_kin.h>


//**TO-DO**
//Implement a trajectory between the desired pose and initial pose
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
void callback_joint_states( const sensor_msgs::JointState& js ) {
  if (startUp) {
    jointstate_init = js;
  }
}


// actionlib
/*
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
*/
int main( int argc, char** argv ){

    // Initialize ROS node "trajectory"
    ros::init( argc, argv, "trajectory" );

    // Create a node handle
    ros::NodeHandle nh;

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

    // Create a subscriber that will receive 6D setposes
    ros::Subscriber sub_setpose;
    sub_setpose = nh.subscribe( "setpose", 1, callback );

    // This is the joint state message coming from the robot
    // Get the initial joint state
    // sub_move is currently not used anywhere
    // ros::Subscriber sub_move;
    //sub_move = nh.subscribe( "move", 1, callback_move );

    // This is the joint state message coming from the robot
    // It is currently only being used to grab the initial pose    
    ros::Subscriber full_joint_states;
    full_joint_states = nh.subscribe( "joint_states", 1, callback_joint_states );
    
    //Object to publish desired joint positions of the robot
    sensor_msgs::JointState jointstate;


    // To listen to the current position and orientation of the robot
    // wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29
    tf::TransformListener listener;
 
    // Rate (Hz) of the trajectory
    // http://wiki.ros.org/roscpp/Overview/Time
    ros::Rate rate( 100 );            // the trajectory rate
    double period = 1.0/100.0;        // the period
    double positionincrement = 0.001; // how much we move
    //ros::Duration time_from_start( 0.0 );

    bool readinitpose = true;                       // used to initialize setpose
    bool moving = false;
    tf::Pose setpose;                               // the destination setpose

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

	    // moving = true;
	    // joint_trajectory.points.clear();
        

            // Convert Pose to Affine3d to Affine3f to Matrix4f
            // You can see how complex it is even to convert Pose into
            // an element of SE3
	    Eigen::Affine3d H0_6d;
            tf::poseTFToEigen( setpose, H0_6d);
            Eigen::Matrix4d H_M = H0_6d.matrix();
	    Eigen::MatrixXf H_6 = H_M.cast <float> ();

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
	      jointstate = jointstate_init;
	      continue;
	    }
	    //Compare the solutions to determine which requires the least amount of joint movement
	    std::vector<double> ang_dist;
	    double dists = 0;
	    for (int i = 0; i < num_sol; i++)
	    {	      
	      for (int j = 0; j < 6; j++)
	      {	
		dists += (jointstate_init.position[j] - q_sol[i][j])*(jointstate_init.position[j] - q_sol[i][j]);
	      }
	      ang_dist.push_back(sqrt(dists));
	      dists = 0;
	    }

	    //Return the iterator value for the smallest angular difference
	    int angs = std::distance(ang_dist.begin(),std::min_element(ang_dist.begin(), ang_dist.end()));
	    
	    //Update the joint positions
            for (int i = 0; i < 6; ++i)
            {
	      //jointstate.position[i] = 0;	        
	      jointstate.position[i] = q_sol[angs][i];	      
	    }	   
	   
	    //After pose is updated, change the initial position to current position
	    startUp = false;
	    jointstate_init = jointstate;
	    // ROS_INFO_STREAM("joints" << jointstate_init.position[0] << "   " <<  jointstate_init.position[1] <<"   " << jointstate_init.position[2] <<"   " << jointstate_init.position[3] <<"   " <<jointstate_init.position[4] <<"   " << jointstate_init.position[5]);
   
	}
	else {
	  jointstate = jointstate_init;
	}

        // publish the joint states
        jointstate.header.stamp = ros::Time::now();
        pub_jointstate.publish( jointstate );

        // Go through callback functions
        ros::spinOnce();

        // sleep
        rate.sleep();

    }

    return 0;

}

