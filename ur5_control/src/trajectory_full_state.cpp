#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <tf_conversions/tf_eigen.h>

#include <inverse_ur5/lab4.h>
#include <ur_kinematics/ur_kin.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>


// global list to hold the setposes. Not very thread safe but it's fine.
// **In ROS, Pose is a data structure composed of Point position and Quaternion
// orientation.
// Whehther list can acccept the Pose data structure is to be tested
// The answer seems to be : NO.
// Define lists for translation and rotation separately
std::list< geometry_msgs::Point > pointlist;
std::list< geometry_msgs::Quaternion > quaternionlist;

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

// This callback_joint_states function is triggered each time when a joint state
// is published by the ur_driver. The only thing it does to initialize
// the initial joint state of the UR5 urdf model in RVIZ ??
sensor_msgs::JointState jointstate_init;
void callback_joint_states( const sensor_msgs::JointState& js )
{ jointstate_init = js; }

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
    ros::Subscriber sub_move;
    sub_move = nh.subscribe( "move", 1, callback_move );

    // This is the joint state message coming from the robot
    // Get the initial joint state
    // full_joint_state is not used anywhere either
    ros::Subscriber full_joint_states;
    full_joint_states = nh.subscribe( "joint_states", 1, callback_joint_states );
    sensor_msgs::JointState jointstate;


    // To listen to the current position and orientation of the robot
    // wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29
    tf::TransformListener listener;

    // Rate (Hz) of the trajectory
    // http://wiki.ros.org/roscpp/Overview/Time
    ros::Rate rate( 100 );            // the trajectory rate
    double period = 1.0/100.0;        // the period
    double positionincrement = 0.001; // how much we move
    ros::Duration time_from_start( 0.0 );

    bool readinitpose = true;                       // used to initialize setpose
    bool moving = false;
    tf::Pose setpose;                               // the destination setpose

    // This is the main trajectory loop
    // At each iteration it computes and publishes a new joint positions that
    // animate the motion of the robot
    // The loop exits when CTRL-C is pressed.
    while( nh.ok() ){

        // Read the current forward kinematics of the robot
        // wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29
        tf::Pose current_pose;
        try{

            // Change the name of the reference frame and target frame.
            // These names will be used by tf to return the position/orientation of
            // the last link of the robot with respect to the base of the robot
            std::string ref_frame( "base_link" );
            std::string tgt_frame( "ee_link" );

            tf::StampedTransform transform;
            listener.lookupTransform( ref_frame, tgt_frame, ros::Time(0), transform );

            // convert tf into ROS message information
            // Note: tf is used to perform mathematicla calculation
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

            moving = true;
            joint_trajectory.points.clear();

            time_from_start = ros::Duration(0.0);
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions = jointstate.position;
            point.velocities = std::vector<double>( 6, 0.0 );
            point.time_from_start = time_from_start;
            time_from_start = time_from_start + ros::Duration( period );

            joint_trajectory.points.push_back( point );
            joint_trajectory.joint_names = jointstate.name;

        }

        // This is the translation left for the trajectory
        tf::Point error = setpose.getOrigin() - current_pose.getOrigin();

        //  ** The inverse kinematics of UR5 for all six dofs is to be implemented.
        //  ** Possible solutions are ik_fast in ur_kinematics (which suffers
        //  ** from a 90% success rate), kdl_chainsolver, or the inverse kinematics
        //  ** package in me530646_lab4.

        // If the error is small enough to go to the destination
        if( moving && positionincrement < error.length() ){

            // Determine a desired cartesian linear velocity.
            // We will command the robot to move with this velocity
            tf::Point v = ( error / error.length() ) * positionincrement;

            double J[3][3], Ji[3][3];

            // Compute the Jacobian
            Jacobian( jointstate, J );

            // Compute the inverse Jacobian. The inverse return the
            // value of the determinant.
            if( fabs(Inverse( J, Ji )) < 1e-09 )
            { std::cout << "Jacobian is near singular." << std::endl; }

            // Convert Pose to Affine3d to Affine3f to Matrix4f
            // You can see how complex it is even to convert Pose into
            // an element of SE3
            Eigen::Affine3d H0_6d;
            tf::poseTFToEigen( setpose, H0_6d);
            Eigen::Affine3f H0_6f = H0_6d.cast<float>();
            Eigen::Matrix4f H_M = H0_6f.matrix();


            // double pointer to store up to 8 ik solutions
            double *q_sol[8];
            for(int i = 0; i < 8; i++ )
            {
                // new will keep the data until it is deleted manually
                q_sol[i] = new double[6];
            }

            int num_sol = inverse(H_M, q_sol);

            // joint selection algorithm is needed to provide safe and
            // smooth trajectory

            for (int i = 0; i < 6; ++i)
            {
                jointstate.position[i] = q_sol[0][i];
            }


            /* This is the inverse kinematics realization for the translation
               of UR5 by incrementing the joint postions

            // Compute the joint velocity by multiplying the (Ji v)
            double qd[3];
            qd[0] = Ji[0][0]*v[0] + Ji[0][1]*v[1] + Ji[0][2]*v[2];
            qd[1] = Ji[1][0]*v[0] + Ji[1][1]*v[1] + Ji[1][2]*v[2];
            qd[2] = Ji[2][0]*v[0] + Ji[2][1]*v[1] + Ji[2][2]*v[2];

            // increment the joint positions
            jointstate.position[0] += qd[0];
            jointstate.position[1] += qd[1];
            jointstate.position[2] += qd[2];
            */


            trajectory_msgs::JointTrajectoryPoint point;
            point.positions = jointstate.position;
            point.velocities = std::vector<double>( 6, 0.0 );
            point.time_from_start = time_from_start;
            time_from_start = time_from_start + ros::Duration( period );

            joint_trajectory.points.push_back( point );

        }
        else{
            jointstate = jointstate_init;
            setpose = current_pose;
            moving = false;
        }

        // publish the joint states
        jointstate.header.stamp = ros::Time::now();
        pub_jointstate.publish( jointstate );

        //
        ros::spinOnce();

        // sleep
        rate.sleep();

    }

    return 0;

}

