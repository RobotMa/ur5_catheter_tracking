#include <ros/ros.h>
#include <Eigen/Core>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ur5/utilities.h>

RvizPlotter::RvizFrame::RvizFrame(tf::Transform transform, std::string parentName, std::string childName){
  this->transform = transform; 
  this->parentName = parentName; 
  this->childName = childName;
}

void RvizPlotter::plotf(Eigen::Matrix4f h, std::string parentName, std::string childName){
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(h(0,3), h(1,3), h(2,3)));
  transform.setBasis(tf::Matrix3x3(h(0,0), h(0,1), h(0,2),
                               h(1,0), h(1,1), h(1,2),
                               h(2,0), h(2,1), h(2,2)));
  for(int i=0; i < frames.size(); i++)
  {
    if(childName.compare(frames[i].childName.c_str()) == 0)
    {
      frames[i].parentName = parentName;
      frames[i].transform = transform;
      usleep(50000);
      return;
    }  
  }
  frames.push_back(RvizPlotter::RvizFrame(transform, parentName, childName));
  usleep(50000);
}

void RvizPlotter::plotf(Eigen::Matrix4f h, std::string frameName){
  plotf(h,"map",frameName);
}



void RvizPlotter::plotv(std::string frameName, Eigen::Vector3f start, Eigen::Vector3f end)
{
  frameName.insert(0,"/");
  visualization_msgs::Marker marker;
  marker.header.frame_id = frameName;
  marker.header.stamp = ros::Time();
  marker.ns = "vectors";
  marker.id = vectors.size();
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.4;
  geometry_msgs::Point point1;
  geometry_msgs::Point point2;
  point1.x = start(0);
  point1.y = start(1);
  point1.z = start(2);
  point2.x = end(0);
  point2.y = end(1);
  point2.z = end(2);
  marker.scale.x = 0.03;
  marker.scale.y = 0.07;
  marker.scale.z = 0.1;
  marker.points.push_back(point1);
  marker.points.push_back(point2);
  marker.lifetime = ros::Duration();
  vectors.push_back(marker);
}

void RvizPlotter::broadcast(){
  while(ros::ok()){
    boost::this_thread::interruption_point();
    for(int i = 0; i < frames.size(); i++){
      br.sendTransform(tf::StampedTransform(frames[i].transform, ros::Time::now(), frames[i].parentName, frames[i].childName));
      double roll; 
      double pitch; 
      double yaw;
      frames[i].transform.getBasis().getRPY(roll,pitch,yaw);
    }
    for(int i = 0; i < vectors.size(); i++){
      pb.publish(vectors[i]);
    }
    boost::this_thread::sleep(boost::posix_time::millisec(10));
  }
}

/**
 * Default constructor included to allow RvizPlotters to be 
 * declared before calling proper constructor with NodeHandle. 
 */
RvizPlotter::RvizPlotter(){}

RvizPlotter::RvizPlotter(ros::NodeHandle &n){
  tf::TransformBroadcaster br;
  pb = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  broadcastThread = new boost::thread(boost::bind(&RvizPlotter::broadcast, this));
}

RvizPlotter::~RvizPlotter()
{
  //Keep plotting around for long enough for tf to latch
  usleep(500000);
  broadcastThread->interrupt();
  broadcastThread->join();
  delete broadcastThread;
}

void printEigen(const Eigen::MatrixXf& m){
  Eigen::IOFormat fmt(2, 0, ", ", "\n", "[", "]");
  std::cout << m.format(fmt) << "\n" << std::endl;
}

Eigen::Matrix4f getTransformation(std::string parentName, std::string childName){
  parentName.insert(0,"/");
  childName.insert(0,"/");
  tf::StampedTransform transform;
  tf::TransformListener listener;
  ros::Time t = ros::Time(0);
  try{
    listener.waitForTransform(parentName, childName, t, ros::Duration(4.0));
    listener.lookupTransform(parentName, childName, t, transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  tf::Matrix3x3 rot = transform.getBasis();
  tf::Vector3 trans = transform.getOrigin();
  Eigen::Matrix4f tranformation = Eigen::MatrixXf::Identity(4,4);
  Eigen::Matrix3f upleft3x3;
  upleft3x3 << rot[0][0], rot[0][1], rot[0][2],
               rot[1][0], rot[1][1], rot[1][2],
               rot[2][0], rot[2][1], rot[2][2];
  tranformation.block<3,3>(0,0) = upleft3x3;
  tranformation.col(3) = Eigen::Vector4f(trans[0],trans[1],trans[2],1);
  return tranformation;
}