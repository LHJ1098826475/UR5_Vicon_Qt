#include <ros/ros.h>
#include <tf/transform_listener.h>

using std::cout;
using std::endl;

int main(int argc, char** argv){
  ros::init(argc, argv, "basetopaper");

  ros::NodeHandle node;


  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("baselink", "vicon/paper/paper",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

   

    rate.sleep();
  }
  return 0;
};
