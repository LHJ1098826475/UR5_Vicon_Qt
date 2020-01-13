#include <ros/ros.h>
#include <tf/transform_listener.h>

using std::cout;
using std::endl;

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;


  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/world", "vicon/Bar/Bar",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    /*turtlesim::Velocity vel_msg;
    vel_msg.angular = 4.0 * atan2(transform.getOrigin().y(),
                                transform.getOrigin().x());
    vel_msg.linear = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                pow(transform.getOrigin().y(), 2));
    turtle_vel.publish(vel_msg);*/
    cout << "-----------------------" << endl;
    cout << transform.getOrigin().x() << endl;
    cout << transform.getOrigin().y() << endl;
    cout << transform.getOrigin().z() << endl;
    cout << "-----------------------" << endl;
 

    cout << "!!!!!!!!!!!!!!!!!!!!!!!" << endl;
    cout << transform.getRotation().x() << endl;
    cout << transform.getRotation().y() << endl;
    cout << transform.getRotation().z() << endl;
    cout << transform.getRotation().w() << endl;
    cout << "!!!!!!!!!!!!!!!!!!!!!!!" << endl;

    rate.sleep();
  }
  return 0;
};
