#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <tf/transform_listener.h>

using std::cout;
using std::endl;
using namespace std;
int main(int argc, char **argv){
 
    ros::init(argc,argv,"JsonSave"); //初始化节点
    ros::NodeHandle n;
    Json::Value root;
    int number = 100;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::Rate rate(10.0);
    while(number > 0)
    {
      try{
           listener.waitForTransform("base_link","vicon/pen/pen",ros::Time(),ros::Duration(4.0));
           listener.lookupTransform("base_link", "vicon/pen/pen",  
                               ros::Time(0), transform);
      }
      catch (tf::TransformException ex){
         ROS_ERROR("%s",ex.what());
         ros::Duration(1.0).sleep();
      }

	root["Ox"].append(transform.getOrigin().x());
	root["Oy"].append(transform.getOrigin().y());
	root["Oz"].append(transform.getOrigin().z());
	root["Rx"].append(transform.getRotation().x());
	root["Ry"].append(transform.getRotation().y());
	root["Rz"].append(transform.getRotation().z());
	root["Rw"].append(transform.getRotation().w());

	number--;
    }

	ofstream os;
	Json::StyledWriter sw;
	os.open("StateInfo");
	os<<sw.write(root);
	os.close();

	return 0;
}

