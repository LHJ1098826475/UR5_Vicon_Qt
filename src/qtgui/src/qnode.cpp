/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <sensor_msgs/JointState.h>
#include "../include/qtgui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtgui {

/*****************************************************************************
** Implementation
*****************************************************************************/
std::string Vicon = " Waiting for connect ";
std::string Ur5 = " Waiting for connect  ";
std::string Capture = " Waiting for capture ";
std::vector<double> joint_angles(6);
std::vector<double> joint_t_angles(6);
std::vector<double> joint_speed(6);

Json::Value save_root;
Json::Reader reader;
uint joint_flag = 0;
bool read_flag = false;
QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

void conncallback(std_msgs::String chatter)
{
  Vicon = chatter.data;
}

void ur5callback(std_msgs::String ur5)
{
  Ur5 = ur5.data;
}

void penpcallback(std_msgs::String penp)
{
  Capture = penp.data;
}

void jointtcallback(const sensor_msgs::JointState& joint_t)
{
  if(joint_flag == 1)
  {
    joint_t_angles.clear();
    for(uint i = 0; i <6; i++)
        {
           joint_t_angles.push_back(joint_t.position[i]);

        }
  }

}

void jointStateCallback(const sensor_msgs::JointState& joint)
{
  if(joint_flag == 1)
  {
    joint_angles.clear();
    joint_speed.clear();
    for(uint i = 0; i <6; i++)
        {
           joint_angles.push_back(joint.position[i]);
        }
  }

}

bool QNode::init() {
	ros::init(init_argc,init_argv,"qtgui");
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
  chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  joint_subscriber = n.subscribe("/joint_states",100,jointStateCallback);
  Vicon_subscriber = n.subscribe("viconp",1,conncallback);
  ur5_subscriber = n.subscribe("ur5",1,ur5callback);
  pen_subscriber = n.subscribe("penp",1,penpcallback);
  joint_t_subscriber = n.subscribe("joint_tar",1,jointtcallback);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"qtgui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	start();
	return true;
}



void QNode::run() {
  ros::Rate loop_rate(20);
  int count = 0;
  while ( ros::ok() ) {
    if((joint_flag == 2) && (!read_flag))
    {
      std::ifstream is;
      is.open("/home/lhj/catkin_ws/SpeedInfo",std::ios::binary);
      if(reader.parse(is,save_root))
      {
        read_flag = true;
        count = 0;
      }
    }

    if((count >= 0) && (count <= (save_root["org_state1"].size()-1)) && (read_flag))
    {
      if(save_root["org_state1"][count].asString() == " one point end ")
        count++;
      if(count <= (save_root["org_state1"].size()-1))
      {
        for(uint j=0;j<6;j++)
        {
          switch(j)
          {
        case 0:
            joint_t_angles[j] = save_root["org_state1"][count].asFloat();
            joint_angles[j] = save_root["state1"][count].asFloat();
            break;
        case 1:
            joint_t_angles[j] = save_root["org_state2"][count].asFloat();
            joint_angles[j] = save_root["state2"][count].asFloat();
            break;
        case 2:
            joint_t_angles[j] = save_root["org_state3"][count].asFloat();
            joint_angles[j] = save_root["state3"][count].asFloat();
            break;
        case 3:
            joint_t_angles[j] = save_root["org_state4"][count].asFloat();
            joint_angles[j] = save_root["state4"][count].asFloat();
            break;
        case 4:
            joint_t_angles[j] = save_root["org_state5"][count].asFloat();
            joint_angles[j] = save_root["state5"][count].asFloat();
            break;
        case 5:
            joint_t_angles[j] = save_root["org_state6"][count].asFloat();
            joint_angles[j] = save_root["state6"][count].asFloat();
            break;
          }

        }

      }
      count++;
    }

    if((count == (save_root["org_state1"].size()-1)) && (read_flag))
    {
      joint_flag = 0;
      read_flag = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace qtgui
