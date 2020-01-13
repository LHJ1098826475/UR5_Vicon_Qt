#include <ros/ros.h>
 #include <math.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>//include normal matrix functions

#include <iostream>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>

#define TTY_PATH            "/dev/tty"
#define STTY_US             "stty raw -echo -F "
#define STTY_DEF            "stty -raw echo -F "

using namespace Eigen;
using namespace std;

int main(int argc, char** argv)
{
	system(STTY_US TTY_PATH);
    ros::init(argc, argv, "urTopaper_publish");

    ros::NodeHandle nh;
    ros::Rate rate(10.0);
    tf::TransformListener marker_position_listener;
    static tf::TransformBroadcaster urTopaperBroadcaster;
    MatrixXf paperTovicon = MatrixXf::Zero(40, 4);
   	MatrixXf urTovicon = MatrixXf::Zero(4, 4);
    MatrixXf urTopaper = MatrixXf::Zero(4, 4);
    std::vector<float> rot(3, 0.0f);
    std::vector<float> rot1(3, 0.0f);


  	while(ros::ok)
  	{
    	tf::StampedTransform vicon_ur;
  		tf::StampedTransform vicon_paper;
  		tf::StampedTransform ur_paper;
    	float thetax, thetay, thetaz;
    	tf::Transform urTopaperTransform;
    	tf::Quaternion q;
  		try
  		{
                        marker_position_listener.waitForTransform("base_link","vicon/marker_0/marker_0",ros::Time(),ros::Duration(4.0));
  			marker_position_listener.lookupTransform("base_link", "vicon/marker_0/marker_0", ros::Time(0), vicon_ur);
                        marker_position_listener.waitForTransform("base_link","vicon/paper/paper",ros::Time(),ros::Duration(4.0));
  			marker_position_listener.lookupTransform("base_link", "vicon/paper/paper", ros::Time(0), vicon_paper);
  		}
  		catch (tf::TransformException ex)
	    {
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
			continue;
	    }
    	rot[0] = vicon_ur.getRotation().getW();
    	rot[1] = vicon_ur.getRotation().getX();
    	rot[2] = vicon_ur.getRotation().getY();
	rot[3] = vicon_ur.getRotation().getZ();

    	rot1[0] = vicon_paper.getRotation().getW();
	rot1[1] = vicon_paper.getRotation().getX();
    	rot1[2] = vicon_paper.getRotation().getY();
    	rot1[3] = vicon_paper.getRotation().getZ();

    	paperTovicon(0, 0) = 1 - 2*rot1[1]*rot1[1] - 2*rot1[2]*rot1[2];
		paperTovicon(0, 1) = 2*rot1[0]*rot1[1] - 2*rot1[3]*rot1[2];
		paperTovicon(0, 2) = 2*rot1[0]*rot1[2] + 2*rot1[3]*rot1[1];
		paperTovicon(0, 3) = vicon_ur.getOrigin().x();
		paperTovicon(1, 0) = 2*rot1[0]*rot1[1] + 2*rot1[3]*rot1[2];
		paperTovicon(1, 1) = 1 - 2*rot1[0]*rot1[0] - 2*rot1[2]*rot1[2];
		paperTovicon(1, 2) = 2*rot1[1]*rot1[2] - 2*rot1[3]*rot1[0];
		paperTovicon(1, 3) = vicon_ur.getOrigin().y();
		paperTovicon(2, 0) = 2*rot1[0]*rot1[2] - 2*rot1[3]*rot1[1];
		paperTovicon(2, 1) = 2*rot1[1]*rot1[2] + 2*rot1[3]*rot1[0];
		paperTovicon(2, 2) = 1 - 2*rot1[0]*rot1[0] - 2*rot1[1]*rot1[1];
		paperTovicon(2, 3) = vicon_ur.getOrigin().z();                  
		paperTovicon(3, 3) =1;										//T1(Vicon->Ur_base)


		urTovicon(0, 0) = 1 - 2*rot[1]*rot[1] - 2*rot[2]*rot[2];
		urTovicon(0, 1) = 2*rot[0]*rot[1] - 2*rot[3]*rot[2];
		urTovicon(0, 2) = 2*rot[0]*rot[2] + 2*rot[3]*rot[1];
		urTovicon(0, 3) = vicon_paper.getOrigin().x();
		urTovicon(1, 0) = 2*rot[0]*rot[1] + 2*rot[3]*rot[2];
		urTovicon(1, 1) = 1 - 2*rot[0]*rot[0] - 2*rot[2]*rot[2];
		urTovicon(1, 2) = 2*rot[1]*rot[2] - 2*rot[3]*rot[0];
		urTovicon(0, 3) = vicon_paper.getOrigin().y() + 65;
		urTovicon(2, 0) = 2*rot[0]*rot[2] - 2*rot[3]*rot[1];
		urTovicon(2, 1) = 2*rot[1]*rot[2] + 2*rot[3]*rot[0];
		urTovicon(2, 2) = 1 - 2*rot[0]*rot[0] - 2*rot[1]*rot[1];
		urTovicon(0, 3) = vicon_paper.getOrigin().z();
		urTovicon(3, 3) = 1;                                       //T2(Vicon->paper)

		urTopaper = urTovicon.inverse() * paperTovicon;            //T(Ur_base->paper) = T1.inverse*T2
		thetax = atan2(urTopaper(2, 1), urTopaper(2, 2));
		thetay = atan2(-urTopaper(2, 0), sqrt(urTopaper(2, 1) * urTopaper(2, 1) + urTopaper(2, 2) * urTopaper(2, 2)));
		thetaz = atan2(urTopaper(1, 0), urTopaper(0, 0));
		q.setRPY(thetax, thetay, thetaz);
		urTopaperTransform.setRotation(q);
		urTopaperTransform.setOrigin(tf::Vector3(urTopaper(0, 3), urTopaper(1, 3), urTopaper(2, 3)));
		urTopaperBroadcaster.sendTransform(tf::StampedTransform(urTopaperTransform, ros::Time::now(), "base_link", "vicon/paper/paper")); //send to TF
    }
    return 0;
}
