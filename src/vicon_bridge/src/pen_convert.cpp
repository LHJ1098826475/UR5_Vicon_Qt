#include <ros/ros.h>
 #include <math.h>
#include <jsoncpp/json/json.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>//include normal matrix functions

#include <iostream>
#include <fstream>
#include <std_msgs/String.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#define TTY_PATH            "/dev/tty"
#define STTY_US             "stty raw -echo -F "
#define STTY_DEF            "stty -raw echo -F "

using namespace Eigen;
using namespace std;

int main(int argc, char** argv)
{
	//system(STTY_US TTY_PATH);
    ros::init(argc, argv, "urTopen_publish");

    ros::NodeHandle nh;
    ros::Rate rate(50.0);
    tf::TransformListener marker_position_listener;
    ros::Publisher penp =  nh.advertise<std_msgs::String>("penp", 1);
    std_msgs::String pmsg;
    std::stringstream ss;
    static tf::TransformBroadcaster urTopenBroadcaster;
    MatrixXf penTovicon = MatrixXf::Zero(4, 4);
    MatrixXf urTovicon = MatrixXf::Zero(4, 4);
    MatrixXf urTopen = MatrixXf::Zero(4, 4);
    std::vector<float> rot(4, 0.0f);
    std::vector<float> rot1(4, 0.0f);
    std::vector<float> old_state(2, 0.0f);
    std::vector<float> now_state(2, 0.0f);

    

    Json::Value root;
    int number = 150,count_end=-5,start_flag=1,count_start=3,flag_action=0;
    tf::TransformListener listener;
    tf::StampedTransform transform;

    cout<<"  Ready to pick pen ! "<<endl;
    pmsg.data = " Ready to pick pen ! ";
    int count_num = 15;
    while(count_num)
    {
      if(ros::ok())
      {
        penp.publish(pmsg);
      }
      rate.sleep();
      count_num--;
    }


    while(count_start > -3)
    {
      if(ros::ok)
      {
        try{
           listener.waitForTransform("base","vicon/pen/pen",ros::Time(),ros::Duration(4.0));
           listener.lookupTransform("base", "vicon/pen/pen",  
                               ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
           ROS_ERROR("%s",ex.what());
           ros::Duration(1.0).sleep();
        }

        now_state[0] = transform.getOrigin().x();
	now_state[1] = transform.getOrigin().y();
	if(flag_action == 0)
	{
	  if((fabs(now_state[1]-old_state[1])<0.01) && (fabs(now_state[0]-old_state[0])<0.01))
	  {
	   count_start=3;
	  }
	  else
	  {
	   count_start--;
	  }
	}
	else
	{
	  if((fabs(now_state[1]-old_state[1])<0.002)&&(fabs(now_state[0]-old_state[0])<0.002))
	  {
	   count_start--;
	  }
	  else
	  {
	   count_start=0;
	  }
	}
	
	if(count_start == 0)  flag_action=1;
      }

      cout<<" count_start "<<count_start<<"  yy !! "<<fabs(now_state[1]-old_state[1])<<" xx !! "<<fabs(now_state[0]-old_state[0])<<endl;
      old_state[0]=now_state[0];
      old_state[1]=now_state[1];
      rate.sleep();
    }

    cout<<"  Wait for 3S ! "<<endl;
    pmsg.data = " Wait for 3S !";
    penp.publish(pmsg);
    while(number--)
    {
      rate.sleep();
    }
    cout<<"  Go to Cpature !! "<<endl;
    pmsg.data = " Go to Cpature !!";
    penp.publish(pmsg);
    old_state[0]=now_state[0];
    old_state[1]=now_state[1];

    count_start = 0;
    while(count_start > -2)
    {
      if(ros::ok)
      {
        try{
           listener.waitForTransform("base","vicon/pen/pen",ros::Time(),ros::Duration(4.0));
           listener.lookupTransform("base", "vicon/pen/pen",  
                               ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
           ROS_ERROR("%s",ex.what());
           ros::Duration(1.0).sleep();
        }

        now_state[0] = transform.getOrigin().x();
	now_state[1] = transform.getOrigin().y();

	if((fabs(now_state[1]-old_state[1]) > 0.002)||(fabs(now_state[0]-old_state[0]) > 0.002))
	{
	   count_start--;
	  }
	  else
	  {
	   count_start=0;
	  }
      }
      old_state[0]=now_state[0];
      old_state[1]=now_state[1];
      rate.sleep();

      
    }

    cout<<"  Capturing !! "<<endl;
    pmsg.data = " Cpaturing !!";
    penp.publish(pmsg);
    while(count_end<0)
    {
      if(ros::ok)
      {
    	tf::StampedTransform vicon_ur;
  	tf::StampedTransform vicon_pen;
  	tf::StampedTransform ur_paper;
    	float thetax, thetay, thetaz;
    	tf::Transform urTopenTransform;
    	tf::Quaternion q;
  		try
  		{
                        marker_position_listener.waitForTransform("vicon_root","base_link",ros::Time(),ros::Duration(4.0));
  			marker_position_listener.lookupTransform("vicon_root", "base_link", ros::Time(0), vicon_ur);
                        marker_position_listener.waitForTransform("vicon_root","vicon/pen/pen",ros::Time(),ros::Duration(4.0));
  			marker_position_listener.lookupTransform("vicon_root", "vicon/pen/pen", ros::Time(0), vicon_pen);
  		}
  		catch (tf::TransformException ex)
	    {
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
			continue;
	    }
Eigen::Quaterniond erot = Eigen::Quaterniond(vicon_ur.getRotation().getW(), 
					    vicon_ur.getRotation().getX(),
					    vicon_ur.getRotation().getY(),
					    vicon_ur.getRotation().getZ()).normalized();
Eigen::Quaterniond erot1 = Eigen::Quaterniond(vicon_pen.getRotation().getW(), 
					    vicon_pen.getRotation().getX(),
					    vicon_pen.getRotation().getY(),
					    vicon_pen.getRotation().getZ()).normalized();
Eigen::Isometry3d eurTovicon = Eigen::Isometry3d::Identity();
eurTovicon.rotate(erot.toRotationMatrix());
eurTovicon.pretranslate(Eigen::Vector3d(vicon_ur.getOrigin().x(),
					vicon_ur.getOrigin().y(),
					vicon_ur.getOrigin().z()));

Eigen::Isometry3d epenTovicon = Eigen::Isometry3d::Identity();
epenTovicon.rotate(erot1.toRotationMatrix());
epenTovicon.pretranslate(Eigen::Vector3d(vicon_pen.getOrigin().x(),
					vicon_pen.getOrigin().y(),
					vicon_pen.getOrigin().z()));
if((eurTovicon.inverse().matrix().cols()==4)&&(eurTovicon.inverse().matrix().rows()==4))
		{
		  Eigen::Isometry3d  epentour = eurTovicon.inverse() * epenTovicon;
		  if(number == 1)
		  {
			cout<<" Eigen urvicon !!  "<<epentour.matrix()<<endl;
		  }
		
		//urTopen = epentour.matrix();
		thetax = atan2(epentour.matrix()(2, 1), epentour.matrix()(2, 2));
		thetay = atan2(-epentour.matrix()(2, 0), sqrt(epentour.matrix()(2, 1) * epentour.matrix()(2, 1) + epentour.matrix()(2, 2) * epentour.matrix()(2, 2)));
		thetaz = atan2(epentour.matrix()(1, 0), epentour.matrix()(0, 0));
		q.setRPY(thetax, thetay, thetaz);
		urTopenTransform.setRotation(q);
		urTopenTransform.setOrigin(tf::Vector3(epentour.matrix()(0, 3), epentour.matrix()(1, 3), epentour.matrix()(2, 3)));
		urTopenBroadcaster.sendTransform(tf::StampedTransform(urTopenTransform, ros::Time::now(), "base_link", "vicon/pen/pen")); //send to TF
		}





    	/*rot[0] = vicon_ur.getRotation().getW();
    	rot[1] = vicon_ur.getRotation().getX();
    	rot[2] = vicon_ur.getRotation().getY();
	rot[3] = vicon_ur.getRotation().getZ();

    	rot1[0] = vicon_pen.getRotation().getW();
	rot1[1] = vicon_pen.getRotation().getX();
    	rot1[2] = vicon_pen.getRotation().getY();
    	rot1[3] = vicon_pen.getRotation().getZ();
//cout<<" rot "<<rot[0]<<rot[1]<<rot[2]<<rot[3]<<rot.back()<<rot.front()<<rot.size()<<endl;
    	penTovicon(0, 0) = 1 - 2*rot1[1]*rot1[1] - 2*rot1[2]*rot1[2];
		penTovicon(0, 1) = 2*rot1[0]*rot1[1] - 2*rot1[3]*rot1[2];
		penTovicon(0, 2) = 2*rot1[0]*rot1[2] + 2*rot1[3]*rot1[1];
		penTovicon(0, 3) = vicon_pen.getOrigin().x();
		penTovicon(1, 0) = 2*rot1[0]*rot1[1] + 2*rot1[3]*rot1[2];
		penTovicon(1, 1) = 1 - 2*rot1[0]*rot1[0] - 2*rot1[2]*rot1[2];
		penTovicon(1, 2) = 2*rot1[1]*rot1[2] - 2*rot1[3]*rot1[0];
		penTovicon(1, 3) = vicon_pen.getOrigin().y();
		penTovicon(2, 0) = 2*rot1[0]*rot1[2] - 2*rot1[3]*rot1[1];
		penTovicon(2, 1) = 2*rot1[1]*rot1[2] + 2*rot1[3]*rot1[0];
		penTovicon(2, 2) = 1 - 2*rot1[0]*rot1[0] - 2*rot1[1]*rot1[1];
		penTovicon(2, 3) = vicon_pen.getOrigin().z();                  
		penTovicon(3, 3) = 1;        //T1(Vicon->Ur_base)


		urTovicon(0, 0) = 1 - 2*rot[1]*rot[1] - 2*rot[2]*rot[2];
		urTovicon(0, 1) = 2*rot[0]*rot[1] - 2*rot[3]*rot[2];
		urTovicon(0, 2) = 2*rot[0]*rot[2] + 2*rot[3]*rot[1];
		urTovicon(0, 3) = vicon_ur.getOrigin().x();
		urTovicon(1, 0) = 2*rot[0]*rot[1] + 2*rot[3]*rot[2];
		urTovicon(1, 1) = 1 - 2*rot[0]*rot[0] - 2*rot[2]*rot[2];
		urTovicon(1, 2) = 2*rot[1]*rot[2] - 2*rot[3]*rot[0];
		urTovicon(1, 3) = vicon_ur.getOrigin().y() + 65;
		urTovicon(2, 0) = 2*rot[0]*rot[2] - 2*rot[3]*rot[1];
		urTovicon(2, 1) = 2*rot[1]*rot[2] + 2*rot[3]*rot[0];
		urTovicon(2, 2) = 1 - 2*rot[0]*rot[0] - 2*rot[1]*rot[1];
		urTovicon(2, 3) = vicon_ur.getOrigin().z();
		urTovicon(3, 3) = 1;          //T2(Vicon->paper)
		if((urTovicon.inverse().cols()==4)&&(urTovicon.inverse().rows()==4))
		{
		  urTopen = urTovicon.inverse() * penTovicon;            //T(Ur_base->paper) = T1.inverse*T2
cout<<"urpen!!   "<<urTopen<<endl;
		thetax = atan2(urTopen(2, 1), urTopen(2, 2));
		thetay = atan2(-urTopen(2, 0), sqrt(urTopen(2, 1) * urTopen(2, 1) + urTopen(2, 2) * urTopen(2, 2)));
		thetaz = atan2(urTopen(1, 0), urTopen(0, 0));
		q.setRPY(thetax, thetay, thetaz);
		urTopenTransform.setRotation(q);
		urTopenTransform.setOrigin(tf::Vector3(urTopen(0, 3), urTopen(1, 3), urTopen(2, 3)));
		urTopenBroadcaster.sendTransform(tf::StampedTransform(urTopenTransform, ros::Time::now(), "base_link", "vicon/pen/pen")); //send to TF
		}*/


	try{
           listener.waitForTransform("base","vicon/pen/pen",ros::Time(),ros::Duration(4.0));
           listener.lookupTransform("base", "vicon/pen/pen",  
                               ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
           ROS_ERROR("%s",ex.what());
           ros::Duration(1.0).sleep();
        }

	root["Ox"].append(transform.getOrigin().x());
	root["Oy"].append(transform.getOrigin().y());
	root["Oz"].append(transform.getOrigin().z());
        tf::Quaternion qq = transform.getRotation();
        qq = qq.normalized();
	root["Rx"].append(qq.x());
	root["Ry"].append(qq.y());
	root["Rz"].append(qq.z());
	root["Rw"].append(qq.w());
	cout<<" write !! "<<endl;
	now_state[0] = transform.getOrigin().x();
	now_state[1] = transform.getOrigin().y();

	if((fabs(now_state[1]-old_state[1])<0.0005)&&(fabs(now_state[0]-old_state[0])<0.0005))
	{
	   count_end++;
	}
	else
	{
	   count_end=-5;
	}
	old_state[0]=now_state[0];
	old_state[1]=now_state[1];
	}
	rate.sleep();	
	
    }
	ofstream os;
	Json::StyledWriter sw;
  os.open("/home/lhj/catkin_qt/StateInfo");
	os<<sw.write(root);
	os.close();
	cout<<" will end ";	
  pmsg.data = " end !!";
  penp.publish(pmsg);
    return 0;
}
