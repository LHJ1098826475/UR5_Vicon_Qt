/********************************************************************************
Copyright (c) 2016, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, 
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software 
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <tf_conversions/tf_kdl.h>
#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

using namespace Eigen;
using std::cout;
using std::endl;
using namespace std;

std::vector<double> joint_angles(6);
std::vector<double> joint_speed(6);
KDL::JntArray nominal(6);
KDL::JntArray diff_delta(6);

double fRand(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

void jointStateCallback(const sensor_msgs::JointState& joint_state)
{
    joint_angles.clear();
    joint_speed.clear();
    for(uint i = 0; i <6; i++)
        {
                joint_angles.push_back(joint_state.position[i]);
		joint_speed.push_back(joint_state.velocity[i]);
		
        }
}



int main(int argc, char** argv)
{

  srand(1);
  ros::init(argc, argv, "ik_tests");
  ros::NodeHandle nh("~");
  ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 100, jointStateCallback);
  ros::Publisher ur_cmd_publisher = nh.advertise<std_msgs::String>("/ur_driver/URScript", 1000);
  ros::Publisher jointp = nh.advertise<sensor_msgs::JointState>("/joint_tar", 1);
  
  KDL::JntArray result;

  ros::spinOnce();
  ros::Rate rate(20);

  Json::Value root;
  Json::Value save_root;
  Json::Reader reader;

  tf::StampedTransform transform; 
  std::vector<float> rott(4, 0.0f);
  std::string chain_start, chain_end, urdf_param;
  std_msgs::String cmd_msg;
  double timeout;
  double eps = 1e-5;
  double Kp = 0.3;
  double Td = 0.5;
  double delta = 1.0;
  double max_speed = 2,nominal_3 = 0.0,nominal_7 = 0.0,min_speed = 0.0;
  int rc,num_samples,count=1;
//Kp=0.3;Td=2.5;max_speed=0.2
  
  
  nh.param("num_samples", num_samples, 1);
  nh.param("chain_start", chain_start, std::string(""));
  nh.param("chain_end", chain_end, std::string(""));
  
  if (chain_start=="" || chain_end=="") {
    ROS_FATAL("Missing chain info in launch file");
    exit (-1);
  }

  nh.param("timeout", timeout, 0.005);
  nh.param("urdf_param", urdf_param, std::string("/robot_description"));
  nh.param("Kp", Kp, 0.3);
  nh.param("Td", Td, 0.5);
  ROS_INFO("Kp:%f, Td:%f", Kp, Td);
  if (num_samples < 1)
    num_samples = 1;

  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

  sensor_msgs::JointState joint_msg;
  joint_msg.header.stamp = ros::Time::now();
  joint_msg.position =std::vector<double>(6,0.0);

  int joint_state_wait = 0;
  while((joint_angles[0] == 0.0) && (joint_angles[1] == 0.0))
  {
    ros::spinOnce();
    joint_msg.position[0] = 1.00;
    jointp.publish(joint_msg);
    joint_state_wait++;
  }
  cout<<" joint_state_wait is "<<joint_state_wait<<endl;
     
  
  ifstream is;
  is.open("/home/lhj/catkin_qt/StateInfo",ios::binary);
  if(reader.parse(is,root))
  {
    ROS_INFO_STREAM(" number is "<<root["Ox"].size());
    for(int i = 0; i < root["Ox"].size(); i++)
	{	
	  transform.setOrigin(tf::Vector3(root["Ox"][i].asFloat(),root["Oy"][i].asFloat(),root["Oz"][i].asFloat()));
	  transform.setRotation(tf::Quaternion(root["Rx"][i].asFloat(),root["Ry"][i].asFloat(),root["Rz"][i].asFloat(),root["Rw"][i].asFloat()));

  	  Eigen::Quaterniond erot = Eigen::Quaterniond(transform.getRotation().getW(), 
					    transform.getRotation().getX(),
					    transform.getRotation().getY(),
					    transform.getRotation().getZ()).normalized();
          
	  
	  if(i == (root["Ox"].size()-1))
	  {
	    Eigen::Isometry3d eurTovicon = Eigen::Isometry3d::Identity();
	    eurTovicon.rotate(erot.toRotationMatrix());
	    cout<<" Eigen Rotation !!  "<<eurTovicon.matrix()<<endl;
	  }
	  diff_delta(0) = 1.0;
//0  0.05; 4   0.035;  5  0.035
	  
	     ROS_INFO_STREAM(" fabs(diff_delta) is "<<fabs(diff_delta(0))<<endl);
	     for (uint j=0; j<6; j++) 
	     {

		nominal(j) = joint_angles[j];
	     } 
	     KDL::Frame cartpos;
	     TransformTFToKDL(transform,cartpos);
	     //KDL::Frame cartpos = KDL::Frame(rot,vact);
	     //Eigen::Isometry3d epenTovicon;
	     //tansformKDLToEigen(cartpos,epenTovicon);

	     rc = tracik_solver.CartToJnt(nominal,cartpos,result);				
	     if (rc>=0)
	     {
      for (uint j=0; j<6; j++)
      {
        joint_msg.position[j] = result(j);
        ROS_INFO_STREAM(count<<" TRAC-IK result " <<j<< " is "<<result(j)<<" !!");
      }
      jointp.publish(joint_msg);


		while((fabs(diff_delta(0)) > 0.01) || (fabs(diff_delta(4)) > 0.01) || (fabs(diff_delta(5)) > 0.01))
	  	{
		  if(ros::ok)
	          {

			  if(i > 1)
				Kp = Kp + 1;
        		  if(Kp > 5)
				Kp = 5;
			  std::vector<double> cmd_vector;
			  for(uint j=0; j<6; j++)
			  {
			    nominal(j) = joint_angles[j];
			    delta = result(j) - nominal(j);
			    diff_delta(j) = delta;
			    if((fabs(delta) < 0.015) && (i>1))
			    {
				Kp = 8;
				Td = 0;
			    }
				
			    double speed = Kp * delta - Td * joint_speed[j];
			    if (speed > max_speed)
			       speed = max_speed;
			    else if((speed < min_speed) && (speed >= 0))
				     speed = min_speed;
				 else if (speed < -max_speed)
			                 speed = -max_speed;
				      else if((speed > -min_speed) && (speed < 0))
				     		speed = -min_speed;
			    switch(j)
			    {
				case 0:
				    save_root["speed1"].append(speed);
				    save_root["state1"].append(nominal(j));
				    save_root["org_state1"].append(result(j));
				    //ROS_INFO_STREAM(" joint_speed is "<<joint_speed[j]<<endl);
				    break;
				case 1:
				    save_root["speed2"].append(speed);
				    save_root["state2"].append(nominal(j));
				    save_root["org_state2"].append(result(j));
				    //ROS_INFO_STREAM(" joint_speed is "<<joint_speed[j]<<endl);
				    break;
				case 2:
				    save_root["speed3"].append(speed);
				    save_root["state3"].append(nominal(j));
				    save_root["org_state3"].append(result(j));
				    //ROS_INFO_STREAM(" joint_speed is "<<joint_speed[j]<<endl);
			 	    break;
				case 3:
				    save_root["speed4"].append(speed);
				    save_root["state4"].append(nominal(j));
				    save_root["org_state4"].append(result(j));
				    //ROS_INFO_STREAM(" joint_speed is "<<joint_speed[j]<<endl);
				    break;
				case 4:
				    save_root["speed5"].append(speed);
				    save_root["state5"].append(nominal(j));
				    save_root["org_state5"].append(result(j));
				    //ROS_INFO_STREAM(" joint_speed is "<<joint_speed[j]<<endl);
				    break;
				case 5:
				    save_root["speed6"].append(speed);
				    save_root["state6"].append(nominal(j));
				    save_root["org_state6"].append(result(j));
				    //ROS_INFO_STREAM(" joint_speed is "<<joint_speed[j]<<endl);
				    break;
			    }
          cmd_vector.push_back(speed);


			  }
			  char cmd[100];
			  sprintf(cmd, "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], 1.0 , 0.05)", cmd_vector[0], cmd_vector[1], cmd_vector[2], cmd_vector[3], cmd_vector[4], cmd_vector[5]);
			//sprintf(cmd, "movej([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f])", result(0), result(1), result(2), result(3),  result(4),  result(5));
			  cmd_msg.data = cmd;	
			  ur_cmd_publisher.publish(cmd_msg);
		  }//ros::ok
		  ros::spinOnce();
		  //sleep(1.0);
		  rate.sleep();		
		}//while循环直到达到该点精度范围
	      }//if逆解成功

	  save_root["speed1"].append(" one point end ");
	  save_root["speed2"].append(" one point end ");
	  save_root["speed3"].append(" one point end ");
	  save_root["speed4"].append(" one point end ");
	  save_root["speed5"].append(" one point end ");
	  save_root["speed6"].append(" one point end ");

	  save_root["state1"].append(" one point end ");
	  save_root["state2"].append(" one point end ");
	  save_root["state3"].append(" one point end ");
	  save_root["state4"].append(" one point end ");
	  save_root["state5"].append(" one point end ");
	  save_root["state6"].append(" one point end ");

	  save_root["org_state1"].append(" one point end ");
	  save_root["org_state2"].append(" one point end ");
	  save_root["org_state3"].append(" one point end ");
	  save_root["org_state4"].append(" one point end ");
	  save_root["org_state5"].append(" one point end ");
	  save_root["org_state6"].append(" one point end ");
	  count++;
	  Kp = 1.0;
	  //min_speed = 0.0009;
	  //max_speed = 0.009;
 	  Td = -0.5;
      }//for循环轨迹点    
  }//读取文件并解析json数据成功

  is.close();
  is.clear(ios::goodbit);


  ofstream os;
  Json::StyledWriter sw;
  os.open("/home/lhj/catkin_qt/SpeedInfo");
  os<<sw.write(save_root);
  os.close();

  return 0;
}

