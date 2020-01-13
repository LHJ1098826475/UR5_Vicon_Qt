#include <ros/ros.h>

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


//string track_save_path = "home/catkin_ws/";

bool getPath(MatrixXf& track_path, tf::TransformListener& marker_position_listener){
    tf::StampedTransform path_transform;
    try
    {
        //pencil_position_listener.lookupTransform("world", "vicon/marker_5", ros::Time(0), path_transform);   modify by xu
        marker_position_listener.lookupTransform("world", "vicon/marker_5/marker_5", ros::Time(0), path_transform);
    }

    catch (tf::TransformException ex)
    {
        return false;
    }
    track_path(0) = path_transform.getOrigin().x();
    track_path(1) = path_transform.getOrigin().y();
    track_path(2) = path_transform.getOrigin().z();
    //cout << 'X:' << track_path[0] << 'Y:' << track_path[1] << 'Z:' << track_path[2] << endl;  modify by xu
    cout << "X:" << track_path(0) << "Y:"<< track_path(1) << "Z:" << track_path(2) << endl;
    //ofstream outStream(string track_save_path.c_str());  modify by xu
    return true;
}
bool savePath(fstream& outStream, tf::TransformListener& marker_position_listener){

    //ofstream outStream(filename.c_str());
    //ofstream outStream(filename);
 
    tf::StampedTransform path_transform;
    try
    {
        marker_position_listener.lookupTransform("world", "vicon/marker_5/marker_5", ros::Time(0), path_transform);
    }

    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }
    //if(!outstream)
   
    outStream << "the error of pencil track and  ee_link" << endl;
    outStream << path_transform.getOrigin().x() << endl;
    outStream << path_transform.getOrigin().y() << endl;
    outStream << path_transform.getOrigin().z() << endl;

    outStream << "rotation" << endl;
    outStream << path_transform.getRotation().getW() << endl;
    outStream << path_transform.getRotation().getX() << endl;
    outStream << path_transform.getRotation().getY() << endl;
    outStream << path_transform.getRotation().getZ() << endl;
//}
  
    return true;
}




int main(int argc, char** argv)
{
    int i = 0;
    //static string solution_save_path = "/home/track/";
    const char* filename = "/home/wjx/data.txt";
    ofstream newFile(filename);
    newFile<<"track data";
    newFile.close();
    system(STTY_US TTY_PATH);
    ros::init(argc, argv, "pencil_track");

    ros::NodeHandle nh;
    ros::Rate rate(10.0);

    tf::TransformListener marker_position_listener;

    //modify by xu
    MatrixXf track_path(3,1);
    track_path= MatrixXf::Zero(3,1);

    //MatrixXf track_path = Matrix::zeros(3, 1);  modify by xu
fstream outstream(filename, ios::ate | ios::out | ios::in);
    while(ros::ok())
        //while (nh.ok)  modify by xu
    {
        
        //tf:StampedTransform path_transform; modify by xu

        getPath(track_path, marker_position_listener);
        savePath(outstream, marker_position_listener);
   
  //      outStream<<"hello\n";



        if(i > 100)
        {
            break;
        }
        i++;
        rate.sleep();
    }
  outstream.close();
    return 0;
}
