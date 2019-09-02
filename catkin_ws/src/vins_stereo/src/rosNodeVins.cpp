#include <iostream>
#include <ros/ros.h>
#include "parameters/parameters.h"
using namespace std;



int main(int argc,char** argv)
{
   ros::init(argc,argv,"vins_node");
   ros::NodeHandle n("~");
   ROS_INFO("HELLO STEREO");
   string config_file = argv[1];
   cout << "config_file: " <<  config_file << endl;
   parameters sysSetting(config_file);



   ros::shutdown();
   return 0;
}
