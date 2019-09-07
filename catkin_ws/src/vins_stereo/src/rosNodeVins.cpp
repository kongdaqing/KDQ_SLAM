#include <iostream>
#include <ros/ros.h>
#include "parameters/parameters.h"
#include "rosVinsInterface.h"

using namespace std;



int main(int argc,char** argv)
{
   ros::init(argc,argv,"vins_node");
   ros::NodeHandle n("~");
   ROS_INFO("HELLO STEREO");
   if(argc == 1)
   {
       ROS_ERROR("Error: No vins-stereo config file,please allocate config file!");
       ros::shutdown();
       return -1;
   }
   string config_file = argv[1];
   ROS_INFO("config_file: %s", config_file.c_str());
   Parameters sysSetting(config_file);
   if(sysSetting.success_flg == false)
   {
       ROS_ERROR("Error: %s doesn't exist,please input right config file!",config_file.c_str());
       ros::shutdown();
       return -1;
   }
   rosVinsInterface ros_vins(sysSetting);

   ros::spin();
   return 0;
}
