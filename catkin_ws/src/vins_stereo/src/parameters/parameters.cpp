#include "parameters.h"

parameters::parameters(string config_file)
{
FILE *fh = fopen(config_file.c_str(),"r");
if(fh == NULL)
{
    ROS_WARN("config_file doesn't exist!");
    ROS_BREAK();
    return;
}
fclose(fh);
cv::FileStorage fsSettings(config_file,cv::FileStorage::READ);

if(!fsSettings.isOpened())
{
    cerr <<"Wrong path to settings!" << endl;
}
fsSettings["imu_topic"] >> imu_topic;
fsSettings["left_image_topic"] >> left_image_topic;
fsSettings["right_image_topic"] >> right_image_topic;
ROS_INFO("imu topic: %s",imu_topic.c_str());
ROS_INFO("left image topic: %s",left_image_topic.c_str());
ROS_INFO("right image topic: %s",right_image_topic.c_str());
fsSettings.release();



}
