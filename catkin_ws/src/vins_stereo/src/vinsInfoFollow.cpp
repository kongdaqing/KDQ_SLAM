#include "vinsInfoFollow.h"

VinsInfoFollow::VinsInfoFollow(double time_now,double imgInfo_print_hz)
    : pre_print_time(time_now)
{
 para_imgInfo_print_period = 1/imgInfo_print_hz;
}
void VinsInfoFollow::printImageInfo(double time_now)
{
    if(time_now - pre_print_time < para_imgInfo_print_period)
        return;
    pre_print_time = time_now;
    ROS_INFO("Current left image index: %d", img_info.leftImage_index);
    ROS_INFO("Current right image index: %d", img_info.rightImage_index);
    ROS_INFO("Current track image index: %d", img_info.trackImage_index);
}
