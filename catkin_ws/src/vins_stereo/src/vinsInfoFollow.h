#ifndef VINSINFOFOLLOW_H
#define VINSINFOFOLLOW_H
#include "vinsDebugInfo.h"
#include "ros/ros.h"

class VinsInfoFollow
{
public:
    VinsInfoFollow(double time_now,double imgInfo_print_hz);
    void printImageInfo(double time_now);
    ImageDebugInfo img_info;
    double para_imgInfo_print_period;
private:
    double pre_print_time;
};

#endif // VINSINFOFOLLOW_H
