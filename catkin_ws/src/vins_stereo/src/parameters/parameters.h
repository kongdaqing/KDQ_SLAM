#ifndef PARAMETERS_H
#define PARAMETERS_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <fstream>
using namespace std;
class parameters
{

public:
    parameters(string config_file);
    string imu_topic;
    string left_image_topic;
    string right_image_topic;
};

#endif // PARAMETERS_H
