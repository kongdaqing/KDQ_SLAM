#ifndef PARAMETERS_H
#define PARAMETERS_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
using namespace std;
class Parameters
{

public:
    Parameters(string config_file);
    string imu_topic;
    string left_image_topic;
    string right_image_topic;
    string track_image_topic;

    int stere_enable;
    int pub_track_image;
    double feature_min_dist;
    int feature_point_max_number;
    bool success_flg;
    double imageInfo_print_hz;
};

#endif // PARAMETERS_H
