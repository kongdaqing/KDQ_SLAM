#include "parameters.h"

Parameters::Parameters(string config_file)
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL)
    {
        success_flg = false;
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
    fsSettings["track_image_topic"] >> track_image_topic;
    stere_enable = fsSettings["stereo_enable"];
    feature_point_max_number = fsSettings["feature_point_max_number"];
    feature_min_dist = fsSettings["feature_min_dist"];
    pub_track_image = fsSettings["pub_track_image"];
    imageInfo_print_hz = fsSettings["imageInfo_print_hz"];
    success_flg = true;

    fsSettings.release();



}
