#ifndef ROSVINSINTERFACE_H
#define ROSVINSINTERFACE_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "parameters/parameters.h"
#include "vins_estimator/estimator.h"
#include "vinsInfoFollow.h"
#include <thread>


class rosVinsInterface
{
public:
    rosVinsInterface(Parameters& paras);
    void showConfig();
    void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
    void leftImage_callback(const sensor_msgs::ImageConstPtr &image_msg);
    void rightImage_callback(const sensor_msgs::ImageConstPtr &image_msg);
    cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);
    bool syncImages(double &time, cv::Mat &left_image, cv::Mat &right_image);
    void vinsProcess();
    void pubTrackImage(const cv::Mat &imgTrack, const double t);
protected:
    ros::NodeHandle node;
    ros::Subscriber imu_subscriber;
    ros::Subscriber image_left_subscriber;
    ros::Subscriber image_right_subscriber;
    ros::Publisher  track_image_publisher;
private:
    Estimator* vins_sys;
    Parameters config;
    VinsInfoFollow *vinsInfo_record;
    queue<sensor_msgs::ImageConstPtr> img_left_buf;
    queue<sensor_msgs::ImageConstPtr> img_right_buf;
    thread vins_thread;
};

#endif // ROSVINSINTERFACE_H
