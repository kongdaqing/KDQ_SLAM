#ifndef ESTIMATOR_H
#define ESTIMATOR_H
#include <mutex>
#include <queue>
#include <thread>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "../parameters/parameters.h"
#include "../feature/feature.h"
using namespace std;
class Estimator
{
public:
    Estimator(Parameters& config_para);
    Feature *feature_manager;
    Parameters config;
    void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
    void image_left_callback(const sensor_msgs::ImageConstPtr &image_msg);
    void image_right_callback(const sensor_msgs::ImageConstPtr &image_msg);
    void vins_process_process();
    bool sync_images(double& time,cv::Mat& left_image,cv::Mat& right_image);
    cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);
protected:
    ros::NodeHandle node;
    ros::Subscriber imu_subscriber;
    ros::Subscriber image_left_subscriber;
    ros::Subscriber image_right_subscriber;
    mutex image_lock;
    mutex imu_lock;

private:
    queue<pair<double,Eigen::Vector3d>> acc_buf;
    queue<pair<double,Eigen::Vector3d>> gyro_buf;
    queue<sensor_msgs::ImageConstPtr> img_left_buf;
    queue<sensor_msgs::ImageConstPtr> img_right_buf;
    queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > > featureBuf;
};

#endif // ESTIMATOR_H
