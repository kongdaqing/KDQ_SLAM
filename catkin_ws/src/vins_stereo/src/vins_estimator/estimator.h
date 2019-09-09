#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <queue>

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

#include "../parameters/parameters.h"
#include "../feature/feature.h"
#include <mutex>

using namespace std;
class Estimator
{
public:
    Estimator(Parameters& config_para);
    Feature *feature_manager;
    Parameters config;
    bool sync_images(double& time,cv::Mat& left_image,cv::Mat& right_image);
    mutex image_lock;
    mutex imu_lock;
    queue<pair<double,Eigen::Vector3d>> acc_buf;
    queue<pair<double,Eigen::Vector3d>> gyro_buf;
private:


    queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > > featureBuf;
};

#endif // ESTIMATOR_H
