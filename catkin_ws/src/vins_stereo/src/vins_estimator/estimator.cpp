#include "estimator.h"

Estimator::Estimator(Parameters& config_para):config(config_para)
{
    feature_manager = new Feature(config);
    imu_subscriber = node.subscribe(config.imu_topic,1000,&Estimator::imu_callback,this);
    image_left_subscriber = node.subscribe(config.left_image_topic,200,&Estimator::image_left_callback,this);
    image_right_subscriber = node.subscribe(config.right_image_topic,200,&Estimator::image_right_callback,this);
    thread vins_thread = thread{vins_process_process};
}
void Estimator::imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    imu_lock.lock();
    Eigen::Vector3d acc,gyro;
    acc << imu_msg->linear_acceleration.x,imu_msg->linear_acceleration.y,imu_msg->linear_acceleration.z;
    gyro << imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z;
    double t = imu_msg->header.stamp.toSec();
    acc_buf.push(make_pair(t,acc));
    gyro_buf.push(make_pair(t,gyro));
    imu_lock.unlock();
}
void Estimator::image_left_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    image_lock.lock();
    img_left_buf.push(image_msg);
    image_lock.unlock();
}
void Estimator::image_right_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    image_lock.lock();
    img_right_buf.push(image_msg);
    image_lock.unlock();
}

cv::Mat Estimator::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}
bool Estimator::sync_images(double &time, cv::Mat &left_image, cv::Mat &right_image)
{

    if(img_left_buf.empty() || img_right_buf.empty())
        return false;
    image_lock.lock();
    double time_left,time_right;
    time_left = img_left_buf.front()->header.stamp.toSec();
    time_right = img_right_buf.front()->header.stamp.toSec();
    if((time_left - time_right) > 0.003f)
    {
        img_right_buf.pop();
        ROS_WARN("throw right image!");
    }else if((time_right - time_left) > 0.003f){
        img_left_buf.pop();
        ROS_WARN("throw left image!");
    }else {
        time = time_left;
        left_image = getImageFromMsg(img_left_buf.front());
        right_image = getImageFromMsg(img_right_buf.front());
        img_left_buf.pop();
        img_right_buf.pop();
    }
    image_lock.unlock();
    if(left_image.empty() || right_image.empty())
        return false;
    else
        return true;
}

void Estimator::vins_process_process()
{

    while(1)
    {
        if(config.stereo_enable)
        {
            double time = 0;
            cv::Mat left_image,right_image;
            if(sync_images(time,left_image,right_image))
            {
                feature_manager->input_image(time,left_image,right_image);
            }
        }

    }
}
