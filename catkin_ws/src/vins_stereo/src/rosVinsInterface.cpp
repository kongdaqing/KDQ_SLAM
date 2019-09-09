#include "rosVinsInterface.h"

rosVinsInterface::rosVinsInterface(Parameters& paras):config(paras)
{
    imu_subscriber = node.subscribe(config.imu_topic,1000,&rosVinsInterface::imu_callback,this);
    image_left_subscriber = node.subscribe(config.left_image_topic,200,&rosVinsInterface::leftImage_callback,this);
    image_right_subscriber = node.subscribe(config.right_image_topic,200,&rosVinsInterface::rightImage_callback,this);
    track_image_publisher = node.advertise<sensor_msgs::Image>(config.track_image_topic,100);
    vins_sys = new Estimator(config);
    double time_now = ros::Time::now().toSec();
    vinsInfo_record = new VinsInfoFollow(time_now,config.imageInfo_print_hz);
    vins_thread = thread{&rosVinsInterface::vinsProcess,this};
    showConfig();
}
void rosVinsInterface::showConfig()
{
    ROS_INFO("vins-stereo configuration");
    ROS_INFO("enable stereo: %d",config.stere_enable);
    ROS_INFO("imu topic: %s",config.imu_topic.c_str());
    ROS_INFO("image0 topic: %s",config.left_image_topic.c_str());
    ROS_INFO("image1 topic: %s",config.right_image_topic.c_str());
    ROS_INFO("track-image topic: %s",config.track_image_topic.c_str());
    ROS_INFO("feature point max number: %d",config.feature_point_max_number);
    ROS_INFO("feature minimum distance: %f",config.feature_min_dist);
    ROS_INFO("publish track image: %d",config.pub_track_image);

}
void rosVinsInterface::imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    vins_sys->imu_lock.lock();
    Eigen::Vector3d acc,gyro;
    acc << imu_msg->linear_acceleration.x,imu_msg->linear_acceleration.y,imu_msg->linear_acceleration.z;
    gyro << imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z;
    double t = imu_msg->header.stamp.toSec();
    vins_sys->acc_buf.push(make_pair(t,acc));
    vins_sys->gyro_buf.push(make_pair(t,gyro));
    vins_sys->imu_lock.unlock();
}
void rosVinsInterface::leftImage_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    vins_sys->image_lock.lock();
    img_left_buf.push(image_msg);
    vinsInfo_record->img_info.leftImage_index++;
    vins_sys->image_lock.unlock();
}
void rosVinsInterface::rightImage_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    vins_sys->image_lock.lock();
    img_right_buf.push(image_msg);
    vinsInfo_record->img_info.rightImage_index++;
    vins_sys->image_lock.unlock();
}

cv::Mat rosVinsInterface::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
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
bool rosVinsInterface::syncImages(double &time, cv::Mat &left_image, cv::Mat &right_image)
{

    if(img_left_buf.empty() || img_right_buf.empty())
        return false;
    vins_sys->image_lock.lock();
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
    vins_sys->image_lock.unlock();
    if(left_image.empty() || right_image.empty())
        return false;
    else
        return true;
}
void rosVinsInterface::pubTrackImage(const cv::Mat &imgTrack, const double t)
{
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(t);

    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    track_image_publisher.publish(imgTrackMsg);
}

void rosVinsInterface::vinsProcess()
{

    while(1)
    {
        if(config.stere_enable)
        {
            double time = 0;
            cv::Mat left_image,right_image;
            if(syncImages(time,left_image,right_image))
            {
                double start_count,end_count;
                start_count = static_cast<double>(cv::getTickCount());
                vins_sys->feature_manager->inputImage(time,left_image,right_image);
                end_count = static_cast<double>(cv::getTickCount());
                vinsInfo_record->img_info.track_cost_time = ((double)(end_count - start_count)/cv::getTickFrequency());
                vinsInfo_record->img_info.trackImage_index++;
            }
        }
        double time_now = ros::Time::now().toSec();
        if(vins_sys->feature_manager->getPubTrackFlg() && config.pub_track_image)
        {
            cv::Mat track_image = vins_sys->feature_manager->getPubTrackImage();
            pubTrackImage(track_image,time_now);
            vins_sys->feature_manager->clearPubTrackFlg();
        }
        //vinsInfo_record->printImageInfo(time_now);
        std::chrono::milliseconds dura(20);
        std::this_thread::sleep_for(dura);
    }
}
