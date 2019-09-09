#include <iostream>
#include <ros/ros.h>
#include "feature/feature.h"
#include "parameters/parameters.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <queue>
#include <opencv2/opencv.hpp>

using namespace std;
int cnt = 0;
int max_corner = 150;
int max_corner_bar = 250;
string source_window = "harris";
queue<cv::Mat> img_buf;
cv::RNG rng(12345);
cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
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

void image_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    cv::Mat img;
    img = getImageFromMsg(image_msg);
    img_buf.push(img);
}
void imageProcess()
{
    while(1)
    {
//        if(!img_buf.empty())
//        {

//            cv::namedWindow(source_window, CV_WINDOW_AUTOSIZE);
//            cv::imshow(source_window, img_buf.front());
//        }

        if(cv::waitKey(100) == 32)
        {
            cout << "before pop :  " << img_buf.size() << endl;
           for(unsigned int i = 0;i<100;i++)
           {

               if(i >= img_buf.size())
                   break;
               img_buf.pop();
           }
           cout << "after pop :  " << img_buf.size() << endl;
        }
        cnt++;
        cout << "running index: "<< cnt << endl;
    }
}
void goodFeaturesToTrack_Demo(int, void*)
{
    if (max_corner < 1) { max_corner = 1; }

    //初始化 Shi-Tomasi algorithm的一些参数
    vector<cv::Point2f> corners;
    double qualityLevel = 0.01;
    double minDistance = 10;
    int blockSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;

    //给原图做一次备份
    cv::Mat src = img_buf.front();
    cv::Mat copy,src_gray;
    cv::cvtColor(src, src_gray, CV_BGR2GRAY);

    copy = src;

    // 角点检测
    cv::goodFeaturesToTrack(src_gray,corners,max_corner,qualityLevel,minDistance,cv::Mat(),blockSize,useHarrisDetector,k);

    //画出检测到的角点
    cout << "** Number of corners detected: " << corners.size() << endl;
    int r = 4;
    for (unsigned int i = 0; i < corners.size(); i++)
    {
        cv::circle(copy, corners[i], r, cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
                                                   rng.uniform(0, 255)), -1, 8, 0);
    }

    cv::namedWindow(source_window, CV_WINDOW_AUTOSIZE);
    cv::imshow(source_window, copy);
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"feature_node");
    ros::NodeHandle n("~");
    ROS_INFO("HELLO Feature Test!");
    if(argc == 1)
    {
        ROS_ERROR("Error: No vins-stereo config file,please allocate config file!");
        ros::shutdown();
        return -1;
    }
    string config_file = argv[1];
    ROS_INFO("config_file: %s", config_file.c_str());
    Parameters config(config_file);
    ros::Subscriber image_subscriber = n.subscribe(config.left_image_topic,200,image_callback);
    thread change_pic;
    change_pic = thread{imageProcess};

    cv::namedWindow(source_window,cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("MaxCorners:", source_window, &max_corner, max_corner_bar, goodFeaturesToTrack_Demo);

    ros::spin();
    return 0;
}
