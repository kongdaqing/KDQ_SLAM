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
#include "tools/tic_toc.h"
using namespace std;

int key_value = 0;
double qualityLevel = 0.01;
double minDistance = 10;
int blockSize = 3;
int cnt = 0;
int max_corner = 150;
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

void checkKeyboard()
{
  static int key_type = 0;
  if(key_value == '1')
    key_type = 1;
  else if (key_value == '2') {
    key_type = 2;
  }else if (key_value == '3') {
    key_type = 3;
  }else if(key_value == 27){
    key_type = 0;
  }


  switch (key_type) {
  case 1:
    cout << "Adjust qualityLevel! current value is " << qualityLevel << endl;
    if(key_value == 61)
      qualityLevel += 0.01;
    if(key_value == 45)
      qualityLevel -=0.01;
    if(qualityLevel <= 0)
      qualityLevel = 0.01;
    break;
  case 2:
    cout << "Adjust minDistance! current value is " << minDistance << endl;
    if(key_value == 61)
      minDistance += 5;
    if(key_value == 45)
      minDistance -= 5;
    if(minDistance <= 0)
      minDistance = 5;
    break;
  case 3:
    if(key_value == 61)
      blockSize += 3;
    if(key_value == 45)
      blockSize -= 3;
    if(blockSize <= 0)
      blockSize = 3;
    cout << "Adjust blockSize! current value is " << blockSize << endl;
    break;
  default:break;
  }

}
void goodFeaturesToTrack_Demo(double qualityLevel,double minDistance,int blockSize)
{
  if (max_corner < 1) { max_corner = 1; }

  //初始化 Shi-Tomasi algorithm的一些参数
  vector<cv::Point2f> corners;
  //    double qualityLevel = 0.01;
  //    double minDistance = 10;
  //    int blockSize = 3;
  bool useHarrisDetector = true;
  double k = 0.04;

  //给原图做一次备份
  cv::Mat src = img_buf.front().clone();
  cv::Mat copy,src_gray;
  cv::cvtColor(src, src_gray, CV_BGR2GRAY);

  copy = src;

  // 角点检测
  TicToc tic;
  cv::goodFeaturesToTrack(src_gray,corners,max_corner,qualityLevel,minDistance,cv::Mat(),blockSize,useHarrisDetector,k);
  double time_ms = tic.toc();
  cout << "take time is " << time_ms << "ms" << endl;
  //画出检测到的角点
  cout << "** Number of corners detected: " << corners.size() << endl;
  int r = 4;
  for (unsigned int i = 0; i < corners.size(); i++)
  {
    cv::circle(copy, corners[i], r, cv::Scalar(0, 0, 255), -1, 8, 0);
  }

  cv::namedWindow(source_window, CV_WINDOW_AUTOSIZE);
  cv::imshow(source_window, copy);
}

void imageProcess()
{
  while(1)
  {

    checkKeyboard();
    if(!img_buf.empty())
     goodFeaturesToTrack_Demo(qualityLevel,minDistance,blockSize);
    key_value = cv::waitKey(300);

    if(key_value == 32)
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

  }
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
  if(argc == 3)
  {
     string first_img = argv[2];
     cv::Mat img = cv::imread(first_img);
     img_buf.push(img);

  }
  ros::Subscriber image_subscriber = n.subscribe(config.left_image_topic,200,image_callback);
  thread change_pic;
  change_pic = thread{imageProcess};


  ros::spin();
  return 0;
}
