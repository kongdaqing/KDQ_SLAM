/*
  Project: read_image
  Goal:    get image from ros node and use opencv imshow images.
  Author:  KDQ
  Data:    2019/7/2
*/

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace std;
using namespace cv;

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
  // we get image infos from ros through cv_brige.
  //cv_bridge transform image type from ros to opencv
  cv_bridge::CvImageConstPtr ptr;
  sensor_msgs::Image img;
  if (img_msg->encoding == "8UC1")
  {
    img.header = img_msg->header;
    img.height = img_msg->height;
    img.width = img_msg->width;
    img.is_bigendian = img_msg->is_bigendian;
    img.step = img_msg->step;
    img.data = img_msg->data;
    img.encoding = "mono8";
    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
  }else{
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
  }
  Mat image = ptr->image;
  if(!image.empty())
  {
    imshow("read_image", image);
    waitKey(30);
  }


}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "ReadImage");
  ros::NodeHandle n("~");
  string IMAGE_TOPIC = "/cam0/image_raw";

  ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);
  ros::spin();
  return 0;
}

