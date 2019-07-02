#include <ros/ros.h>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "capture_camera");
    ros::NodeHandle cap_camera("~");
    VideoCapture capture(0);//打开摄像头  
    if (!capture.isOpened())//没有打开摄像头的话，就返回。
	return -1;

   while(1)
   {
	Mat frame; //定义一个Mat变量，用于存储每一帧的图像  
	capture >> frame;  //读取当前帧                          
	if (frame.empty())
	{
		break;
	}	            
	else
	{
		imshow("Video", frame); //显示当前帧  
	}
	if(waitKey(30) > 0) //延时30ms
	break;
    }
    capture.release();
    destroyAllWindows();

    ros::spinOnce();
    return 0;
}

