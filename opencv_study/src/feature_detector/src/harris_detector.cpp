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
    namedWindow("input",CV_WINDOW_AUTOSIZE);
    namedWindow("output", CV_WINDOW_AUTOSIZE);
    Mat src, dst,norm_dst,gray_img,abs_dst,out1;
    int thresh = 200;
    while(1)
    {

        capture >> src;  //读取当前帧
        if (src.empty())
        {
            break;
        }
        else
        {

            imshow("input", src); //显示当前帧
            dst = Mat::zeros(gray_img.size(), CV_32FC1);
            out1 = Mat::zeros(gray_img.size(), CV_32FC1);
            cvtColor(src, gray_img, CV_BGR2GRAY);
            cornerHarris(gray_img, dst, 2, 3, 0.04);
            normalize(dst, norm_dst, 0, 255, NORM_MINMAX, CV_32FC1,Mat());
            convertScaleAbs(norm_dst, abs_dst);//？

            Mat result_img = src.clone();
            for (int i = 0; i < result_img.rows; i++) {
                for (int j = 0; j < result_img.cols; j++) {
                    uchar value = abs_dst.at<uchar>(i, j);
                    if (value > thresh) {
                        circle(result_img, Point(j, i),1,Scalar(0,255,0),2);
                    }
                }
            }
            imshow("output", result_img);
        }
        if(waitKey(30) > 0) //延时30ms
            break;
    }
    capture.release();
    destroyAllWindows();

    ros::spinOnce();
    return 0;
}

/*
#include<opencv2/opencv.hpp>
using namespace cv;
using namespace std;

int thresh = 80;
Mat src, dst,norm_dst,gray_img,abs_dst,out1,out2;
void callback(int, void*);
int main(int arc, char** argv)
{

    src = imread("1.jpg");
    namedWindow("input",CV_WINDOW_AUTOSIZE);
    imshow("input", src);
    cvtColor(src, gray_img, CV_BGR2GRAY);

    namedWindow("output", CV_WINDOW_AUTOSIZE);
    createTrackbar("threshold", "output", &thresh, 255, callback);
    callback(0, 0);
    waitKey(0);
    return 0;
}
void callback(int, void*) {
    dst = Mat::zeros(gray_img.size(), CV_32FC1);
    out1 = Mat::zeros(gray_img.size(), CV_32FC1);

    cornerHarris(gray_img, dst, 2, 3, 0.04);
    normalize(dst, norm_dst, 0, 255, NORM_MINMAX, CV_32FC1,Mat());
    convertScaleAbs(norm_dst, abs_dst);//？

    Mat result_img = src.clone();
    for (int i = 0; i < result_img.rows; i++) {
        for (int j = 0; j < result_img.cols; j++) {
            uchar value = abs_dst.at<uchar>(i, j);
            if (value > thresh) {
                circle(result_img, Point(j, i),1,Scalar(0,255,0),2);
            }
        }
    }
    imshow("output", result_img);
}
*/
