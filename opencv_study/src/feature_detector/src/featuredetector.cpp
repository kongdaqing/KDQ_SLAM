#include "featuredetector.h"

void KDQFeatureDetector::ExtractFeaturePoints(Mat &image, int8_t method)
{

  switch (method) {
  case Harris:
    ExtractFeatureByHarris(image);
    break;
  case ORB:
    break;
  default:
    break;
  }


}

void KDQFeatureDetector::ExtractFeatureByHarris(Mat& image)
{
  uchar thresh = 200;
  Mat dst,norm_dst,gray_img,abs_dst;
  cvtColor(image, gray_img, CV_BGR2GRAY);
  cornerHarris(gray_img, dst, 2, 3, 0.04);
  normalize(dst, norm_dst, 0, 255, NORM_MINMAX, CV_32FC1,Mat());
  convertScaleAbs(norm_dst, abs_dst);//？


  for (int i = 0; i < image.rows; i++) {
    for (int j = 0; j < image.cols; j++) {
      uchar value = abs_dst.at<uchar>(i, j);
      if (value > thresh) {
        circle(image, Point(j, i),1,Scalar(0,255,0),2);
      }
    }
  }

}
