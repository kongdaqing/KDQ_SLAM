#ifndef FEATUREDETECTOR_H
#define FEATUREDETECTOR_H
#include <opencv2/opencv.hpp>
using namespace cv;

class KDQFeatureDetector
{

public:
  enum{
    Harris,
    ORB,
    SIFT,
    SURF
  };
  KDQFeatureDetector(){};
  void ExtractFeaturePoints(Mat& image,int8_t method);
  void ExtractFeatureByHarris(Mat& image);
};

#endif // FEATUREDETECTOR_H
