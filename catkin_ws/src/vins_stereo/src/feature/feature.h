#ifndef FEATURE_H
#define FEATURE_H
#include <vector>
#include <map>
#include <queue>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
using namespace std;

class Feature
{
public:
    Feature(int max_pts,int min_dist,int pubTrackImage_flg);
    void inputImage(double t,const cv::Mat& left_img,const cv::Mat& right_img = cv::Mat());
    void trackFeatureUsingFlowPyrLK();
    int getPubTrackFlg(){return flg_pubTrackImage;}
    void clearPubTrackFlg(){flg_pubTrackImage = 0;}
    cv::Mat getPubTrackImage(){return pubTrackImage;}
    void setMask();
    bool inBorder(const cv::Point2f &pt);
    void reduceVector(vector<int>& vec,vector<unsigned char>& status);
    void reduceVector(vector<cv::Point2f>& vec,vector<unsigned char>& status);
private:
    void updatePubTrackImage();
    int cols,rows;
    cv::Mat mask;
    cv::Mat cur_img,pre_img;
    vector<cv::Point2f> pre_pts, cur_pts;
    vector<int> track_cnt;
    cv::Mat pubTrackImage;
    bool flg_pubTrackImage;
    queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > > featureBuf;

    unsigned int para_maxPtsNumber;
    int para_minDist;
    int  para_pubTrackImageFlg;


};

#endif // FEATURE_H
