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
    int getPubTrackFlg(){return flg_pubTrackImage;}
    void clearPubTrackFlg(){flg_pubTrackImage = 0;}
    cv::Mat getPubTrackImage(){return pubTrackImage;}


private:
    void updatePubTrackImage(const cv::Mat& img,vector<cv::Point2f>& pts);

    cv::Mat cur_img,pre_img;
    vector<cv::Point2f> prev_pts, cur_pts, n_pts;
    cv::Mat pubTrackImage;
    bool flg_pubTrackImage;
    queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > > featureBuf;

    unsigned int para_maxPtsNumber;
    int para_minDist;
    int  para_pubTrackImageFlg;


};

#endif // FEATURE_H
