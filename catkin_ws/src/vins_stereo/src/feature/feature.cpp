#include "feature.h"

Feature::Feature(int max_pts,int min_dist,int pubTrackImage_flg)
    :para_maxPtsNumber(max_pts),
      para_minDist(min_dist),
      para_pubTrackImageFlg(pubTrackImage_flg)
{
    pre_img = cv::Mat();
    cur_img = cv::Mat();
    cur_pts.clear();
    pre_pts.clear();
    flg_pubTrackImage = false;
    pubTrackImage = cv::Mat();

}
void reduceVector(vector<cv::Point2f>& vec,vector<unsigned char>& status)
{
    int j =0,i=0;
    for(auto &index:status)
    {
        i++;
        if(index)
            vec[j++] = vec[i];
    }
    vec.resize(j);
}

void Feature::updatePubTrackImage(const cv::Mat &img, vector<cv::Point2f>& pre_feature,vector<cv::Point2f>& cur_feature)
{
    pubTrackImage = img.clone();
    cv::cvtColor(pubTrackImage,pubTrackImage,CV_GRAY2RGB);
    for (size_t j = 0; j < cur_feature.size(); j++)
    {
        double len = 25;
        cv::circle(pubTrackImage, cur_feature[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
        cv::line(pubTrackImage,cur_feature[j],cur_feature[j],cv::Scalar(255, 0, 0), 2);
    }
    flg_pubTrackImage = 1;
}



void Feature::trackFeatureUsingFlowPyrLK(const cv::Mat& pre_image,vector<cv::Point2f>& pre_feature,const cv::Mat& cur_image,vector<cv::Point2f>& cur_feature)
{
    cur_feature.clear();
    if(pre_feature.size() == 0)
    {
        cv::goodFeaturesToTrack(cur_image,cur_feature,para_maxPtsNumber,0.01f,para_minDist);
        return;
    }

    vector<unsigned char> status;
    vector<float> err;
    cv::calcOpticalFlowPyrLK(pre_image,pre_feature,cur_image,cur_feature,status,err,cv::Size(21,21),3);
    reduceVector(cur_feature,status);
    reduceVector(pre_feature,status);

    if(para_pubTrackImageFlg)
    {
        updatePubTrackImage(cur_img,pre_pts,cur_pts);
    }
    vector<cv::Point2f> pts;
    if(cur_pts.size() < para_maxPtsNumber)
        cv::goodFeaturesToTrack(cur_image,pts,para_maxPtsNumber - cur_feature.size(),0.01f,para_minDist);

    for(auto &index:n_pts)
    {
        cur_feature.push_back(index);
    }

}

void Feature::inputImage(double t,const cv::Mat& left_img,const cv::Mat& right_img)
{
    cur_img = left_img;
    trackFeatureUsingFlowPyrLK(pre_img,pre_pts,cur_img,cur_pts);
    pre_img = cur_img;
    pre_pts = cur_pts;
}
