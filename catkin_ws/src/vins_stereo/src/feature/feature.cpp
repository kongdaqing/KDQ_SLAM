#include "feature.h"

Feature::Feature(int max_pts,int min_dist,int pubTrackImage_flg)
    :para_maxPtsNumber(max_pts),
     para_minDist(min_dist),
     para_pubTrackImageFlg(pubTrackImage_flg)
{
  pre_img = cv::Mat();
  cur_img = cv::Mat();
  cur_pts.clear();
  prev_pts.clear();
  flg_pubTrackImage = false;
  pubTrackImage = cv::Mat();

}
void Feature::updatePubTrackImage(const cv::Mat &img, vector<cv::Point2f> &pts)
{
  pubTrackImage = img.clone();
  cv::cvtColor(pubTrackImage,pubTrackImage,CV_GRAY2RGB);
  for (size_t j = 0; j < pts.size(); j++)
  {
      double len = 25;
      cv::circle(pubTrackImage, pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
  }
  flg_pubTrackImage = 1;
}

void Feature::inputImage(double t,const cv::Mat& left_img,const cv::Mat& right_img)
{
     cur_img = left_img;
     vector<unsigned char> status;
     vector<float> err;
     cur_pts.clear();
     if(prev_pts.size()>0){
         cv::calcOpticalFlowPyrLK(pre_img,prev_pts,cur_img,cur_pts,status,err,cv::Size(21,21),3);
     }

     if(cur_pts.size() < para_maxPtsNumber)
      cv::goodFeaturesToTrack(cur_img,n_pts,para_maxPtsNumber - cur_pts.size(),0.01f,para_minDist);

     for(auto &index:n_pts)
     {
         cur_pts.push_back(index);
     }

     if(para_pubTrackImageFlg)
     {
         updatePubTrackImage(cur_img,cur_pts);
     }

     pre_img  = cur_img;


}
