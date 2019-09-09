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
    track_cnt.clear();
    flg_pubTrackImage = false;
    pubTrackImage = cv::Mat();

}

bool Feature::inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < cols - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < rows - BORDER_SIZE;
}

void Feature::reduceVector(vector<cv::Point2f>& vec,vector<unsigned char>& status)
{
    int j =0;
    for(unsigned int i = 0;i < status.size();i++)
    {
        if(status[i] == 1)
            vec[j++] = vec[i];
    }
    vec.resize(j);
}
void Feature::reduceVector(vector<int>& vec,vector<unsigned char>& status)
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

void Feature::updatePubTrackImage()
{
    pubTrackImage = cur_img.clone();
    cv::cvtColor(pubTrackImage,pubTrackImage,CV_GRAY2RGB);
    for (size_t j = 0; j < cur_pts.size(); j++)
    {
       double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
        cv::circle(pubTrackImage, cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    flg_pubTrackImage = 1;
}

void Feature::setMask()
{
   mask = cv::Mat(rows, cols, CV_8UC1, cv::Scalar(255));
   vector<pair<int,cv::Point2f>> sort_pts;

   for(unsigned int i=0;i<track_cnt.size();i++)
   {
     sort_pts.push_back(make_pair(track_cnt[i],cur_pts[i]));
   }

   sort(sort_pts.begin(),sort_pts.end(),[](const pair<int,cv::Point2f> &a,const pair<int,cv::Point2f>&b)
   {return a.first > b.first;});

   cur_pts.clear();
   track_cnt.clear();


   for(auto &it:sort_pts)
   {

        if(mask.at<uchar>(it.second) == 255)
        {

            cur_pts.push_back(it.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second,para_minDist, 0, -1);

        }
   }



}

void Feature::trackFeatureUsingFlowPyrLK()
{
    static int cnt = 0;
    static double sum_time = 0;
    cnt++;

    cur_pts.clear();
    if(pre_pts.size() == 0)
    {
        cout << "intialize track!" << endl;
        cv::goodFeaturesToTrack(cur_img,cur_pts,para_maxPtsNumber,0.01f,para_minDist);
        for(unsigned int i=0;i<cur_pts.size();i++)
            track_cnt.push_back(1);
        return;
    }

    vector<uchar> status;
    vector<float> err;
    TicToc time_cost;
    cv::calcOpticalFlowPyrLK(pre_img,cur_img,pre_pts,cur_pts,status,err,cv::Size(21,21),3);
    sum_time += time_cost.toc();
    if(cnt == 30)
    {
        cout << " track image feature cost: " << sum_time/cnt << "ms" << endl;
        cnt = 0;
        sum_time = 0;
    }

    for(unsigned int i=0;i<status.size();i++)
        if(status[i] == 1 && !inBorder(cur_pts[i]))
            status[i] = 0;

    reduceVector(cur_pts,status);
    reduceVector(pre_pts,status);
    reduceVector(track_cnt,status);

    for(auto &p:track_cnt)
        p++;
    setMask();
    //cout << "track size: " << cur_pts.size() << endl;

    if(cur_pts.size() < para_maxPtsNumber)
        cv::goodFeaturesToTrack(cur_img,n_pts,para_maxPtsNumber - cur_pts.size(),0.01f,para_minDist,mask);
    else
        n_pts.clear();
    //cout << "n_pts: " << n_pts.size() << endl;
    for(auto &index:n_pts)
    {
        cur_pts.push_back(index);
        track_cnt.push_back(1);
    }




    if(para_pubTrackImageFlg)
    {
       updatePubTrackImage();
    }
}

void Feature::inputImage(double t,const cv::Mat& left_img,const cv::Mat& right_img)
{
    cur_img = left_img;
    cols = cur_img.cols;
    rows = cur_img.rows;

    trackFeatureUsingFlowPyrLK();

    pre_img = cur_img;
    pre_pts = cur_pts;
}
