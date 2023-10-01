#ifndef CV_HELPERS_H
#define CV_HELPERS_H

#include "common-includes.h"


namespace cvHelpers{
    void createFilters( cv::Ptr<cv::xfeatures2d::SURF>& surf , cv::Ptr<cv::BFMatcher>& correspondence_matcher, cv::Ptr<cv::StereoSGBM>& left_matcher, cv::Ptr< cv::StereoMatcher>& right_matcher, cv::Ptr< cv::ximgproc::DisparityWLSFilter>& wls_filter );
    
    void getFeatures( cv::Ptr<cv::xfeatures2d::SURF> surf, cv::Mat image, vector<cv::KeyPoint>& keyPoints, cv::Mat& descriptor  );
    
    cv::Rect generateDisparityMap(cv::Mat& imLeft, cv::Mat& imRight, cv::Mat& outDisparity, cv::Ptr<cv::StereoSGBM>& left_matcher,  cv::Ptr<cv::StereoMatcher>& right_matcher,  cv::Ptr< cv::ximgproc::DisparityWLSFilter>& wls_filter);
    
    vector< vector<cv::Point2f> > findCorrespondance(cv::Ptr<cv::BFMatcher> correspondance_matcher, vector<cv::KeyPoint> keyPoints1, cv::Mat& descriptor1, vector<cv::KeyPoint> keyPoints2, cv::Mat& descriptor2 );
    
    #if 0
    vector< vector<cv::Point2d> > filterPoints(cv::BFMatcher* matcher, vector<cv::KeyPoint> keyPoints1, cv::Mat& descriptor1, vector<cv::KeyPoint> keyPoints2, cv::Mat& descriptor2 );
    #endif
    
    vector<cv::Point3f> imgToWorldCoords(cv::Mat disparityMap, vector<cv::Point2f> imageCoords);

    vector<cv::Point3d> get3DPoints(vector<cv::Point2d> triangulation_points1, vector<cv::Point2d> triangulation_points2, cv::Mat& currentPos, cv::Mat& currentRot);
    
    cv::Mat convertTo3x4Pose(cv::Mat rotation, cv::Mat position);
    
    cv::Mat convertTo4x4Pose(cv::Mat rotation, cv::Mat position);
    
    void decompose4x4Pose(cv::Mat pose, cv::Mat& outRotation, cv::Mat& outPosition);
    void pose_estimation_3d3d( const vector<cv::Point3f> &pts1, const vector<cv::Point3f> &pts2,  cv::Mat &R, cv::Mat &t);
}
#endif //CV_HELPERS_H

