#ifndef CV_HELPERS_H
#define CV_HELPERS_H

#include "common-includes.h"


namespace cvHelpers{
    void getFeatures( cv::Ptr<cv::xfeatures2d::SURF> surf, cv::Mat image, vector<cv::KeyPoint>& keyPoints, cv::Mat& descriptor  );
    vector< vector<cv::Point2d> > filterPoints(cv::BFMatcher* matcher, vector<cv::KeyPoint> keyPoints1, cv::Mat& descriptor1, vector<cv::KeyPoint> keyPoints2, cv::Mat& descriptor2 );
    vector<cv::Point3d> get3DPoints(vector<cv::Point2d> triangulation_points1, vector<cv::Point2d> triangulation_points2, cv::Mat& currentPos, cv::Mat& currentRot);
    cv::Mat convertTo3x4Pose(cv::Mat rotation, cv::Mat position);
    cv::Mat convertTo4x4Pose(cv::Mat rotation, cv::Mat position);
    void decompose4x4Pose(cv::Mat pose, cv::Mat& outRotation, cv::Mat& outPosition);
}
#endif //CV_HELPERS_H

