#ifndef CV_HELPERS_H
#define CV_HELPERS_H

#include "common-includes.h"

using namespace ximgproc;

namespace cvHelpers{
    void getFeatures( Ptr<SURF> surf, Mat image, vector<cv::KeyPoint>& keyPoints, Mat& descriptor  );
    vector < vector<Point2d> > filterPoints(Ptr<DescriptorMatcher>& matcher, vector<KeyPoint> keyPoints1, Mat& descriptor1, vector<KeyPoint> keyPoints2, Mat& descriptor2 );
    Mat get3DPoints(vector<Point2d> triangulation_points1, vector<Point2d> triangulation_points2, Mat& currentPos, Mat& currentRot);
    Mat convertTo3x4Pose(Mat rotation, Mat position);
    Mat convertTo4x4Pose(Mat rotation, Mat position);
}
#endif //CV_HELPERS_H

