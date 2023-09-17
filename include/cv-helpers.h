#ifndef CV_HELPERS_H
#define CV_HELPERS_H

#include "common-includes.h"

using namespace cv;
using namespace xfeatures2d;
using namespace std;

namespace cvHelpers{
    void getFeatures( Ptr<SURF> surf, Mat image, vector<cv::KeyPoint>& keyPoints, Mat& descriptor  );
    vector< vector<Point2f> > filterPoints(Ptr<DescriptorMatcher>& matcher, vector<KeyPoint> keyPoints1, Mat& descriptor1, vector<KeyPoint> keyPoints2, Mat& descriptor2 );
    vector<Point3d> get3DPoints(vector<Point2f> triangulation_points1, vector<Point2f> triangulation_points2, Mat& currentPos, Mat& currentRot);
    Mat convertTo3x4Pose(Mat rotation, Mat position);
    Mat convertTo4x4Pose(Mat rotation, Mat position);
    void decompose4x4Pose(Mat pose, Mat& outRotation, Mat& outPosition);
}
#endif //CV_HELPERS_H

