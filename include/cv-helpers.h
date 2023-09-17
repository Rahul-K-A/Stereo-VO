#ifndef CV_HELPERS_H
#define CV_HELPERS_H

#include "common-includes.h"

using namespace ximgproc;

namespace cvHelpers{
    void getFeatures( Ptr<SURF> surf, Mat image, vector<cv::KeyPoint>& keyPoints, Mat& descriptor  );
    vector< vector<Point2f> > filterPoints(Ptr<DescriptorMatcher>& matcher, vector<KeyPoint> keyPoints1, Mat& descriptor1, vector<KeyPoint> keyPoints2, Mat& descriptor2 );

}
#endif //CV_HELPERS_H

