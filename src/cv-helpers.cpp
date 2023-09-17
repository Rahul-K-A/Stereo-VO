#include "cv-helpers.h"

void cvHelpers::getFeatures( Ptr<SURF> surf, Mat image, vector<cv::KeyPoint>& keyPoints, Mat& descriptor  )
{
    surf->detectAndCompute(image, noArray(), keyPoints, descriptor);
}

vector< vector<Point2f> > cvHelpers::filterPoints(Ptr<DescriptorMatcher>& matcher, vector<KeyPoint> keyPoints1, Mat& descriptor1, vector<KeyPoint> keyPoints2, Mat& descriptor2 )
{
    //cv::BFMatcher* matcher = new cv::BFMatcher(cv::NORM_L2, false); 

    //Image 1 to 2 correspondance, and Image 2 to 1 correspondance;
    vector< vector<cv::DMatch> > knn_matches_12, knn_matches_21;
    
    //Find matches
    matcher->knnMatch( descriptor1, descriptor2, knn_matches_12, 2 );
    matcher->knnMatch( descriptor2, descriptor1, knn_matches_21, 2 );

    //Perform lowes ratio test
    const double ratio = 0.8;
    vector<cv::Point2f> selected_points1, selected_points2;
    
    //Do lowes ratio test for KNN matches and only accept point if both tests past
    for(int i = 0; i < knn_matches_12.size(); i++) {
        if( knn_matches_12[i][0].distance/knn_matches_12[i][1].distance < ratio and knn_matches_21[knn_matches_12[i][0].trainIdx][0].distance / knn_matches_21[knn_matches_12[i][0].trainIdx][1].distance < ratio ) 
        {
            // If both tests pass, verify that the KNN matches are actually using the same keypoints
            if(knn_matches_21[knn_matches_12[i][0].trainIdx][0].trainIdx == knn_matches_12[i][0].queryIdx) 
            {
                selected_points1.push_back(keyPoints1[knn_matches_12[i][0].queryIdx].pt);
                selected_points2.push_back(keyPoints2[knn_matches_21[knn_matches_12[i][0].trainIdx][0].queryIdx].pt);
            }
        }
    }
    vector< vector<Point2f> > results;
    results.push_back(selected_points1);
    results.push_back(selected_points2);
    return results;
}



