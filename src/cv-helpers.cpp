#include "cv-helpers.h"
#include "TsukubaParser.h"

/// @brief Get keypoints and descriptors from a given image using SURF
/// @param surf Pointer to SURF detector
/// @param image Input image
/// @param keyPoints Output keypoints
/// @param descriptor Output descriptor
void cvHelpers::getFeatures( Ptr<SURF> surf, Mat image, vector<cv::KeyPoint>& keyPoints, Mat& descriptor  )
{
    surf->detectAndCompute(image, noArray(), keyPoints, descriptor);
}


/// @brief Given 2 pairs of keypoints and descriptors, find the most viable features that can be used for triangulation
/// @param matcher Pointer to FLANNBASED descriptor matcher
/// @param keyPoints1 First input keypoints
/// @param descriptor1 First input descriptor
/// @param keyPoints2 Second input keypoints
/// @param descriptor2 Second input descriptor
/// @return vector of Point2d vectors with size 2, corresponsing to the 2 pairs of input data
vector< vector<Point2d> > cvHelpers::filterPoints(Ptr<DescriptorMatcher>& matcher, vector<KeyPoint> keyPoints1, Mat& descriptor1, vector<KeyPoint> keyPoints2, Mat& descriptor2 )
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
    //TODO: Make this easier to understand
    Mat E;
    Mat mask;
    //Find essential matrix and use the output mask to calculate inliers
    E = cv::findEssentialMat(selected_points1, selected_points2, TsukubaParser::getCameraMatrix().at<double>(0,0),
                           // cv::Point2f(0.f, 0.f),
                           cv::Point2d(240.f, 320.f),
                           cv::RANSAC, 0.999, 1.0, mask);
    vector<cv::Point2f> inlier_match_points1, inlier_match_points2;
    for(int i = 0; i < mask.rows; i++) {
        if(mask.at<unsigned char>(i)){
            inlier_match_points1.push_back(selected_points1[i]);
            inlier_match_points2.push_back(selected_points2[i]);
        }
    }

    //Store the inlier points as double values               
    vector<cv::Point2d> triangulation_points1, triangulation_points2;
    for(int i = 0; i < mask.rows; i++) {
        if(mask.at<unsigned char>(i)){
            triangulation_points1.push_back (cv::Point2d((double)inlier_match_points1[i].x,(double)inlier_match_points1[i].y));
            triangulation_points2.push_back(cv::Point2d((double)inlier_match_points2[i].x,(double)inlier_match_points2[i].y));
        }
    }

    vector< vector<Point2d> > results;
    results.push_back(triangulation_points1);
    results.push_back(triangulation_points2);
    return results;
}



vector<Point3d> cvHelpers::get3DPoints(vector<Point2d> triangulation_points1, vector<Point2d> triangulation_points2, Mat& currentPos, Mat& currentRot)
{

    Mat RtLeft = cvHelpers::convertTo4x4Pose(currentRot, currentPos);
    // Right cam pose = left cam pose * relative transform
    cv::Mat RtRight = RtLeft * TsukubaParser::getRightCamRelativeTransform();
    RtLeft.pop_back();
    RtRight.pop_back();
    cv::Mat homogenised_3d_points;
    cv::Mat Kd = TsukubaParser::getCameraMatrix();

    //Get 3d points wrt Left camera (not scaled afaik)
    cv::triangulatePoints(Kd * RtLeft, Kd * RtRight, triangulation_points1, triangulation_points2,  homogenised_3d_points);

    vector<Point3d> points_3d;
    for(uint16_t i = 0; i < homogenised_3d_points.cols; i++)
    {
        Mat currPoint3d =  homogenised_3d_points.col(i);
        currPoint3d /= currPoint3d.at<double>(3, 0);
        Point3d p(
            currPoint3d.at<double>(0, 0),
            currPoint3d.at<double>(1, 0),
            currPoint3d.at<double>(2, 0)
        );
        points_3d.push_back(p);

    }
    assert(homogenised_3d_points.cols == triangulation_points1.size());
    return points_3d;
}


/// @brief Converts rotation and positon matrices into a single 3x4 pose matrix 
/// @param rotation input rotation matrix
/// @param position input position matrix
/// @return output 3x4 matrix
Mat cvHelpers::convertTo3x4Pose(Mat rotation, Mat position){
    Mat output = cv::Mat::eye(3, 4, CV_64FC1);
    for(uint8_t r = 0; r < rotation.rows; r++)
    {
        for(uint8_t c = 0; c < rotation.cols; c++)
        {
            output.at<double>(r,c) = rotation.at<double>(r,c);
        }   
    }

    output.at<double>(0,3) = position.at<double>(0,0);
    output.at<double>(1,3) = position.at<double>(0,1);
    output.at<double>(2,3) = position.at<double>(0,2);
    return output;
}

/// @brief Converts rotation and positon matrices into a single 4x4 pose matrix 
/// @param rotation input rotation matrix
/// @param position input position matrix
/// @return output 4x4 matrix 
Mat cvHelpers::convertTo4x4Pose(Mat rotation, Mat position)
{
    Mat output = cv::Mat::eye(4, 4, CV_64FC1);
    for(uint8_t r = 0; r < rotation.rows; r++)
    {
        for(uint8_t c = 0; c < rotation.cols; c++)
        {
            output.at<double>(r,c) = rotation.at<double>(r,c);
        }   
    }

    output.at<double>(0,3) = position.at<double>(0,0);
    output.at<double>(1,3) = position.at<double>(0,1);
    output.at<double>(2,3) = position.at<double>(0,2);
    output.at<double>(3,3) = (double)1.f;
    return output;
}




