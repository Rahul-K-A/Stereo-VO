#include "cv-helpers.h"
#include "TsukubaParser.h"

using namespace cv;
using namespace xfeatures2d;
using namespace ximgproc;
using namespace std;


/// @brief Allocate memory and initialise all the algorithms used
/// @param surf Reference to surf object pointer
/// @param correspondence_matcher Reference to BFMatcher object pointer
/// @param left_matcher Reference to StereoSGBM object pointer
/// @param right_matcher Reference to StereoMatcher object pointer
/// @param wls_filter Reference to WLSDisparityFiler object pointer
void cvHelpers::createFilters( Ptr<SURF>& surf , Ptr<BFMatcher>& correspondence_matcher, Ptr<StereoSGBM>& left_matcher, Ptr<StereoMatcher>& right_matcher, Ptr<DisparityWLSFilter>& wls_filter )
{

    surf = SURF::create();
    correspondence_matcher = BFMatcher::create(NORM_L2, false);
    //Empirical parameters based on messing around
    left_matcher  = StereoSGBM::create(0, 96, 5);
    left_matcher->setMinDisparity(0);
    left_matcher->setNumDisparities(160);
    left_matcher->setSpeckleRange(7);
    left_matcher->setP1(200);
    left_matcher->setP2(500);
    left_matcher->setPreFilterCap(1);
    left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
    left_matcher->setUniquenessRatio(0);
    wls_filter = ximgproc::createDisparityWLSFilter(left_matcher);
    right_matcher = ximgproc::createRightMatcher(left_matcher);
    wls_filter->setSigmaColor(2.f);
}


/// @brief Given two images, this function generates a signed 16-bit disparity map
/// @param imLeft Left view image
/// @param imRight Right view image 
/// @param outDisparity output 16-bit Disparity view
/// @param left_matcher Pointer to SGBM matcher
/// @param right_matcher Pointer to corresponding right matcher
/// @param wls_filter Pointer to WLS filter 
/// @return ROI of disparity image
Rect cvHelpers::generateDisparityMap(Mat& imLeft, Mat& imRight, Mat& outDisparity, Ptr<StereoSGBM>& left_matcher,  Ptr<StereoMatcher>& right_matcher,  Ptr<DisparityWLSFilter>& wls_filter)
{
    Mat gLeft, gRight;
    Mat lDisparity, rDisparity;
    Mat filteredDisparity;
    cvtColor(imLeft, gLeft, COLOR_BGRA2GRAY);
    cvtColor(imRight, gRight, COLOR_BGRA2GRAY);
    left_matcher->compute(gLeft, gRight,lDisparity);
    right_matcher->compute(gRight, gLeft, rDisparity);
    wls_filter->filter(lDisparity, gLeft, filteredDisparity, rDisparity, Rect(), gRight);
    Rect ROI = wls_filter->getROI();
    filteredDisparity = filteredDisparity(ROI);
    outDisparity = filteredDisparity.clone();
    return ROI;
}

/// @brief Get keypoints and descriptors from a given image using SURF
/// @param surf Pointer to SURF detector
/// @param image Input image
/// @param keyPoints Output keypoints
/// @param descriptor Output descriptor
void cvHelpers::getFeatures( Ptr<SURF> surf, Mat image, vector<cv::KeyPoint>& keyPoints, Mat& descriptor  )
{
    if(!keyPoints.empty() || !descriptor.empty())
    {
        descriptor.release();
        keyPoints.clear();
    }
    surf->detectAndCompute(image, noArray(), keyPoints, descriptor);
}

/// @brief Given a sequence of 2 image, find the common features between them
/// @param correspondance_matcher 
/// @param keyPoints1 
/// @param descriptor1 
/// @param keyPoints2 
/// @param descriptor2 
/// @return Vector of vectors containing image coordinates of the features in the current and previous images respectively
vector< vector<Point2f> > cvHelpers::findCorrespondance(Ptr<BFMatcher> correspondance_matcher, vector<KeyPoint> keyPoints1, Mat& descriptor1, vector<KeyPoint> keyPoints2, Mat& descriptor2 )
{
    //Image 1 to 2 correspondance, and Image 2 to 1 correspondance;
    vector< vector<cv::DMatch> > knn_matches_12, knn_matches_21;
    
    //Find matches
    correspondance_matcher->knnMatch( descriptor1, descriptor2, knn_matches_12, 2 );
    correspondance_matcher->knnMatch( descriptor2, descriptor1, knn_matches_21, 2 );

    //Perform lowes ratio test
    const double ratio = 0.5;
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

#if 0
/// @brief Given 2 pairs of keypoints and descriptors, find the most viable features that can be used for triangulation
/// @param matcher Pointer to FLANNBASED descriptor matcher
/// @param keyPoints1 First input keypoints
/// @param descriptor1 First input descriptor
/// @param keyPoints2 Second input keypoints
/// @param descriptor2 Second input descriptor
/// @return vector of Point2d vectors with size 2, corresponsing to the 2 pairs of input data
vector< vector<Point2d> > cvHelpers::filterPoints(cv::BFMatcher* matcher, vector<KeyPoint> keyPoints1, Mat& descriptor1, vector<KeyPoint> keyPoints2, Mat& descriptor2 )
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
    Mat E,R,t,mask;
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
    mask.release();
    cv::recoverPose(E, 
                  inlier_match_points1,
                  inlier_match_points2, 
                  R, t, TsukubaParser::getCameraMatrix().at<double>(0,0), 
                  // cv::Point2f(0, 0),
                  cv::Point2d(240.f, 320.f),
                  mask);

    //Do second round of filtering              
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
#endif

/// @brief Generate 3D World coordinate points from disparity and feature points
/// @param disparityMap 16-bit signed disparity image
/// @param imageCoords floating point image coords
/// @return Vector containing world coordinates of features
vector<Point3f> cvHelpers::imgToWorldCoords(Mat disparityMap, vector<Point2f> imageCoords)
{
    vector<Point3f> outVecDepth;
    int16_t currDisp = 0;
    int16_t prevDisp = 0;
    Point3f depthPoint;
    for(Point2f point : imageCoords )
    {
        currDisp = disparityMap.at<int16_t>(point);
        if(currDisp){
            prevDisp = currDisp;
        }
        else{
            currDisp = prevDisp;
        }
        depthPoint.z = (1.f * FX * BASELINE)/ currDisp;
        depthPoint.x = (point.x - CX) * (depthPoint.z / FX);
        depthPoint.y = (point.y - CY) * (depthPoint.z / FY);
        outVecDepth.push_back(depthPoint);
    }
    return outVecDepth;
}



/// @brief Performs 3D Point triangulation and returns a vector of 3D world points
/// @param triangulation_points1 Left camera points
/// @param triangulation_points2 Right camera points
/// @param currentPos Current Left cam position
/// @param currentRot Current Left cam rotation
/// @return Vector contanining homogenised 3D world points
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
    //Output 3d points must have same number of elements as the input triangulated points
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

/// @brief Converts rotation and positon matrices into a single 4x4 64-bit floating point pose matrix 
/// @param rotation input rotation matrix (type: double)
/// @param position input position matrix (type: double)
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



/// @brief Decompose a 64-bit floating point 4x4matrix into 64-bit rotation and translation matrices
/// @param pose Input double-type matrix
/// @param outRotation Reference to output double-type Rotation matrix
/// @param outPosition Reference to output double-type Translation Matrix
void cvHelpers::decompose4x4Pose(Mat pose, Mat& outRotation, Mat& outPosition)
{
    cout<<"decompose"<<endl;
    for(uint8_t row = 0; row < 3; row++)
    {
        for(uint8_t col = 0; col < 3; col++)
        {
            outRotation.at<double>(row, col) = (double) pose.at<double>(row,col);
        }

    }

    outPosition.at<double>(0,0) = (double) pose.at<double>(0,3);
    outPosition.at<double>(0,1) = (double) pose.at<double>(1,3) ;
    outPosition.at<double>(0,2) = (double) pose.at<double>(2,3) ;
}


void cvHelpers::pose_estimation_3d3d(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    Mat &R, Mat &t)
{
    Point3f p1, p2;
// center of mass
    int N = pts1.size();
    for (int i = 0; i < N; i++) {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = Point3f(Vec3f(p1) / N);
    p2 = Point3f(Vec3f(p2) / N);
    vector<Point3f> q1(N), q2(N); // remove the center
    for (int i = 0; i < N; i++) {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1∗q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++) {
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d R_ = U * (V.transpose());
    if (R_.determinant() < 0) {
        R_ = -R_;
    }
    Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);
    // convert to cv::Mat
    R = (Mat_<double>(3, 3) <<
    R_(0, 0), R_(0, 1), R_(0, 2),
    R_(1, 0), R_(1, 1), R_(1, 2),
    R_(2, 0), R_(2, 1), R_(2, 2)
    );

    R = R.inv();
    t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
}


