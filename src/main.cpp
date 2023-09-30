#include "common-includes.h"
#include "pcl-helpers.h"
#include "cv-helpers.h"
#include "TsukubaParser.h"

using namespace std;
using namespace cv;
using namespace xfeatures2d;
using namespace ximgproc;

//Left cam -> origin of coordinate system
Mat currPosition = (Mat1d(1, 3) << 0, 0, 0);
Mat currRotation = Mat::eye(3, 3, CV_64F);
Mat LGImage, RGImage;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr myPC;
Mat prevLeftImage;
Mat prevDisparity;
Mat currLeftImage;

//Algorithm pointers
Ptr<SURF> surf = nullptr;
Ptr<BFMatcher> correspondance_matcher = nullptr; 
Ptr<StereoSGBM> left_matcher = nullptr;
Ptr<StereoMatcher> right_matcher = nullptr;
Ptr<DisparityWLSFilter> wls_filter = nullptr;
Rect ROI;
vector<KeyPoint> currentKp, prevKp;
Mat currDesc, prevDesc;

void processFirstStereo(TsukubaParser& tParser, Ptr<SURF>& surf, Ptr<BFMatcher> correspondance_matcher)
{
    tParser.getNextStereoImages();
    Mat lImg, rImg;
    lImg = tParser.getLImage().clone();
    rImg = tParser.getRImage().clone();

    ROI = cvHelpers::generateDisparityMap(lImg, rImg, prevDisparity, left_matcher, right_matcher, wls_filter);
    prevLeftImage = tParser.getLImage()(ROI);
    currLeftImage = prevLeftImage.clone();
    cvHelpers::getFeatures(surf, prevLeftImage, prevKp, prevDesc );
}

int main(int argc, char** argv)
{
    if (argc == 1 || argc > 2 )
    {
        cout << "Please run the program as NTSD-VisualOdometry <path-to-NTSD-root-directory>" << endl;
    }
    string DBPath = argv[1];
    TsukubaParser tParser(DBPath);

    cvHelpers::createFilters(surf, correspondance_matcher, left_matcher, right_matcher, wls_filter );
    

    //Output position and rotation
    Mat outPos = (Mat1d(1, 3) << 0, 0, 0);
    Mat outRot = Mat::eye(3, 3, CV_64F);

    //TODO: add "Process first stereo image here". We have to set first image as frame of reference for all others
    cout<< "ProcessFirstStereo\n";
    Mat lImg, rImg;
    processFirstStereo(tParser, surf, correspondance_matcher);
    cout<< "Loop\n";
    Mat currDisparity;
    while( tParser.getIter() <= NTSD_DB_SIZE)
    {
        tParser.getNextStereoImages();
        lImg = tParser.getLImage().clone();
        rImg = tParser.getRImage().clone();

        ROI = cvHelpers::generateDisparityMap(lImg, rImg, currDisparity, left_matcher, right_matcher, wls_filter );
        lImg = lImg(ROI);

        cvHelpers::getFeatures(surf, lImg, currentKp, currDesc );
        vector<vector<Point2f>> filtered_img_coords = cvHelpers::findCorrespondance(correspondance_matcher, currentKp, currDesc, prevKp, prevDesc);
        vector<Point3f> currWorldPts = cvHelpers::imgToWorldCoords(currDisparity, filtered_img_coords[0]);
        vector<Point3f> prevWorldPts = cvHelpers::imgToWorldCoords(prevDisparity, filtered_img_coords[1]);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr currPC = pclHelpers::Vec3DToPointCloudXYZRGB(currWorldPts, filtered_img_coords[0], lImg);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr prevPC = pclHelpers::Vec3DToPointCloudXYZRGB(prevWorldPts, filtered_img_coords[1], prevLeftImage); 
        pclHelpers::performICP(prevPC, currPC, currPosition, currRotation, outPos, outRot);
        prevKp = currentKp;
        prevLeftImage.release();
        prevLeftImage = lImg.clone();
        prevDesc.release();
        prevDesc = currDesc.clone();
        prevDisparity.release();
        prevDisparity = currDisparity.clone();
        if( tParser.showStereoImages() == 27)
        {
            break;
        }
    }

    cv::destroyAllWindows();
}

