#include "common-includes.h"
#include "pcl-helpers.h"
#include "cv-helpers.h"
#include "TsukubaParser.h"


#define FX 615
#define FY 615
#define CX 320
#define CY 240

using namespace std;
using namespace cv;
using namespace xfeatures2d;
using namespace ximgproc;

//Left cam -> origin of coordinate system
Mat currPosition = (Mat1d(1, 3) << 0, 0, 0);
Mat currRotation = Mat::eye(3, 3, CV_64F);
Mat LGImage, RGImage;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr myPC;

void processFirstStereo(TsukubaParser& tParser, Ptr<SURF>& surf, cv::BFMatcher* correspondance_matcher)
{
    tParser.getNextStereoImages();
    cvtColor(tParser.getLImage(), LGImage, COLOR_BGRA2GRAY);
    cvtColor(tParser.getRImage(), RGImage, COLOR_BGRA2GRAY);
    vector<KeyPoint> kpLeft,kpRight;
    Mat descriptorLeft, descriptorRight;
    //Get features
    cvHelpers::getFeatures(surf, LGImage, kpLeft, descriptorLeft );
    cvHelpers::getFeatures(surf, RGImage, kpRight, descriptorRight );
    // TODO: rename inlier_points to filteredFeatures or something
    vector<vector<Point2d> > inlier_points =  cvHelpers::filterPoints(correspondance_matcher, kpLeft, descriptorLeft, kpRight, descriptorRight);
    vector<Point3d> points_3d = cvHelpers::get3DPoints(inlier_points.at(0), inlier_points.at(1), currPosition, currRotation);
    myPC = pclHelpers::Vec3DToPointCloudXYZRGB(points_3d, inlier_points.at(1), tParser.getLImage());
    pclHelpers::registerFirstPointCloud( myPC );
}

int main(int argc, char** argv)
{
    if (argc == 1 || argc > 2 )
    {
        cout << "Please run the program as NTSD-VisualOdometry <path-to-NTSD-root-directory>" << endl;
    }
    string DBPath = argv[1];
    TsukubaParser tParser(DBPath);

    //Algorithm pointers
    Ptr<SURF> surf = nullptr;
    Ptr<BFMatcher> correspondance_matcher = nullptr; 
    Ptr<StereoSGBM> left_matcher = nullptr;
    Ptr<StereoMatcher> right_matcher = nullptr;
    Ptr<DisparityWLSFilter> wls_filter = nullptr;
    cvHelpers::createFilters(surf, correspondance_matcher, left_matcher, right_matcher, wls_filter );
    //Vectors to store points
    vector< vector<Point2d> > filtered_points;
    vector<Point2d> filtered_pointsLeft, filtered_pointsRight;
    //Output position and rotation
    Mat outPos = (Mat1d(1, 3) << 0, 0, 0);
    Mat outRot = Mat::eye(3, 3, CV_64F);
    Mat descriptorLeft, descriptorRight;

    //TODO: add "Process first stereo image here". We have to set first image as frame of reference for all others
    cout<< "ProcessFirstStereo\n";
    processFirstStereo(tParser, surf, correspondance_matcher);
    cout<< "Loop\n";
    while( tParser.getIter() <= NTSD_DB_SIZE)
    {
        tParser.getNextStereoImages();
        cvtColor(tParser.getLImage(), LGImage, COLOR_BGRA2GRAY);
        cvtColor(tParser.getRImage(), RGImage, COLOR_BGRA2GRAY);
        //cout << LGImage.rows << " cols: " << LGImage.cols << endl;
        vector<KeyPoint> kpLeft,kpRight;
        //Get features
        cvHelpers::getFeatures(surf, LGImage, kpLeft, descriptorLeft );
        cvHelpers::getFeatures(surf, RGImage, kpRight, descriptorRight );
        // TODO: rename inlier_points to filteredFeatures or something
        vector<vector<Point2d> > inlier_points =  cvHelpers::filterPoints(correspondance_matcher, kpLeft, descriptorLeft, kpRight, descriptorRight);
        vector<Point3d> points_3d = cvHelpers::get3DPoints(inlier_points.at(0), inlier_points.at(1), currPosition, currRotation);
        //Get point cloud
        myPC = pclHelpers::Vec3DToPointCloudXYZRGB(points_3d, inlier_points.at(0), tParser.getLImage());
        //Do ICP
        pclHelpers::registerCurrentPointCloud(myPC);
        //Apply transform to currentPos and currentRot
        pclHelpers::performICP(currRotation, currPosition, outPos, outRot);
        currRotation = outRot.clone();
        currPosition = outPos.clone();
        if( tParser.showStereoImages() == 27)
        {
            break;
        }
    }

    cv::destroyAllWindows();
}

