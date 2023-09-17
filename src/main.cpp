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

const int dis_slider_max = 100;
const int block_slider_max = 255;
int dis_curr = 28;
int block_curr = 48 ;

Mat K= (Mat1d(3, 3) << FX, 0, CX, 0, FY, CY, 0, 0, 1);

//Left cam -> origin of coordinate system
Mat currPosition = (Mat1d(1, 3) << 0, 0, 0);
Mat currRotation = Mat::eye(3, 3, CV_64F);

//Right cam
Mat relativeRCamPosition = (Mat1d(1,3) << 10, 0, 0);
Mat relativeRCamRotation = Mat::eye(3,3,CV_64F);


void processFirstStereo(TsukubaParser& tParser, Ptr<SURF> surf, Ptr<DescriptorMatcher> matcher)
{
    Mat LGImage, RGImage;
    tParser.getNextStereoImages();
    cvtColor(tParser.getLImage(), LGImage, COLOR_BGRA2GRAY);
    cvtColor(tParser.getRImage(), RGImage, COLOR_BGRA2GRAY);
    vector<KeyPoint> kpLeft,kpRight;
    Mat descriptorLeft, descriptorRight;
    //Get features
    cvHelpers::getFeatures(surf, LGImage, kpLeft, descriptorLeft );
    cvHelpers::getFeatures(surf, RGImage, kpRight, descriptorRight );
    // TODO: rename inlier_points to filteredFeatures or something
    vector<vector<Point2d> > inlier_points =  cvHelpers::filterPoints(matcher, kpLeft, descriptorLeft, kpRight, descriptorRight);
    vector<Point3d> points_3d = cvHelpers::get3DPoints(inlier_points.at(0), inlier_points.at(1), currPosition, currRotation);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr myPC = pclHelpers::Vec3DToPointCloudXYZRGB(points_3d, inlier_points.at(1), tParser.getLImage());
    pclHelpers::registerFirstPointCloud( myPC );
}

int main(int argc, char** argv)
{
    if (argc == 1 || argc > 2 )
    {
        cout << "Please run the program as NTSD-VisualOdometry <path-to-NTSD-root-directory>" << endl;
        assert(0);
    }
    string DBPath = argv[1];
    TsukubaParser tParser(DBPath);

    Ptr<SURF> surf = SURF::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    Mat LGImage, RGImage;
    vector< vector<Point2f> > filtered_points;
    vector<Point2f> filtered_pointsLeft, filtered_pointsRight;
    //TODO: add "Process first stereo image here". We have to set first image as frame of reference for all others
    processFirstStereo(tParser, surf, matcher);
    while( tParser.getIter() <= NTSD_DB_SIZE)
    {
        tParser.getNextStereoImages();
        cvtColor(tParser.getLImage(), LGImage, COLOR_BGRA2GRAY);
        cvtColor(tParser.getRImage(), RGImage, COLOR_BGRA2GRAY);
        //cout << LGImage.rows << " cols: " << LGImage.cols << endl;
        vector<KeyPoint> kpLeft,kpRight;
        Mat descriptorLeft, descriptorRight;
        //Get features
        cvHelpers::getFeatures(surf, LGImage, kpLeft, descriptorLeft );
        cvHelpers::getFeatures(surf, RGImage, kpRight, descriptorRight );
        // TODO: rename inlier_points to filteredFeatures or something
        vector<vector<Point2d> > inlier_points =  cvHelpers::filterPoints(matcher, kpLeft, descriptorLeft, kpRight, descriptorRight);
        vector<Point3d> points_3d = cvHelpers::get3DPoints(inlier_points.at(0), inlier_points.at(1), currPosition, currRotation);
        //Get point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr myPC = pclHelpers::Vec3DToPointCloudXYZRGB(points_3d, inlier_points.at(1), tParser.getLImage());
        //Do ICP

        //Apply transform to currentPos and currentRot

        if( tParser.showStereoImages() == 27)
        {
            break;
        }
    }

    cv::destroyAllWindows();
}

