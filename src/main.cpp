#include "common-includes.h"
#include "TsukubaParser.h"
#include "cv-helpers.h"

#define FX 615
#define FY 615
#define CX 320
#define CY 240

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
    while( tParser.getIter() <= 1800)
    {
        tParser.getNextStereoImages();
        cvtColor(tParser.getLImage(), LGImage, COLOR_BGRA2GRAY);
        cvtColor(tParser.getRImage(), RGImage, COLOR_BGRA2GRAY);
        //cout << LGImage.rows << " cols: " << LGImage.cols << endl;
        vector<KeyPoint> kpLeft,kpRight;
        Mat descriptorLeft, descriptorRight;
        cvHelpers::getFeatures(surf, LGImage, kpLeft, descriptorLeft );
        cvHelpers::getFeatures(surf, RGImage, kpRight, descriptorRight );
        // TODO: rename inlier_points to filteredFeatures or something
        vector<vector<Point2d> > inlier_points =  cvHelpers::filterPoints(matcher, kpLeft, descriptorLeft, kpRight, descriptorRight);
        cvHelpers::get3DPoints(inlier_points.at(0), inlier_points.at(1), currPosition, currRotation);
        
        if( tParser.showStereoImages() == 27)
        {
            break;
        }
    }

    cv::destroyAllWindows();
}

