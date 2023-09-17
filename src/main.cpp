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

Mat currTL = (Mat1d(1,3) << 0,0,0);
Mat currTR = (Mat1d(1,3) << 10.f,0,0 );
Mat currRL = (Mat1d::eye(3,3));
Mat currRR = (Mat1d::eye(3,3));




static void on_trackbar_1( int, void* )
{
    return;
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
    while( tParser.getIter() <= 1800)
    {
        tParser.getNextStereoImages();
        cvtColor(tParser.getLImage(), LGImage, COLOR_BGRA2GRAY);
        cvtColor(tParser.getRImage(), RGImage, COLOR_BGRA2GRAY);
        vector<KeyPoint> kpLeft,kpRight;
        Mat descriptorLeft, descriptorRight;
        cout<<"1\n";
        cvHelpers::getFeatures(surf, LGImage, kpLeft, descriptorLeft );
        cout<<"1\n";
        cvHelpers::getFeatures(surf, RGImage, kpRight, descriptorRight );
        cout<<"1\n";
        cvHelpers::filterPoints(matcher, kpLeft, descriptorLeft, kpRight, descriptorRight);
        cout<<"1\n";
        if( tParser.showStereoImages() == 27)
        {
            break;
        }
    }

    cv::destroyAllWindows();
}

