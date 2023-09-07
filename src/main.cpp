#include "common-includes.h"
#include "TsukubaParser.h"

#define FX 615
#define FY 615
#define CX 320
#define CY 240

const int dis_slider_max = 100;
const int block_slider_max = 255;
int dis_curr = 28;
int block_curr = 48 ;

Mat K= (Mat1d(3, 3) << FX, 0, CX, 0, FY, CY, 0, 0, 1);




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
    int ndisparities = 16 * 10;
    const int wsize = 5;
    TsukubaParser tParser(DBPath);

    Rect ROI;
    double matching_time, filtering_time;
    double solving_time = 0;

    Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(0, ndisparities, wsize);
    left_matcher->setMinDisparity(0);
    left_matcher->setNumDisparities(160);
    left_matcher->setSpeckleRange(7);
    left_matcher->setP1(200);
    left_matcher->setP2(500);
    left_matcher->setPreFilterCap(1);
    left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
    left_matcher->setUniquenessRatio(0);
    Ptr<ximgproc::DisparityWLSFilter> wls_filter = ximgproc::createDisparityWLSFilter(left_matcher);
    Ptr<StereoMatcher> right_matcher = ximgproc::createRightMatcher(left_matcher);


    wls_filter->setSigmaColor(2.f);
    Mat LGImage, RGImage, dImageRL, dImageLR, dImageF;
    while( tParser.getIter() <= 1800)
    {
        tParser.getNextStereoImages();
        LGImage = tParser.getLImage().clone();
        RGImage = tParser.getRImage().clone();

        // cvtColor(tParser.getLImage(), LGImage, COLOR_BGRA2GRAY);
        // cvtColor(tParser.getRImage(), RGImage, COLOR_BGRA2GRAY);
        Mat disImageLR(LGImage.rows, LGImage.cols, CV_16S);
        Mat disImageRL(LGImage.rows, LGImage.cols, CV_16S);
        Mat disImageF(LGImage.rows, LGImage.cols, CV_16S);

        left_matcher->compute(LGImage, RGImage, disImageLR);
        right_matcher->compute(RGImage, LGImage, disImageRL);
        wls_filter->filter(disImageLR, LGImage, disImageF, disImageRL);
        ximgproc::getDisparityVis(disImageLR, dImageLR);
        ximgproc::getDisparityVis(disImageRL, dImageRL);
        ximgproc::getDisparityVis(disImageF, dImageF);
        Rect ROI = wls_filter->getROI();
        
        dImageF = dImageF(ROI);
        Mat LGImage1 = LGImage(ROI); 
        imshow("disLR",dImageLR);
        imshow("disRL",LGImage1);
        imshow("disF", dImageF);
        if( tParser.showStereoImages() == 27)
        {
            break;
        }
    }

    cv::destroyAllWindows();
}

