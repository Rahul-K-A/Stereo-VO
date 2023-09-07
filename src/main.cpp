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
    Ptr<SURF> surf = SURF::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);

    wls_filter->setSigmaColor(2.f);
    Mat LGImage, RGImage, dImageRL, dImageLR, dImageF;
    Mat d1, d2;
    Mat p1, p2;

    while( tParser.getIter() <= 1800)
    {
        tParser.getNextStereoImages();

        cvtColor(tParser.getLImage(), LGImage, COLOR_BGRA2GRAY);
        cvtColor(tParser.getRImage(), RGImage, COLOR_BGRA2GRAY);
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
        vector<KeyPoint> kp1,kp2;
        vector<vector<DMatch>> knn_matches;
        vector<DMatch> good_matches;

        surf->detectAndCompute(LGImage1, noArray(), kp1, d1);
        surf->detectAndCompute(RGImage, noArray(), kp2, d2);

        matcher->knnMatch(d1,d2,knn_matches,2);

        // //-- Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.31f;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                good_matches.push_back(knn_matches[i][0]);
            }
        }

        if(good_matches.size() < 5)
        {
            cout<<"WARNING, num matches less than minimum" << tParser.getIter() -1 << "\n";
        }
        // -- Draw matches
        Mat img_matches;
        drawMatches( LGImage1, kp1, RGImage, kp2, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        //imshow("disLR",img_matches);

        // imshow("disRL",LGImage1);
        // imshow("disF", dImageF);
        // if( tParser.showStereoImages() == 27)
        // {
        //     break;
        // }
    }

    cv::destroyAllWindows();
}

