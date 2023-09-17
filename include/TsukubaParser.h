#ifndef TSUKUBA_PARSER_H
#define TSUKUBA_PARSES_H
#include "common-includes.h"
#define NTSD_DB_SIZE 1800
#define NAME_BUF_SIZE 13

class TsukubaParser{
    public:
        TsukubaParser(string TsukubaPath);
        ~TsukubaParser();
        void getNextStereoImages();
        Mat getLImage();
        Mat getRImage();
        static Mat getCameraMatrix()
        {
            return (Mat1d(3,3) << 615.f, 0.f, 320.f,         // fx, 0, cx
                                    0.f, 615.f, 240.f,       // 0, fy, cy,
                                    0,   0,     1 );         // 0, 0 , 1
        };

        static Mat getRightCamRelativeTransform()
        {
            return ( Mat1d(4, 4) << 1, 0, 0, 10,   //Baseline -> 10cm
                                    0, 1, 0, 0,
                                    0, 0, 1, 0,
                                    0, 0, 0, 1);
        }

        /// @brief Returns rows and columns of the images in NTSD
        /// @return vector<int> in the format {row, column}
        static vector<int> getImageShape()
        {
            return vector<int>{480, 640};
        }

        int showStereoImages();
        int getIter();
    
    private:
        string imgDirPath;
        int iter;
        string LFullPath;
        string RFullPath;
        char LNameBuff[NAME_BUF_SIZE];
        char RNameBuff[NAME_BUF_SIZE];
        Mat LImage;
        Mat RImage;

};


#endif //TSUKUBA_PARSER_H
