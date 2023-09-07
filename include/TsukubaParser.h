#ifndef TSUKUBA_PARSER_H
#define TSUKUBA_PARSES_H
#include "common-includes.h"
#define NTSD_DB_SIZE 1800;
#define NAME_BUF_SIZE 13

class TsukubaParser{
    public:
        TsukubaParser(string TsukubaPath);
        ~TsukubaParser();
        void getNextStereoImages();
        Mat getLImage();
        Mat getRImage();

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
