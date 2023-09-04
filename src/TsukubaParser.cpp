#include "TsukubaParser.h"

TsukubaParser::TsukubaParser(string TsukubaPath): imgDirPath(TsukubaPath), iter(1)
{
    if(imgDirPath.back() != '/')
    {
        imgDirPath.append("/");
    }
    imgDirPath.append("NewTsukubaStereoDataset/illumination/daylight/");
    namedWindow("Concat");
}


vector<Mat> TsukubaParser::getNextStereoImages()
{
    vector<Mat> OutputVec;
    assert(iter <= 1800);
    snprintf(LNameBuff, NAME_BUF_SIZE-1 ,"L_%05d.png",iter);
    snprintf(RNameBuff, NAME_BUF_SIZE-1 ,"R_%05d.png",iter);
    LFullPath = imgDirPath + string(LNameBuff);
    RFullPath = imgDirPath + string(RNameBuff);
    OutputVec.push_back(RImage);
    OutputVec.push_back(LImage);
    LImage = imread(LFullPath.c_str(), IMREAD_UNCHANGED);
    RImage = imread(LFullPath.c_str(), IMREAD_UNCHANGED);
    iter ++;
    return OutputVec;
}

void TsukubaParser::showStereoImages()
{
    vector<Mat> vec{LImage,RImage};
    Mat concatImage;
    hconcat(vec, concatImage);
    imshow("Concat", concatImage);
    waitKey(1);
}

int TsukubaParser::getIter()
{
    return iter;
}

TsukubaParser::~TsukubaParser()
{
    if(!LImage.empty())
    {
        LImage.deallocate();
    }
    
    if(!RImage.empty())
    {
        RImage.deallocate();
    }

    imgDirPath.erase();
    LFullPath.erase();
    RFullPath.erase();
    destroyWindow("Concat");
}