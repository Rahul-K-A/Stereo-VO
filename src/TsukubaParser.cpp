#include "TsukubaParser.h"

TsukubaParser::TsukubaParser(string TsukubaPath): imgDirPath(TsukubaPath), iter(1)
{
    if(imgDirPath.back() != '/')
    {
        imgDirPath.append("/");
    }
    imgDirPath.append("NewTsukubaStereoDataset/illumination/fluorescent/");
    cout << "ImgDir " << imgDirPath << endl;
    namedWindow("Concat");
}


void TsukubaParser::getNextStereoImages()
{
    assert(iter <= 1800);
    snprintf(LNameBuff, NAME_BUF_SIZE-1 ,"L_%05d.png",iter);
    snprintf(RNameBuff, NAME_BUF_SIZE-1 ,"R_%05d.png",iter);
    LFullPath = imgDirPath + string(LNameBuff);
    RFullPath = imgDirPath + string(RNameBuff);
    LImage = imread(LFullPath.c_str(), 0);
    RImage = imread(RFullPath.c_str(), 0);
    iter ++;
}


Mat TsukubaParser::getLImage()
{
    return LImage;
}


Mat TsukubaParser::getRImage()
{
    return RImage;
}



int TsukubaParser::showStereoImages()
{
    vector<Mat> vec{LImage,RImage};
    Mat concatImage;
    hconcat(vec, concatImage);
    imshow("Concat", concatImage);
    return waitKey(1);
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