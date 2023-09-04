#include "common-includes.h"
#include "TsukubaParser.h"
int main(int argc, char** argv)
{
    if (argc == 1 || argc > 2 )
    {
        cout << "Please run the program as NTSD-VisualOdometry <path-to-NTSD-root-directory>" << endl;
        assert(0);
    }
    string DBPath = argv[1];

    TsukubaParser tParser(DBPath);

    while( tParser.getIter() <= 1800)
    {
        tParser.getNextStereoImages();
        tParser.showStereoImages();
    
    }
}

