#include "common-includes.h"
#define NAME_BUF_SIZE 13
int main()
{
    char LNameBuff[NAME_BUF_SIZE];
    char RNameBuff[NAME_BUF_SIZE];
    size_t iter = 1;
    while(iter <= 1800)
    {
        snprintf(LNameBuff, NAME_BUF_SIZE-1 ,"L_%05u.png",iter);
        snprintf(RNameBuff, NAME_BUF_SIZE-1 ,"R_%05u.png",iter);
        cout << string(LNameBuff) << endl;
        cout << string(RNameBuff) << endl;
    
    }
}