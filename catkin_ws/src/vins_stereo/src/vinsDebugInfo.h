#ifndef VINSDEBUGINFO_H
#define VINSDEBUGINFO_H

struct ImageDebugInfo
{
    ImageDebugInfo():
        leftImage_index(0),
        rightImage_index(0),
        trackImage_index(0)
    {}
    int leftImage_index;
    int rightImage_index;
    int trackImage_index;
};


#endif // VINSDEBUGINFO_H
