#ifndef VINSDEBUGINFO_H
#define VINSDEBUGINFO_H

struct ImageDebugInfo
{
    ImageDebugInfo():
        leftImage_index(0),
        rightImage_index(0),
        trackImage_index(0),
        track_cost_time(0.0f)
    {}
    int leftImage_index;
    int rightImage_index;
    int trackImage_index;
    double track_cost_time;
};


#endif // VINSDEBUGINFO_H
