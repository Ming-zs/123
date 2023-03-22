#ifndef VISUAL_OPTION_H__
#define VISUAL_OPTION_H__

#include "GlobalVar/inc/ControlFrame.h"
#include "Serial/FoundationType/inc/FLOAT32.h"
#include "GlobalVar/inc/GlobalVar.h"

#include <opencv2/opencv.hpp>

struct AngleFrame{
    FLOAT32 yaw;
    FLOAT32 pitch;
};

class VisualOption{
protected:

public:
    virtual AngleFrame getTarget(cv::Mat & frame) = 0;

    virtual void reset() = 0;
};

#endif