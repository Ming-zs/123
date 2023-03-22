//
// Created by experi on 2023/3/4.
//
#include "Visual/ArmorOption/EnergyDestroyer/inc/EnergyDestroyer.h"

#include "GlobalVar/inc/ControlFrame.h"

void EnergeDestory::reset() {
    this->fan_search.reset();
}

AngleFrame EnergeDestory::getTarget(cv::Mat &frame) {
    fan_search.fresh_frame(frame);
    fan_search.get_light();
    if(fan_search.get_fan_contours(this->predict_point) != 0) {
        this->predict_point = cv::Point(frame.cols / 2, frame.rows / 2);
    }
    return getAngle(this->predict_point.x, this->predict_point.y);
}
AngleFrame EnergeDestory::getAngle(float x, float y) {
    double positionX = (x - MVCAMERA_CX) /MVCAMERA_FX;
    double positionY = (y - MVCAMERA_CY) / MVCAMERA_FY;
    AngleFrame angleFrame;
    angleFrame.pitch.setFloat(static_cast<float>(atan(positionY)*57.2957805f));
    angleFrame.yaw.setFloat(static_cast<float>(atan(positionX)*57.2957805f));
    return angleFrame;
}
