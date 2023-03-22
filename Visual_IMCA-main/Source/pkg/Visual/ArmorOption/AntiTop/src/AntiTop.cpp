//
// Created by experi on 2023/3/4.
//

#include "Visual/ArmorOption/AntiTop/inc/AntiTop.h"
#include "GlobalVar/inc/GlobalVar.h"
#include "GlobalVar/inc/ControlFrame.h"

// 获取目标（帧）
AngleFrame AntiTop::getTarget(cv::Mat &frame) {
    ArmorBox armor_box;
    this->armor_finder->find_armor_box(frame, armor_box);
    mt_g_ssi.lock();
    // 射速
    uint8_t bulletSpeed = g_ssi.bulletSpeed;
    mt_g_ssi.unlock();
    //
    get_car_center(frame, g_ssi.currentPitch.getFloat(), armor_box, bulletSpeed);
    return getAngle(this->car_point.x, this->car_point.y);
}

void AntiTop::reset() {

}

int AntiTop::get_car_center(cv::Mat &frame, float cameraAngle,
                            ArmorBox &armor_box, float buttelSpeed) {

    if(armor_box.get_center().x > (double)frame.cols || armor_box.get_center().y > (double)frame.rows){
        return -1;
    }
    // DPI计算方法 f/fx = 1/dx (px/mm)
    double dx = abs(armor_box.get_center().x - (double)frame.cols / 2);
    double dy = abs(armor_box.get_center().y - (double)frame.rows / 2);
    double F = std::sqrt(dx*dx + dy*dy + pow(camera_f * camera_dpi, 2));
    double a = (car_width * F) / (2 * armor_box.get_box_distance());
    double b = a * sin(cameraAngle);
    double k = armor_box.get_box_angle();
    double the_same = sqrt(k*k*a*a + b*b);
    double delta_x = k * pow(a, 2) / the_same, delta_y = -b*b/the_same;

    cv::Point car_center_point = cv::Point(frame.cols / 2, armor_box.get_center().y + delta_y);

    if(car_center_point.x < 0 || car_center_point.x > frame.cols || car_center_point.y < 0 || car_center_point.y > frame.rows){
        return -2;
    }

    cv::Mat car_center;
    frame.copyTo(car_center);
    cv::circle(car_center, car_center_point, 1, cv::Scalar(0, 255, 255), 5);

    cv::imshow("CarCenter", car_center);

    this->car_point = cv::Point(car_center_point.x, (int)car_center_point.y + b);

    return 0;
}
AntiTop::AntiTop(ArmorFinder *_finder) {
    this->armor_finder = _finder;
}
AngleFrame AntiTop::getAngle(float x, float y) {
    double positionX = (x - MVCAMERA_CX) /MVCAMERA_FX;
    double positionY = (y - MVCAMERA_CY) / MVCAMERA_FY;
    AngleFrame angleFrame;
    angleFrame.pitch.setFloat(static_cast<float>(atan(positionY)*57.2957805f));
    angleFrame.yaw.setFloat(static_cast<float>(atan(positionX)*57.2957805f));
    return angleFrame;
}
