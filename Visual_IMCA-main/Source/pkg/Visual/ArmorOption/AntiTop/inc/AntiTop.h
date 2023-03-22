//
// Created by experi on 2023/3/4.
//

#ifndef EXPERI_ANTITOP_H
#define EXPERI_ANTITOP_H

#include "Visual/ArmorOption/Interface/VisualOption.h"
#include "Visual/ArmorFinder/inc/FindArmorBox.h"

using AntiTopOutput = std::vector<cv::Point>;
using AntiTopPoints = AntiTopOutput;

using ArmorBoxesInTime = std::map<clock_t, cv::Point>;

class AntiTop : public VisualOption{
  private:

    float camera_f = 6.0; // 相机焦距
    float camera_dpi = 335; // TODO: 这个是内参的数据，需要修改
    float car_width = 340.0; // 车的宽度

    cv::Point car_point;

    FindArmorBox find_armor_box;

    ArmorBoxesInTime armor_boxes_in_time;

    ArmorFinder* armor_finder;

    int get_car_center(cv::Mat & frame, float cameraAngle, ArmorBox & armor_box, float buttelSpeed);

    AngleFrame getAngle(float x, float y);
  public:

    AntiTop(ArmorFinder* _finder);

    void reset();

    AngleFrame getTarget(cv::Mat & frame);
};

#endif //EXPERI_ANTITOP_H
