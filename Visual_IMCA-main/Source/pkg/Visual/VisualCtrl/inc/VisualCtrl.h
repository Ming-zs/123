//
// Created by experi on 2023/3/4.
//

#ifndef VISUAL_CTRL_H__
#define VISUAL_CTRL_H__

#include "Visual/ArmorFinder/inc/FindArmorBox.h"
#include "Visual/ArmorOption/AntiTop/inc/AntiTop.h"
#include "Visual/ArmorOption/EnergyDestroyer/inc/EnergyDestroyer.h"
#include "GlobalVar/inc/RobotState.h"

#include <memory>

class VisualCtrl{
  private:

    ArmorFinder* armor_finder;
    VisualOption * visual_option;
    AntiTop * anti_top;
    EnergeDestory * energe_destory;

    uint8_t current_state = ROBOT_FREE;

    cv::VideoWriter video_writer;
  public:
    VisualCtrl(std::string save_path);

    void switchOption(uint8_t state);

    void saveVideos(const cv::Mat &gimbal_src);

    AngleFrame run(cv::Mat & frame);
};

#endif // ROOT_IMCA_VISUALCTRL_H
