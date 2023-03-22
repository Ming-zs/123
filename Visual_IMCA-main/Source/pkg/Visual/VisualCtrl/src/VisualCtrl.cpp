//
// Created by experi on 2023/3/4.
//
#include "Visual/VisualCtrl/inc/VisualCtrl.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

std::string getfilename(){
    time_t currentTime;
    struct tm* timePointer;
    time(&currentTime);
    timePointer = localtime((&currentTime));
    return std::to_string(timePointer->tm_year+1980)+"-"+
           std::to_string(timePointer->tm_mon+1)+"-"+
           std::to_string(timePointer->tm_mday)+"-"+
           std::to_string(timePointer->tm_hour)+":"+
           std::to_string(timePointer->tm_min)+":"+
           std::to_string(timePointer->tm_sec);
}
//可视化控制
VisualCtrl::VisualCtrl(std::string save_path) {
    std::string file_name = getfilename() + ".mp4v";
    this->video_writer.open(save_path + "/" + file_name, 0x7634706d, 90, cv::Size(1024, 800), true);

    this->visual_option = nullptr;
    this->armor_finder = new ArmorFinder;
    this->energe_destory = new EnergeDestory;
    this->anti_top = new AntiTop(this->armor_finder);
}

// 切换模式使用
void VisualCtrl::switchOption(uint8_t state) {
    if(state != this->current_state){
        // 不识别装甲模式
        if(state == ROBOT_FREE){
            this->visual_option = nullptr;
        }
        // 装甲板识别模式
        else if(state == ROBOT_ANTITOP_MODE){
            this->visual_option = this->anti_top;
        }
        // 能量机关模式
        else if(state == ROBOT_ENERGY_MODE){
            this->visual_option = this->energe_destory;
        }
        // 装甲板跟踪
        else if(state == ROBOT_TRACKER_MODE){
            // TODO: 跟踪
        }
        this->current_state = state;
    }
}
AngleFrame VisualCtrl::run(cv::Mat &frame) {
    if(this->visual_option == nullptr){
        AngleFrame angleFrame;
        angleFrame.yaw.setFloat(0);
        angleFrame.pitch.setFloat(0);
        return angleFrame;
    }
    /*******根据上面的识别模式通过getTarget捕获图像信息*********/
    return this->visual_option->getTarget(frame);
}
void VisualCtrl::saveVideos(const cv::Mat &gimbal_src) {
    if (!gimbal_src.empty()) {
        video_writer.write(gimbal_src);
    } else return;
}
