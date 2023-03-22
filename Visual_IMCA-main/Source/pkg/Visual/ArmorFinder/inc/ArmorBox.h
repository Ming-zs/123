#ifndef ARMOR_BOX_H__
#define ARMOR_BOX_H__

#include "Visual/ArmorFinder/inc/LightBlob.h"

// 装甲板类
class ArmorBox{
  public:
    cv::Rect2d rect;  // 矩形
    LightBlobs light_blobs;  // 灯条
    clock_t get_time; // 搜索板子的时刻
    int id;  // 板子id

    /**
     * @brief 装甲板类的构造函数
     * 
     * @param _pos 装甲板在图像上的位置
     * @param _blobs 该装甲板的灯条
     * @param _color 对方的颜色
     * @param _id 对方的ID
     */
    explicit ArmorBox(const cv::Rect & _pos=cv::Rect2d(), const LightBlobs & _blobs=LightBlobs(), int _id = 0);

    // 获取装甲板中心
    cv::Point2f get_center() const;
    // 获取两个灯条中心间距
    double get_blobs_distance() const;
    // 获取灯条中心距和灯条长度的比值
    double length_distance_ratio() const;
    // 获取装甲板到摄像头的距离
    double get_box_distance() const;
    // 获取装甲板角度，出错返回-1
    double get_box_angle() const;

// TODO: 这里会有一个装甲板优先级的类，用重载做
};

// ArmorBox的Vector的集合
using ArmorBoxes = std::vector<ArmorBox>;

#endif