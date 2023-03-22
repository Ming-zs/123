//
// Created by imca on 2023/3/4.
//

#ifndef FANSEARCH_H
#define FANSEARCH_H

#include <opencv2/opencv.hpp>

#include "Visual/ArmorOption/EnergyDestroyer/inc/KfPredict.h"

#define PI 3.14159


using FanLeaf = cv::Point2f;                      // 叶片Center
using Color = cv::Scalar;
using ContoursVec = std::vector<std::vector<cv::Point> >;   // 边缘点集合
using HierarchyVec = std::vector<cv::Vec4i>;        // 层级集合
using Hierarchy = cv::Vec4i;                        // 层级
using ContourPloy = ContoursVec;                    // 拟合集合
using FanCenterVec = std::vector<cv::Point>;           // 风车中心集合
using FanCenter = cv::Point2f;                      // 风车Center
using FanLeavesVec = FanCenterVec;                        // 风车叶片集合

class FanSearch {
  private:
    cv::Mat frame;
    cv::Mat thresh_mat;

    FanLeavesVec fan_leaves;
    FanCenter fan_center;
    FanLeaf last_leaf;

    cv::Scalar green;
    cv::Scalar yellow;
    cv::Scalar red;
    cv::Scalar cyan;

    KfPredict kf_predict;

    static void draw_rotated_rect(cv::Point2f * _ps, cv::Mat & _frame, cv::Scalar & _color);

    void draw_point(cv::Point point, cv::Mat & _frame, Color _color);

    /**
     * 获取风车中心
     * @param _ps 旋转矩形点集合
     * @param _hie 层级
     * @return 1:不是候选中心 2:不是风车中心 0:找到中心
     */
    static int get_fan_center(cv::Point2f * _ps, Hierarchy & _hie);

    /**
     * 获取风车叶片
     * @param _ps 旋转矩形点集合
     * @param _hie 层级
     * @return 1:不是候选叶片 2:不是风车叶片 0:找到中心
     */
    static int get_fan_leaf(cv::Point2f * _ps, Hierarchy & _hie);

    /**
     * 确认叶片
     * @param output 保存
     * @return
     */
    int identify_leaf(cv::Point2f & output);

  public:

    /**
     * 更新视频帧
     * @param frame
     */
    void fresh_frame(cv::Mat & frame);

    /**
     * 获取灯条的二直图
     * @return
     */
    int get_light();

    /**
     * 获取风车轮廓
     * @return
     */
    int get_fan_contours(cv::Point & output);

    void reset();


    FanSearch();
};

#endif // ROOT_IMCA_FANSEARCH_H
