//
// Created by experi on 2023/3/4.
//

#ifndef ENERYDESTROYER_H
#define ENERYDESTROYER_H

#include "Visual/ArmorOption/Interface/VisualOption.h"
#include "Visual/ArmorFinder/inc/FindArmorBox.h"


#include "Visual/ArmorOption/EnergyDestroyer/inc/FanSearch.h"

using PredictVec = std::vector<double>;         // 预测
using ContoursVec = std::vector<std::vector<cv::Point> >;   // 边缘点集合
using HierarchyVec = std::vector<cv::Vec4i>;        // 层级集合
using Hierarchy = cv::Vec4i;                        // 层级
using ContourPloy = ContoursVec;                    // 拟合集合

using FanLeaf = cv::Point2f;                      // 叶片Center
using Color = cv::Scalar;


class EnergeDestory : public VisualOption{
  private:
    FanSearch fan_search;

    cv::Point predict_point;

    AngleFrame getAngle(float x, float y);

  public:
    void reset() override;

    AngleFrame getTarget(cv::Mat & frame) override;

};

#endif // ROOT_IMCA_ENERYDESTROYER_H
