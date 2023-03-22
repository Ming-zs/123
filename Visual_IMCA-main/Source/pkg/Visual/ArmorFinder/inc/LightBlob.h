#ifndef LIGHT_BLOB_H__
#define LIGHT_BLOB_H__

//#include <opencv4/opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

/**
 * @brief 灯条类
 *
 */
class LightBlob{
  public:
    cv::RotatedRect rect;   //灯条位置
    double area_ratio;      // 灯条在图像中的面积与外接矩形面积的比
    double length;          //灯条长度

    /**
     * @brief 灯条类
     *
     * @param _rect 灯条位置，类型为cv的旋转矩阵
     * @param _ratio 灯条在图像中的面积与外接矩形面积的比
     */
    LightBlob(cv::RotatedRect &_rect, double _ratio) : rect(_rect), area_ratio(_ratio) {
        length = fmax(rect.size.height, rect.size.width);
    };
    LightBlob() = default;
};

// LightBlob的vector集合
using LightBlobs = std::vector<LightBlob>;

#endif