#ifndef  FIND_LIGHT_BLOB_H__
#define FIND_LIGHT_BLOB_H__

#include "Visual/ArmorFinder/inc/ArmorBox.h"

// 搜索灯条的类
class FindLightBlob{
    friend class FindArmorBox;
  private:

    cv::Mat _max_color;
    cv::Mat color;

    cv::Mat element;
    cv::Mat element2;

    uint8_t enemy_color;

    /**
     * @brief 搜索灯条函数 NO1
     * 
     * @param src 原始图像
     * @param light_blobs 已找到的灯条储存在此
     * @return 如果找到返回true 
     */
    bool find_light_blobs(const cv::Mat &src, LightBlobs &light_blobs);

    /**
     * @brief 搜索灯条函数 NO2
     * 
     * @param src 原始图像
     * @param light_blobs 已找到的灯条储存在此
     * @return 如果找到返回 true 
     */
    bool find_light_blobs2(const cv::Mat &src, LightBlobs &light_blobs);

    void draw_rotated_rect(const cv::Mat & frame, cv::RotatedRect & rect);
  public:
    FindLightBlob();

};

#endif