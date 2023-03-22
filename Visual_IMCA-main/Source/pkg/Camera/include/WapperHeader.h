//
// Created by ica on 2023/2/19.
//

#ifndef WAPPERHEADER_H__
#define WAPPERHEADER_H__


#include <opencv2/core/core.hpp>
#include <memory>

/**
 * @brief 一个接口类，用于提供视频的来源是从相机还是从文件
 */
class WrapperHead {
  public:
    virtual ~WrapperHead() = default;;
    virtual bool init() = 0;
    virtual bool read(cv::Mat &src) = 0;
};

using WrapperHeadPtr = std::shared_ptr<WrapperHead>;

#endif // WAPPERHEADER_H__
