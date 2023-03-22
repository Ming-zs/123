//
// Created by ica on 2023/2/19.
//

#ifndef VIDEOWAPPER_H__
#define VIDEOWAPPER_H__


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "WapperHeader.h"


class VideoWrapper:public WrapperHead {
public:
    explicit VideoWrapper(const std::string& filename);
    ~VideoWrapper() override;

    /**
     * @brief initialize cameras
     * @return bool value: whether it success
     */
    bool init() final;


    /**
     * @brief read images from camera
     * @param src_left : output source video of left camera
     * @param src_right : output source video of right camera
     * @return bool value: whether the reading is successful
     */
    bool read(cv::Mat &src) final;
private:
    cv::VideoCapture video;

};

#endif // VIDEOWAPPER_H__
