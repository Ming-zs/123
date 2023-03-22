//
// Created by ica on 2023/2/19.
//

#include "VideoWapper.h"

VideoWrapper::VideoWrapper(const std::string &filename) {
    video.open(filename);
}

VideoWrapper::~VideoWrapper() = default;


bool VideoWrapper::init() {
    return video.isOpened();
}

bool VideoWrapper::read(cv::Mat &src) {
    return video.read(src);
}
