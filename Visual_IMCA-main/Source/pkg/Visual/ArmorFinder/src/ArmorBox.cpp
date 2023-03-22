//
// Created by experi on 2023/3/4.
//
#include "Visual/ArmorFinder/inc/ArmorBox.h"

#include <opencv2/opencv.hpp>

ArmorBox::ArmorBox(const cv::Rect & _pos, const LightBlobs & _blobs, int _id):
    rect(_pos), id(_id){
    this->light_blobs = _blobs;
}

cv::Point2f ArmorBox::get_center() const {
    return {
        static_cast<float>(rect.x + rect.width / 2),
        static_cast<float>(rect.y + rect.height / 2)
    };
}

double ArmorBox::get_blobs_distance() const{
    if (light_blobs.size() == 2) {
        auto &x = light_blobs[0].rect.center;
        auto &y = light_blobs[1].rect.center;
        return sqrt((x.x - y.x) * (x.x - y.x) + (x.y - y.y) * (x.y - y.y));
    } else {
        return 0;
    }
}

double ArmorBox::length_distance_ratio() const{
    if (light_blobs.size() == 2) {
        return fmax(light_blobs[0].length, light_blobs[1].length)
               / get_blobs_distance();
    } else {
        return 100;
    }
}

double ArmorBox::get_box_distance() const{
    if (light_blobs.size() == 2) {
        return 1758.3 / 2 / fmax(light_blobs[0].length, light_blobs[1].length);
    } else {
        return 1758.3 / rect.height;
    }
}

double ArmorBox::get_box_angle() const{
    if (light_blobs.size() == 2) {
        if (light_blobs[0].rect.center.x >= light_blobs[1].rect.center.x)
            return (light_blobs[0].rect.center.y - light_blobs[1].rect.center.y) / (light_blobs[0].rect.center.x - light_blobs[1].rect.center.x);
        else
            return (light_blobs[1].rect.center.y - light_blobs[0].rect.center.y) / (light_blobs[1].rect.center.x - light_blobs[0].rect.center.x);
    }
    else
        return -1.0;
}