//
// Created by experi on 2023/3/4.
//
#include "GlobalVar/inc/Logger.h"
#include "Visual/ArmorFinder/inc/FindArmorBox.h"
#include <eigen3/Eigen/Eigen>


#define FIND_ARMOR_JUDGE 0

// 判断两个灯条的角度差
static bool angelJudge(const LightBlob &light_blob_i,
                       const LightBlob &light_blob_j) {
    float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height
                    ? light_blob_i.rect.angle
                    : light_blob_i.rect.angle - 90;
    float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height
                    ? light_blob_j.rect.angle
                    : light_blob_j.rect.angle - 90;
    return abs(angle_i - angle_j) < 20;
}
// 判断两个灯条的高度差
static bool heightJudge(const LightBlob &light_blob_i,
                        const LightBlob &light_blob_j) {
    cv::Point2f centers = light_blob_i.rect.center - light_blob_j.rect.center;
    return abs(centers.y) < 30;
}
// 判断两个灯条的间距
static bool lengthJudge(const LightBlob &light_blob_i,
                        const LightBlob &light_blob_j) {
    double side_length;
    cv::Point2f centers = light_blob_i.rect.center - light_blob_j.rect.center;
    side_length = sqrt(centers.ddot(centers));
#if FIND_ARMOR_JUDGE
    LOGD("灯条间距 %.2f", side_length / light_blob_i.length);
#endif
    return (side_length / light_blob_i.length < 10 &&
            side_length / light_blob_i.length > 0.5);
}
// 判断两个灯条的长度比
static bool lengthRatioJudge(const LightBlob &light_blob_i,
                             const LightBlob &light_blob_j) {
#if FIND_ARMOR_JUDGE
    LOGD("长度比为 %.2f", light_blob_i.length / light_blob_j.length);
#endif
    return (light_blob_i.length / light_blob_j.length < 2.5 &&
            light_blob_i.length / light_blob_j.length > 0.4);
}

/* 判断两个灯条的错位度，不知道英文是什么！！！ */
static bool CuoWeiDuJudge(const LightBlob &light_blob_i,
                          const LightBlob &light_blob_j) {
    float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height
                    ? light_blob_i.rect.angle
                    : light_blob_i.rect.angle - 90;
    float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height
                    ? light_blob_j.rect.angle
                    : light_blob_j.rect.angle - 90;
    float angle = (angle_i + angle_j) / 2.0 / 180.0 * 3.14159265459;
    if (abs(angle_i - angle_j) > 90) {
        angle += 3.14159265459 / 2;
    }
    Eigen::Vector2f orientation(cos(angle), sin(angle));
    Eigen::Vector2f p2p(light_blob_j.rect.center.x - light_blob_i.rect.center.x,
                        light_blob_j.rect.center.y -
                        light_blob_i.rect.center.y);
    return abs(orientation.dot(p2p)) < 25;
}
// 判断装甲板方向
static bool boxAngleJudge(const LightBlob &light_blob_i,
                          const LightBlob &light_blob_j) {
    float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height
                    ? light_blob_i.rect.angle
                    : light_blob_i.rect.angle - 90;
    float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height
                    ? light_blob_j.rect.angle
                    : light_blob_j.rect.angle - 90;
    float angle = (angle_i + angle_j) / 2.0;
    if (abs(angle_i - angle_j) > 90) {
        angle += 90.0;
    }
    return (-120.0 < angle && angle < -60.0) || (60.0 < angle && angle < 120.0);
}
// 判断两个灯条是否可以匹配
static bool isCoupleLight(const LightBlob &light_blob_i,
                          const LightBlob &light_blob_j) {
    return
           lengthRatioJudge(light_blob_i, light_blob_j) &&
           lengthJudge(light_blob_i, light_blob_j) &&
           //           heightJudge(light_blob_i, light_blob_j) &&
           angelJudge(light_blob_i, light_blob_j) &&
           boxAngleJudge(light_blob_i, light_blob_j) &&
           CuoWeiDuJudge(light_blob_i, light_blob_j);
}

FindArmorBox::FindArmorBox(){
}

bool FindArmorBox::match_armor_a_boxes(const cv::Mat &src,
                                       const LightBlobs &light_blobs,
                                       ArmorBoxes &armor_boxes) {
    armor_boxes.clear();
    LOGD("light_blob.size : %zu", light_blobs.size());
    for (int i = 0; i < int(light_blobs.size()) - 1; ++i) {
        for (int j = i + 1; j < light_blobs.size(); ++j) {
            if (!isCoupleLight(light_blobs.at(i), light_blobs.at(j))) {
                continue;
            }
            cv::Rect2d rect_left = light_blobs.at(static_cast<unsigned long>(i))
                .rect.boundingRect();
            cv::Rect2d rect_right =
                light_blobs.at(static_cast<unsigned long>(j))
                    .rect.boundingRect();
            double min_x, min_y, max_x, max_y;
            min_x = fmin(rect_left.x, rect_right.x) - 4;
            max_x = fmax(rect_left.x + rect_left.width,
                         rect_right.x + rect_right.width) +
                    4;
            min_y = fmin(rect_left.y, rect_right.y) -
                    0.5 * (rect_left.height + rect_right.height) / 2.0;
            max_y = fmax(rect_left.y + rect_left.height,
                         rect_right.y + rect_right.height) +
                    0.5 * (rect_left.height + rect_right.height) / 2.0;
            if (min_x < 0 || max_x > src.cols || min_y < 0 ||
                max_y > src.rows) {
                continue;
            }
            if ((max_x - min_x) / (max_y - min_y) < 0.8)
                continue;
            LightBlobs pair_blobs = {light_blobs.at(i), light_blobs.at(j)};
            // SolveAngler另一个类
            // SolveAngler getdistance;
            // dis = getdistance.getDistance(pair_blobs[0].length);
            armor_boxes.emplace_back(
                cv::Rect2d(min_x, min_y, max_x - min_x, max_y - min_y),
                pair_blobs);
        }
    }
    return !armor_boxes.empty();
}

bool FindArmorBox::find_armor_box(cv::Mat &src, ArmorBox &box) {
    LightBlobs light_blobs; // 存储所有可能的灯条
    ArmorBoxes armor_boxes; // 装甲板候选区

    std::vector<cv::Mat> frames; // 选取的图像

    bool return_code = true;

    box.rect = cv::Rect2d(0, 0, 0, 0);
    box.id = -1;
    // 寻找所有可能的灯条

    if (!find_light_blob.find_light_blobs2(src, light_blobs)) {
        return_code = false;
    }
    if (!match_armor_a_boxes(src, light_blobs, armor_boxes)) {
        return_code = false;
    }


#if 1
    cv::Mat target_frame;
    src.copyTo(target_frame);
    if(!armor_boxes.empty()){
        cv::rectangle(target_frame, armor_boxes[0].rect, cv::Scalar(0, 0, 255));
    }
    cv::imshow("Target", target_frame);
#endif

    if(!armor_boxes.empty()){
        armor_boxes[0].get_time = clock();
        box = armor_boxes[0];
    }
    return return_code;
}
