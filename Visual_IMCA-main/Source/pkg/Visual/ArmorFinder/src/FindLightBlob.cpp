//
// Created by experi on 2023/3/4.
//

#include "Visual/ArmorFinder/inc/FindLightBlob.h"
#include "GlobalVar/inc/RobotState.h"
#include "GlobalVar/inc/GlobalVar.h"

// 旋转矩形的长宽比
static double lw_rate(const cv::RotatedRect &rect) {
    return rect.size.height > rect.size.width
           ? rect.size.height / rect.size.width
           : rect.size.width / rect.size.height;
}
// 轮廓面积和其最小外接矩形面积之比
static double areaRatio(const std::vector<cv::Point> &contour,
                        const cv::RotatedRect &rect) {
    return cv::contourArea(contour) / rect.size.area();
}
// 判断轮廓是否为一个灯条
static bool isValidLightBlob(const std::vector<cv::Point> &contour,
                             const cv::RotatedRect &rect) {
#if JUDGE_LIGHT_BAR
    LOGD("lw_rate %.2f", lw_rate(rect));
    LOGD("rect.size.area() : %.2f", rect.size.area());
    LOGD("areaRatio(contour, rect): %.2f", areaRatio(contour, rect));
    LOGD("++++++++++++++++");
#endif
    return (1.2 < lw_rate(rect) && lw_rate(rect) < 10) &&
           //           (rect.size.area() < 3000) &&
           ((rect.size.area() < 50 && areaRatio(contour, rect) > 0.4) ||
            (rect.size.area() >= 50 && areaRatio(contour, rect) > 0.6));
}
// 判断灯条颜色(此函数可以有性能优化).
static uint8_t get_blob_color(const cv::Mat &src,
                                 const cv::RotatedRect &blobPos) {
    auto region = blobPos.boundingRect();
    region.x -= fmax(3, region.width * 0.1);
    region.y -= fmax(3, region.height * 0.05);
    region.width += 2 * fmax(3, region.width * 0.1);
    region.height += 2 * fmax(3, region.height * 0.05);
    region &= cv::Rect(0, 0, src.cols, src.rows);
    cv::Mat roi = src(region);
    int red_cnt = 0, blue_cnt = 0;
    for (int row = 0; row < roi.rows; row++) {
        for (int col = 0; col < roi.cols; col++) {
            red_cnt += roi.at<cv::Vec3b>(row, col)[2];
            blue_cnt += roi.at<cv::Vec3b>(row, col)[0];
        }
    }
    if (red_cnt > blue_cnt) {
        return RED;
    } else {
        return BLUE;
    }
}
// 判断两个灯条区域是同一个灯条
static bool isSameBlob(LightBlob blob1, LightBlob blob2) {
    auto dist = blob1.rect.center - blob2.rect.center;
    return (dist.x * dist.x + dist.y * dist.y) < 9;
}

// 开闭运算
static void imagePreProcess(cv::Mat &src) {
    static cv::Mat kernel_erode =
        getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    erode(src, src, kernel_erode);

    static cv::Mat kernel_dilate =
        getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    dilate(src, src, kernel_dilate);

    static cv::Mat kernel_dilate2 =
        getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    dilate(src, src, kernel_dilate2);

    static cv::Mat kernel_erode2 =
        getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    erode(src, src, kernel_erode2);
}

FindLightBlob::FindLightBlob() {
    this->element =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
    this->element2 =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
}

//寻找灯条1
bool FindLightBlob::find_light_blobs(const cv::Mat &src,
                                     LightBlobs &light_blobs) {
    cv::Mat color_channel;
    cv::Mat src_bin_light, src_bin_dim;
    std::vector<cv::Mat> channels; // 通道拆分

    mt_g_ssi.lock();
    cv::split(src, channels);        /************************/
    if (g_ssi.robotColor != BLUE) { /*                      */
        color_channel = channels[0]; /* 根据目标颜色进行通道提取 */
    } else if (g_ssi.robotColor != RED) { /*                      */
        color_channel = channels[2];       /************************/
    }

    int light_threshold;
    if (g_ssi.robotColor != BLUE) {
        light_threshold = 225;
    } else {
        light_threshold = 200;
    }
    mt_g_ssi.unlock();
    cv::threshold(color_channel, src_bin_light, light_threshold, 255,
                  cv::THRESH_BINARY); // 二值化对应通道
    if (src_bin_light.empty())
        return false;
    imagePreProcess(src_bin_light); // 开闭运算

    cv::threshold(color_channel, src_bin_dim, 140, 255,
                  cv::THRESH_BINARY); // 二值化对应通道
    if (src_bin_dim.empty())
        return false;
    imagePreProcess(src_bin_dim); // 开闭运算

    if (src_bin_light.size() == cv::Size(1024, 600) /*&& show_light_blobs*/) {
#if SHOW_IMG
        imshow("bin_light", src_bin_light);
        imshow("bin_dim", src_bin_dim);
#endif
    }
    // 使用两个不同的二值化阈值同时进行灯条提取，减少环境光照对二值化这个操作的影响。
    // 同时剔除重复的灯条，剔除冗余计算，即对两次找出来的灯条取交集。
    std::vector<std::vector<cv::Point>> light_contours_light,
        light_contours_dim;
    LightBlobs light_blobs_light, light_blobs_dim;
    std::vector<cv::Vec4i> hierarchy_light, hierarchy_dim;
    cv::findContours(src_bin_light, light_contours_light, hierarchy_light,
                     cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
    cv::findContours(src_bin_dim, light_contours_dim, hierarchy_dim,
                     cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
    for (int i = 0; i < light_contours_light.size(); i++) {
        if (hierarchy_light[i][2] == -1) {
            cv::RotatedRect rect = cv::minAreaRect(light_contours_light[i]);
            if (isValidLightBlob(light_contours_light[i], rect)) {
                light_blobs_light.emplace_back(
                    rect, areaRatio(light_contours_light[i], rect));
            }
        }
    }
    for (int i = 0; i < light_contours_dim.size(); i++) {
        if (hierarchy_dim[i][2] == -1) {
            cv::RotatedRect rect = cv::minAreaRect(light_contours_dim[i]);
            if (isValidLightBlob(light_contours_dim[i], rect)) {
                light_blobs_dim.emplace_back(
                    rect, areaRatio(light_contours_dim[i], rect));
            }
        }
    }
    std::vector<int> light_to_remove, dim_to_remove;
    for (int l = 0; l != light_blobs_light.size(); l++) {
        for (int d = 0; d != light_blobs_dim.size(); d++) {
            if (isSameBlob(light_blobs_light[l], light_blobs_dim[d])) {
                if (light_blobs_light[l].area_ratio >
                    light_blobs_dim[d].area_ratio) {
                    dim_to_remove.emplace_back(d);
                } else {
                    light_to_remove.emplace_back(l);
                }
            }
        }
    }
    sort(light_to_remove.begin(), light_to_remove.end(),
         [](int a, int b) { return a > b; });
    sort(dim_to_remove.begin(), dim_to_remove.end(),
         [](int a, int b) { return a > b; });
    for (auto x : light_to_remove) {
        light_blobs_light.erase(light_blobs_light.begin() + x);
    }
    for (auto x : dim_to_remove) {
        light_blobs_dim.erase(light_blobs_dim.begin() + x);
    }
    for (const auto &light : light_blobs_light) {
        light_blobs.emplace_back(light);
    }
    for (const auto &dim : light_blobs_dim) {
        light_blobs.emplace_back(dim);
    }
    return light_blobs.size() >= 2;
}

//寻找灯条2
bool FindLightBlob::find_light_blobs2(const cv::Mat &src,
                                      LightBlobs &light_blobs) {
    mt_g_ssi.lock();
    if (g_ssi.robotColor != RED) {
        cv::Mat thres_whole;
        std::vector<cv::Mat> splited;
        cv::split(src, splited);
        cv::cvtColor(src, thres_whole, cv::COLOR_BGR2GRAY);
        cv::threshold(thres_whole, thres_whole, 60, 255, cv::THRESH_BINARY);
        // cv::imshow("thres_whole", thres_whole);
        cv::subtract(splited[2], splited[0], color);
        cv::threshold(color, color, 40, 255, cv::THRESH_BINARY); // 46
        cv::erode(color, color, element2);
        cv::dilate(color, color, element, cv::Point(-1, -1), 2);
        // cv::imshow("color", color);
        _max_color = color & thres_whole;
        cv::dilate(_max_color, _max_color, element2);

    } else {
        cv::Mat thres_whole;
        std::vector<cv::Mat> splited;
        cv::split(src, splited);
        cv::cvtColor(src, thres_whole, cv::COLOR_BGR2GRAY);
        cv::threshold(thres_whole, thres_whole, 65, 255, cv::THRESH_BINARY);
        cv::subtract(splited[0], splited[2], _max_color);
        cv::threshold(_max_color, _max_color, 46, 255, cv::THRESH_BINARY);
        cv::dilate(_max_color, _max_color, element);

        _max_color = _max_color & thres_whole;
        cv::dilate(_max_color, _max_color, element2);
    }
    mt_g_ssi.unlock();
    std::vector<std::vector<cv::Point>> light_contours_light,
        light_contours_dim;
    LightBlobs light_blobs_light, light_blobs_dim;
    std::vector<cv::Vec4i> hierarchy_light, hierarchy_dim;
    cv::findContours(_max_color, light_contours_light, hierarchy_light,
                     cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat frame = src.clone();
    for (int i = 0; i < light_contours_light.size(); i++) {
        if (hierarchy_light[i][2] == -1) {
            cv::RotatedRect rect = cv::minAreaRect(light_contours_light[i]);
            if (isValidLightBlob(light_contours_light[i], rect)) {
                light_blobs_light.emplace_back(
                    rect, areaRatio(light_contours_light[i], rect));
            }
        }
    }
    for (const auto &light : light_blobs_light) {
        light_blobs.emplace_back(light);
    }
    return light_blobs.size() >= 2;
}

//绘制旋转矩形（装甲板大小优先级）
//todo 装甲板中心优先级
void FindLightBlob::draw_rotated_rect(const cv::Mat & frame, cv::RotatedRect & rect){
    cv::Point2f rect_point[4];
    rect.points(rect_point);
    for(int i = 1; i <= 4; i++){
        cv::line(frame, rect_point[i - 1], rect_point[i % 4], cv::Scalar(0, 0, 255), 2);
    }
}