//
// Created by imca on 2023/3/4.
//

#include "Visual/ArmorOption/EnergyDestroyer/inc/FanSearch.h"

float get_two_point_dis(const cv::Point2f& _p1, const cv::Point2f& _p2){
    return (float)sqrt(pow(abs(_p1.x - _p2.x), 2) + pow(abs(_p1.y - _p2.y), 2));
}

float get_radian(const cv::Point2f& _center, const cv::Point2f& _p2){
    float dx = _p2.x - _center.x ;
    float dy = _p2.y - _center.y ;

    auto rad = (float)(atan2(dy, dx)*180.0f / PI);
    return rad > 0.0 ? rad : 360.0f + rad;
    //return rad > 0.0 ? rad : 360.0 - rad;
}

cv::Point get_point(const cv::Point2f& _center, float _rad, float _radius){
    if(_rad > 360.0){
        _rad -= 360.0;
    } else if(_rad < 0.0){
        _rad += 360.0;
    }
    _rad = PI * _rad / 180.0f;
    float x = _center.x + cos(_rad) * _radius;
    float y = _center.y + sin(_rad) * _radius;
    return cv::Point(x, y);
}

int getFanCenter(ContourPloy & _contour_ploy, HierarchyVec & _hie, cv::Point & output){
    for (int i = 0; i < _contour_ploy.size(); ++i) {
        if(_contour_ploy[i].size() == 4){
            if(_hie[i][2] == -1 && _hie[i][3] == -1){
                cv::RotatedRect rate = cv::minAreaRect(_contour_ploy[i]);
                output = rate.center;
                return 0;
            }
        }
    }
    return 1;
}

int getFanLeaf(ContourPloy & _contour_ploy, HierarchyVec & _hie, cv::Point & output){
    std::vector<cv::Point2f> fan_center;
    for (int i = 0; i < _contour_ploy.size(); ++i) {
        if(_contour_ploy[i].size() >= 5 && _contour_ploy[i].size() <= 10){
            if(_hie[i][2] == -1 && _hie[i][3] == -1){
                cv::RotatedRect rate = cv::minAreaRect(_contour_ploy[i]);
                fan_center.push_back(rate.center);
            }
        }
    }
    if(fan_center.empty()){
        return 1;
    }
    float min_dis = 999999;
    int target1, target2;
    for (int i = 0; i < fan_center.size(); ++i) {
        for (int j = 0; j < fan_center.size(); ++j) {
            if(i == j){
                continue;
            }
            float t_min = get_two_point_dis(fan_center[i], fan_center[j]);
            if (t_min < min_dis) {
                min_dis = t_min;
                target1 = i;
                target2 = j;
            }
        }
    }
    output = cv::Point((fan_center[target1].x + fan_center[target2].x) / 2,
                       (fan_center[target1].y + fan_center[target2].y) / 2);
    return 0;
}


void FanSearch::fresh_frame(cv::Mat & _frame) {
    this->frame = _frame;
}

int FanSearch::get_light() {
    if(this->frame.empty()){
        return 1;
    }
    typedef std::vector<cv::Mat> FrameColorChannel;
    FrameColorChannel channels;
    cv::split(frame, channels);
    cv::Mat r_channel, g_channel, b_channel;
    cv::threshold(channels[0], b_channel, 127, 255, cv::THRESH_BINARY);
    cv::threshold(channels[1], g_channel, 127, 255, cv::THRESH_BINARY_INV);
    cv::threshold(channels[2], r_channel, 127, 255, cv::THRESH_BINARY_INV);

    cv::bitwise_or(g_channel, r_channel, g_channel);
    cv::bitwise_and(b_channel, g_channel, b_channel);

    cv::Mat erode_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4));
    cv::Mat dilate_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    cv::erode(b_channel, this->thresh_mat, erode_element, cv::Point(-1, -1), 1);
    cv::dilate(this->thresh_mat, this->thresh_mat, dilate_element, cv::Point(-1, -1), 7);
    //cv::erode(b_channel, this->thresh_mat, element, cv::Point(-1, -1), 1);
    cv::GaussianBlur(this->thresh_mat, this->thresh_mat, cv::Size(5, 5), 5, 0);
    cv::threshold(this->thresh_mat, this->thresh_mat, 127, 255, cv::THRESH_BINARY);
    cv::dilate(this->thresh_mat, this->thresh_mat, dilate_element, cv::Point(-1, -1), 1);
    cv::imshow("thresh_mat", this->thresh_mat);
    return 0;
}

int FanSearch::get_fan_contours(cv::Point & output) {
    ContoursVec contours;
    HierarchyVec hierarchy;

    cv::findContours(this->thresh_mat, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
//    ContourPloy contour_ploy(contours.size());
//    std::vector<cv::Rect>

    //ContourPloy contout_ploy(contours.size());
    cv::Scalar red(0, 0, 255);
    cv::Scalar green(0, 255, 0);
    cv::Mat leaf_cont;
    frame.copyTo(leaf_cont);

    cv::RotatedRect rect;

    for (int i = 0; i < contours.size(); ++i) {
        if(cv::contourArea(contours[i]) < 100){
            continue;
        }
        rect = cv::minAreaRect(contours[i]);
        cv::Point2f rotated_rect_ps[4];
        rect.points(rotated_rect_ps);
        if(get_fan_center(rotated_rect_ps, hierarchy[i]) == 0){
            fan_center = rect.center;
        }
        else if(get_fan_leaf(rotated_rect_ps, hierarchy[i]) == 0){
            fan_leaves.emplace_back(rect.center);
        }
    }

    cv::Point2f leaf_point = cv::Point(-1, -1);
    identify_leaf(leaf_point);
    if(this->fan_center.x == -1){
        this->fan_leaves.clear();
        return 1;
    }
    if(leaf_point.x == -1){
        if(last_leaf.x != -1)
            leaf_point = last_leaf;
        else
            return 2;
    }
//    draw_point(fan_center, leaf_cont, yellow);
//    draw_point(leaf_point, leaf_cont, green);

    double leaf_angle = get_radian(fan_center, leaf_point);

    double predict_angle = kf_predict.predict(leaf_angle);
    if(predict_angle < 0.0){
        leaf_point = last_leaf;
        this->fan_leaves.clear();
        return 3;
    }
    auto predict_point = get_point(fan_center, leaf_angle + predict_angle, get_two_point_dis(fan_center, leaf_point));

#ifndef NODEBUG
    cv::putText(leaf_cont, "Current Angle : " + std::to_string(leaf_angle), cv::Point(0, 20), cv::FONT_HERSHEY_TRIPLEX,\
                    0.5, cyan, 2, 8);
    cv::circle(leaf_cont, fan_center, get_two_point_dis(fan_center, leaf_point), red, 2);
    cv::circle(leaf_cont, fan_center, 1, yellow, 10);
    cv::circle(leaf_cont, leaf_point, 1, green, 2);
    cv::circle(leaf_cont, predict_point, 15, green, 1);
    cv::imshow("leaf_cont", leaf_cont);
#endif

    last_leaf = leaf_point;
    this->fan_leaves.clear();
    return 0;
}

void FanSearch::draw_rotated_rect(cv::Point2f *_ps, cv::Mat &_frame, cv::Scalar & _color) {
    for (int i = 1; i <= 4; ++i) {
        cv::line(_frame, _ps[i % 4], _ps[i-1], _color, 2);
    }
}

int FanSearch::get_fan_center(cv::Point2f *_ps, Hierarchy &_hie) {
//    if(_hie[i][2] == -1 && _hie[i][3] == -1){
//            cv::RotatedRect rate = cv::minAreaRect(_contour_ploy[i]);
//            fan_center.push_back(rate.center);
//        }
    if(_hie[2] != -1 || _hie[3] != -1){
        return 1;
    }

    double pl1 = get_two_point_dis(_ps[0], _ps[1]);
    double pl2 = get_two_point_dis(_ps[1], _ps[2]);

    double rate = pl1 <= pl2 ? pl1 / pl2 : pl2 / pl1;

    if(rate > 0.8 && rate <= 1) {
        return 0;
    }
    return 2;
}

FanSearch::FanSearch() {
    this->green = cv::Scalar(0, 255, 0);
    this->yellow = cv::Scalar(0, 255, 255);
    this->red = cv::Scalar(0, 255, 255);
    this->cyan = cv::Scalar(247, 232, 12);
    this->last_leaf = FanLeaf(-1, -1);
    this->fan_center = FanCenter(-1, -1);
}

int FanSearch::get_fan_leaf(cv::Point2f *_ps, Hierarchy &_hie) {
    if(_hie[2] != -1 || _hie[3] != -1){
        return 1;
    }
    double pl1 = get_two_point_dis(_ps[0], _ps[1]);
    double pl2 = get_two_point_dis(_ps[1], _ps[2]);

    double rate = pl1 <= pl2 ? pl1 / pl2 : pl2 / pl1;

    if(rate > 0 && rate <= 0.4){
        return 0;
    }

    return 2;
}

int FanSearch::identify_leaf(cv::Point2f &output) {
    if(this->fan_leaves.size() < 2){
        return 1;
    }
    double less_dis = 99999.0;
    int target_x;
    int target_y;
    for (int i = 0; i < this->fan_leaves.size(); ++i) {
        for (int j = i; j < this->fan_leaves.size(); ++j) {
            if(j == i){
                continue;
            }
            double tmp_dis = get_two_point_dis(this->fan_leaves[i], this->fan_leaves[j]);
            if(tmp_dis < less_dis){
                target_x = i;
                target_y = j;
                less_dis = tmp_dis;
            }
        }
    }

    output = cv::Point((this->fan_leaves[target_x].x + this->fan_leaves[target_y].x) / 2,
                       (this->fan_leaves[target_x].y + this->fan_leaves[target_y].y) / 2);
    return 0;
}

void FanSearch::draw_point(cv::Point point, cv::Mat &_frame, Color _color) {
    cv::circle(frame, point, 2, _color, 10);
}
void FanSearch::reset() {
    this->kf_predict.reset();
}
