//
// Created by imca on 2023/3/4.
//

#include "Visual/ArmorOption/EnergyDestroyer/inc/KfPredict.h"

int KfPredict::add_leaf(float _target) {
    int is_full = 0;
    TargetWithTime target_entity{};
    target_entity.target = _target;
    target_entity.time = clock();
    if(!this->leaf_vec.empty() && (double)(target_entity.time - leaf_vec.begin()->time) > (TIME * 1500000)){
        leaf_vec.erase(leaf_vec.begin());
        is_full = 1;
    }
    leaf_vec.emplace_back(target_entity);

    return is_full;
}

double KfPredict::get_avg_speed() {
    double avg_speed = 0.0;
    for(auto & sep : separate_vec){
        avg_speed += sep.target;
    }
    return avg_speed / separate_vec.size();
}

int KfPredict::separate_leaf() {
    int space = 5, count = 0;
    auto begin_leaf = leaf_vec[0];
    for (int i = leaf_vec.size() / 2; i < leaf_vec.size(); ++i) {
        if(space--)
            continue;
        space = 5;
        count++;
        float space_rad = leaf_vec[i].target - begin_leaf.target;
        forward = space_rad < 0.0f ? 1 : -1;
        forward = space_rad > 270.0f ? -1 : 1;
        space_rad = space_rad < 0.0f ? 360.0f + space_rad : space_rad;
        space_rad = space_rad > 270.0f ? 360.0f - space_rad : space_rad;
        if(!separate_vec.empty()){
            float rad_rate = space_rad - separate_vec.rbegin()->target > 0 ? separate_vec.rbegin()->target / space_rad :
                             space_rad / separate_vec.rbegin()->target;
            if(rad_rate < 0.78 || abs(space_rad - separate_vec.rbegin()->target) > 3){
                space_rad = (space_rad + separate_vec.rbegin()->target) / 2;
            }
        }
        if(separate_vec.size() > 5)
            separate_vec.erase(separate_vec.begin());
        separate_vec.emplace_back(TargetWithTime{space_rad,leaf_vec[i].time - begin_leaf.time});
        begin_leaf = leaf_vec[i];
    }
    return 0;
}

float KfPredict::predict(float _rad) {
    if(!add_leaf(_rad)){
        return -1.0;
    }
    separate_leaf();
//    for (int i = 0; i < separate_vec.size(); ++i) {
//        printf("separate : %.2f, \ttime: %ld\n", separate_vec[i].target, separate_vec[i].time);
//    }
    //printf("separate_vec : %d\n", separate_vec.size());
    if(separate_vec.empty()){
        return -2.0;
    }
    kf->statePost = (cv::Mat_<float>(1, 1) << 1)*separate_vec.rbegin()->target;   //x(0)初始化 接口
    auto kf_predict = kf->predict();
    // std::cout << "Predict : " << kf_predict << std::endl;
    return kf_predict.at<float>(0) * 0.8f * forward;
}

KfPredict::KfPredict() {
    int stateNum = 1;
    int measureNum = 1;
    kf = new cv::KalmanFilter(stateNum, measureNum, 0);
    kf->transitionMatrix = cv::Mat_<float>(1, 1) << 1;
    setIdentity(kf->measurementMatrix);                         //测量矩阵H
    setIdentity(kf->processNoiseCov, cv::Scalar::all(1e-3));    //系统噪声方差矩阵Q——参数调整
    setIdentity(kf->measurementNoiseCov, cv::Scalar::all(1e-2));//测量噪声方差矩阵R——参数调整
    setIdentity(kf->errorCovPost, cv::Scalar::all(1));          //后验错误估计协方差矩阵P
    measurement = cv::Mat::zeros(measureNum, 1, CV_32F);        //初始测量值x'(0)
}
void KfPredict::reset() {
    this->leaf_vec.clear();
}
