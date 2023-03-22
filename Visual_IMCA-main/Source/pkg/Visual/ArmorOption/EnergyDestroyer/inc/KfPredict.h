//
// Created by imca on 2023/3/4.
//

#ifndef KFPREDICT_H
#define KFPREDICT_H

#include <opencv2/opencv.hpp>

#define TIME 0.4

struct TargetWithTime{
    float target;
    clock_t time;
};


class KfPredict {
    using LeafVec = std::vector<TargetWithTime>;
    using SeparateVec = LeafVec;
  private:

    int forward;

    LeafVec leaf_vec;
    SeparateVec separate_vec;

    cv::KalmanFilter * kf;

    int separate_leaf();

    double get_avg_speed();

    int add_leaf(float _target);

    cv::Mat measurement;   //接收测量值

  public:
    float predict(float _rad);

    void reset();

    KfPredict();
};


#endif // ROOT_IMCA_KFPREDICT_H
