//
// Created by ica on 2023/2/19.
//

#ifndef CAMERAWAPPER_H__
#define CAMERAWAPPER_H__

#include "GlobalVar/inc/RoundQueue.h"
#include "MVSDK/CameraApi.h"
#include "WapperHeader.h"
#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>

class CameraWrapper: public WrapperHead {
    friend void cameraCallback(CameraHandle hCamera, BYTE *pFrameBuffer, tSdkFrameHead* pFrameHead,PVOID pContext);
private:
    const std::string name;
    int mode;
    bool readParam = 1;//dont read camera param twice;
    bool init_done;

    unsigned char* rgb_buffer;
    int camera_cnts;
    int camera_status;
    tSdkCameraDevInfo camera_enum_list[2];
    int h_camera;
    /*int _imageWidth = 640;
    int _imageHeight = 480;*/
    char camera_name[32];

    tSdkCameraCapbility tCapability;
    tSdkFrameHead frame_info;
    BYTE *pby_buffer;
    IplImage* iplImage;
    int channel;

    RoundQueue<cv::Mat, 2> src_queue;
  public:
    int gain;
    int exposure;
    CameraWrapper(const std::string &n, int explosure);
    ~CameraWrapper() final;

    bool init() final;
    bool read(cv::Mat& src) final;
    bool readRaw(cv::Mat& src);
    bool readProcessed(cv::Mat& src);
    bool readCallback(cv::Mat& src);

    static bool checkReconnect(bool is_camera_connect);
};

#endif // CAMERAWAPPER_H__
