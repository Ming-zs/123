//
// Created by ica on 2023/2/19.
//
#include "Camera/include/config/CameraVar.h"
#include "CameraWapper.h"
#include "GlobalVar/inc/Logger.h"
#include "config/CameraConfig.h"

#include <iostream>
#include <opencv2/core/core_c.h>
#include <opencv2/core/types_c.h>

using namespace std;
using namespace cv;

CameraWrapper::CameraWrapper(const std::string &n, int explosure)
    : name(n), init_done(false), mode(2), camera_cnts(2), camera_status(-1),
      iplImage(nullptr), rgb_buffer(nullptr), channel(3),
      gain(ARMOR_CAMERA_GAIN), exposure(explosure) {

    LOGM("相机名字: %s", g_cam_name.c_str());
    LOGM("相机参数文件: %s", g_cam_config_path);
    LOGM("相机曝光度: %d", g_cam_exposure);
    LOGM("相机使能: %s", g_cam_enable ? "Yes" : "No");
}
// 回调函数
void cameraCallback(CameraHandle hCamera, BYTE *pFrameBuffer,
                    tSdkFrameHead *pFrameHead, PVOID pContext) {
    CameraWrapper *c = (CameraWrapper *)pContext;
    CameraImageProcess(hCamera, pFrameBuffer, c->rgb_buffer, pFrameHead);
    auto iplImage =
        cvCreateImageHeader(cvSize(pFrameHead->iWidth, pFrameHead->iHeight),
                            IPL_DEPTH_8U, c->channel);
    cvSetData(
        iplImage, c->rgb_buffer,
        pFrameHead->iWidth *
            c->channel); //此处只是设置指针，无图像块数据拷贝，不需担心转换效率
    c->src_queue.push(cv::cvarrToMat(iplImage).clone());
}
// 初始化函数
bool CameraWrapper::init() {
    CameraSdkInit(1);
    int camera_enumerate_device_status =
        CameraEnumerateDevice(camera_enum_list, &camera_cnts);
    if (camera_enumerate_device_status != CAMERA_STATUS_SUCCESS) {
        LOGE("CameraEnumerateDevice fail with %d!",
             camera_enumerate_device_status);
    }
    if (camera_cnts == 0) {
        LOGE("没找到相机！");
        return false;
    } else if (camera_cnts >= 1) {
        LOGM("%d 相机未链接!", camera_cnts);
    }
    int i;
    for (i = 0; i < camera_cnts; i++) {
        camera_status = CameraInit(&camera_enum_list[i], -1, -1, &h_camera);
        if (camera_status != CAMERA_STATUS_SUCCESS) {
            CameraUnInit(h_camera);
            continue;
        }
        CameraGetFriendlyName(h_camera, camera_name);
        if (name == "NULL" || strcmp(name.data(), camera_name) == 0) {
            break;
        }
        CameraUnInit(h_camera);
    }
    if (i >= camera_cnts) {
        LOGE("相机：%s未连接", name.data());
        return false;
    }

    auto status = CameraGetCapability(h_camera, &tCapability);
    if (status != CAMERA_STATUS_SUCCESS) {
        cout << "CameraGetCapability return error code " << status << endl;
        return false;
    }
    rgb_buffer =
        (unsigned char *)malloc(tCapability.sResolutionRange.iHeightMax *
                                tCapability.sResolutionRange.iWidthMax * 3);

    LOGM("加载参数文件.........");
    CameraReadParameterFromFile(
        h_camera, g_cam_config_path); // PROJECT_DIR"/others/bbbb.config"

    LOGM("Read Param : %d ",
         CameraReadParameterFromFile(h_camera, g_cam_config_path));

    LOGM("加载参数成功");
    // CameraLoadParameter(h_camera,0);//before Load need to save as TeamA;//gp
    // cout<<"STATUS:  "<<CameraLoadParameter(h_camera, 0xff)<<endl;
    /*tSdkImageResolution imageResolution = { 0 };
    //set to 0xff for custom resolution
    imageResolution.iIndex = 0xff;
    imageResolution.uBinSumMode = 0;
    imageResolution.uBinAverageMode = 0;
    imageResolution.uSkipMode = 0;
    imageResolution.uResampleMask = 0;
    imageResolution.iHOffsetFOV = 0;
    imageResolution.iVOffsetFOV = 0;
    imageResolution.iWidth = 1024;
    imageResolution.iWidthFOV = 1024;
    imageResolution.iHeight = 600;
    imageResolution.iHeightFOV = 600;
    imageResolution.iWidthZoomHd = 0;
    imageResolution.iHeightZoomHd=0;
    imageResolution.iWidthZoomSw=0;
    imageResolution.iHeightZoomSw=0;
    CameraSetImageResolution(h_camera, &imageResolution);*/
    CameraSetAeState(h_camera, 0); // 曝光模式
    CameraSetAntiFlick(h_camera, 1);
    // 曝光时间
    CameraSetExposureTime(h_camera, exposure * 1000); // original;exposure *
                                                      // 1000
    CameraSetFrameSpeed(h_camera, 2);
    CameraSetAnalogGain(h_camera, 3); // gain

    double t;
    int g;

    // 分辨率结构体
    tSdkImageResolution *pImageResolution;

    // 获取抓取的分辨率
    CameraGetImageResolution(h_camera, pImageResolution); // 1024x1280
    // 修改分辨率
//    pImageResolution->iWidth = 512;
//    pImageResolution->iHeight = 640;
    // 设置分辨率
    CameraSetImageResolution(h_camera, pImageResolution);

    CameraGetExposureTime(h_camera, &t);
    CameraGetAnalogGain(h_camera, &g);
    LOGM("相机曝光时间: %lf ms, gain:%d", t / 1000.0, g);
    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */
    CameraPlay(h_camera);

    //设置/读取分辨率

    /*其他的相机参数设置
    例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
         CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
         CameraSetGamma、CameraSetContrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
         CameraGetFriendlyName    CameraSetFriendlyName
    获取/设置相机名称（该名称可写入相机硬件）
    */
    // cout << tCapability.sIspCapacity.bMonoSensor << endl;
    if (tCapability.sIspCapacity.bMonoSensor) {
        channel = 1;
        CameraSetIspOutFormat(h_camera, CAMERA_MEDIA_TYPE_MONO8);
        LOGM("camera %s mono ", camera_name);
    } else {
        channel = 3;
        CameraSetIspOutFormat(h_camera, CAMERA_MEDIA_TYPE_BGR8);
        LOGM("camera %s color ", camera_name);
    }
    if (mode == 2) {
        CameraSetCallbackFunction(h_camera, cameraCallback, this, nullptr);
    }
    init_done = true;
    return true;
}
// 读取图像
bool CameraWrapper::read(cv::Mat &src) {
    if (init_done) {
        //读取帧的三个模式
        if (mode == 0)
            return readProcessed(src);
        if (mode == 1)
            return readRaw(src);
        if (mode == 2)
            return readCallback(src);
    } else {
        return false;
    }
    return true;
}

//-->
// 读取原始数据
bool CameraWrapper::readRaw(cv::Mat &src) {
    // 获得一帧图像数据
    if (CameraGetImageBuffer(h_camera, &frame_info, &pby_buffer, 500) ==
        CAMERA_STATUS_SUCCESS) {
        // 判空
        if (iplImage) {
            cvReleaseImageHeader(&iplImage);
        }
        // 创建IplImage图像头
        iplImage = cvCreateImageHeader(
            cvSize(frame_info.iWidth, frame_info.iHeight), IPL_DEPTH_8U, 1);
        // 设置指定图像头的数据
        cvSetData(
            iplImage, pby_buffer,
            frame_info
                .iWidth); //此处只是设置指针，无图像块数据拷贝，不需担心转换效率
        //        printf("H----->%d,W------>%d",frame_info.iHeight,frame_info.iWidth);
        // 转化为mat实例
        src = cv::cvarrToMat(iplImage).clone();

        //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
        //否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
        CameraReleaseImageBuffer(h_camera, pby_buffer);

        return true;
    } else {
        src = cv::Mat();
        return false;
    }
}
// 读取处理
bool CameraWrapper::readProcessed(cv::Mat &src) {
    //	cerr << "Get-1" << endl;
    if (CameraGetImageBuffer(h_camera, &frame_info, &pby_buffer, 500) ==
        CAMERA_STATUS_SUCCESS) {
        CameraImageProcess(
            h_camera, pby_buffer, rgb_buffer,
            &frame_info); // this function is super slow, better not to use it.
        if (iplImage) {
            cvReleaseImageHeader(&iplImage);
        }
        iplImage =
            cvCreateImageHeader(cvSize(frame_info.iWidth, frame_info.iHeight),
                                IPL_DEPTH_8U, channel);
        cvSetData(
            iplImage, rgb_buffer,
            frame_info.iWidth *
                channel); //此处只是设置指针，无图像块数据拷贝，不需担心转换效率
        src = cv::cvarrToMat(iplImage).clone();
        //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
        //否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
        CameraReleaseImageBuffer(h_camera, pby_buffer);
        return true;
    } else {
        src = cv::Mat();
        return false;
    }
}
// 读取回调函数
bool CameraWrapper::readCallback(cv::Mat &src) {
    systime ts, te;
    getsystime(ts);
    while (src_queue.empty()) {
        getsystime(te);
        if (getTimeIntervalms(te, ts) > 500) {
            return false;
        }
    }
    return src_queue.pop(src);
}
//<--

// 析构函数
CameraWrapper::~CameraWrapper() {
    CameraUnInit(h_camera);
    //注意，先反初始化后再free
    if (rgb_buffer != nullptr)
        free(rgb_buffer);
}

extern WrapperHead *video;
// 检查连接
bool CameraWrapper::checkReconnect(bool is_camera_connect) {
    if (!is_camera_connect) {
        int curr_gain = ((CameraWrapper *)video)->gain;
        int curr_exposure = ((CameraWrapper *)video)->exposure;
        delete video;
        video = new CameraWrapper(g_cam_name, g_cam_exposure);
        is_camera_connect = video->init();
    }
    return is_camera_connect;
}
