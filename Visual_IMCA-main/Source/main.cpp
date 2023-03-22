// create by experi	 2023-03-04 10:26:07

/*
 _____  __  __   _____
|_   _||  \/  | / ____|    /\
  | |  | \  / || |        /  \
  | |  | |\/| || |       / /\ \
 _| |_ | |  | || |____  / ____ \
|_____||_|  |_| \_____|/_/    \_\

    Frame By Thrent
*/

#include "Serial/FoundationType/inc/FLOAT32.h"
#include "Serial/FoundationType/inc/SSI.h"
#include "Serial/SerialCtrl/inc/SerialPort.h"

#include "GlobalVar/inc/Logger.h"

#include "Camera/include/VideoWapper.h"
#include "Camera/include/CameraWapper.h"

#include "Visual/VisualCtrl/inc/VisualCtrl.h"

#include "iostream"
#include <mutex>
#include <fstream>
#include <memory>
#include <thread>

#include "nlohmann/json.hpp"

using Json = nlohmann::json;
using namespace std;

SSI         g_ssi;   // 机器人初始信息结构体
std::mutex  mt_g_ssi;

int         g_cam_exposure;
std::string g_cam_name;
char*       g_cam_config_path;
bool        g_cam_enable;
ControlFrame control_frame;


SerialPort serial("/dev/ttyUSB0");

WrapperHead* video = nullptr;

//线程、串口操作
void uartReceive(SerialPort *pSerial){
    while(true)
    {
        pSerial->receive();
        mt_g_ssi.lock();
        g_ssi=pSerial->getSSI();
        mt_g_ssi.unlock();
    }
}

int main(int argc, char ** argv){
    /***************************初始化系统(JSON)*****************************************8*/
    //将初始化所需的参数保存到json文件中
    Json config_file;
    std::ifstream j_config("../Source/config.json");
    j_config >> config_file;

    g_cam_name = config_file.at("Camera").at("CameraName"); //相机名称
    std::string camera_param_path = config_file.at("Camera").at("ConfigFile");  //相机配置文件路径
    g_cam_config_path = camera_param_path.data();
    g_cam_exposure = config_file.at("Camera").at("Exposure"); // 相机曝光时间（毫秒）
    g_cam_enable = config_file.at("Camera").at("Enable");  //相机状态

    int enemy_color = config_file.at("DefaultVisualSetting").at("EnemyColor"); // 敌方装甲颜色
    int self_type = config_file.at("DefaultVisualSetting").at("SelfType");  // 敌方兵种类型
    int mode = config_file.at("DefaultVisualSetting").at("Mode");  //切换功能

    g_ssi.robotColor = !enemy_color;
    g_ssi.ssType = self_type;
    g_ssi.robotModel = mode;

    std::string video_path = config_file.at("Video").at("VideoPath");  // 视频输入路径
    std::string video_save_path = config_file.at("Video").at("VideoSavePath");  // 视频保存路径

    bool capture = config_file.at("Video").at("Capture");

    LOGM("敌方颜色：%s", enemy_color ? "蓝色" : "红色");

    /***************************初始化完成*****************************************8*/

    serial.open();  // 串口开启
    thread recevie(uartReceive, &serial);  //线程

    //动态分配内存给智能指针托管
    std::shared_ptr<VisualCtrl> visual_ctrl = std::make_shared<VisualCtrl>(video_save_path);

    while(true){
        //摄像头or视频
        if(g_cam_enable){
            video = new CameraWrapper(g_cam_name, g_cam_exposure);
        } else{
            video = new VideoWrapper(video_path);
        }
        // 图像源初始化
        if(video->init()){
            LOGM("视频源初始化成功");
        } else{
            LOGW("当前视频源不可用");
        }
        //去掉前十帧噪声图像
        cv::Mat src;
        for (int i = 0; i < 10; i++) {
            if (video) {
                //
                video->read(src);
            }
        }

        bool ok = true;
        LOGM("启动...");
        do{
            // 关机
            if(g_ssi.shutdownPC){
                // TODO: 关机
                break;
            }
            // 检查/重连
            ok = CameraWrapper::checkReconnect(video->read(src));
            // 机器人模式切换
            visual_ctrl->switchOption(g_ssi.robotModel);
            // 获取图像信息，整合成角帧
            auto angleFrame = visual_ctrl->run(src);
            // 控制帧结构体赋值
            control_frame.model = true;
            control_frame.seq++;
            control_frame.yaw = angleFrame.yaw;
            control_frame.pitch = angleFrame.pitch;
            // 串口发送控制帧
            serial.send(control_frame);

            //如果获取到图像帧
            if(capture){
                // 视频保存
                visual_ctrl->saveVideos(src);
            }
            cv::namedWindow("MainCamera",0);  // 可调节窗口
            cv::resizeWindow("MainCamera",512,640);  // 窗口大小
            cv::imshow("MainCamera", src);
            cv::waitKey(30);
        } while (ok);
        delete video;
        video = nullptr;
        LOGW("崩溃，恢复中...");
        if(g_ssi.shutdownPC){
            LOGM("关机..");
            break;
        }
    }

    return 0;
}
