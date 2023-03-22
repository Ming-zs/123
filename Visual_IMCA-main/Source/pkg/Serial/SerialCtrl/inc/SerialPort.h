
#ifndef SERIAL__H__
#define SERIAL__H__


#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>

#include <memory.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>

#include "Serial/FoundationType/inc/SSI.h"
#include "GlobalVar/inc/ControlFrame.h"



class SerialPort
{
  public:
    const int dataLength = 17;  // 数据长度




  public:
    /**
     * @brief SerialPort类的结构体
     *
     * @param portName 串口名，存在于/dev/，一般被命名为RMUSB
     */
    explicit SerialPort(const std::string portName);

    /**
     * @brief 析构函数
     *
     */
    ~SerialPort();

  public:
    /**
     * @brief 打开串口
     *
     * @param baudRate 波特率，默认为115200
     * @return true
     * @return false
     */
    bool open(speed_t baudRate = B115200);

    /**
     * @brief 关闭串口
     *
     */
    void close();

    /*--------------------------------------------------------------------
    \@brief         send data
    \@param         the control frame which need to be send
    \@return        the number of byte that send successfully
    *------------------------------------------------------------------*/

    /**
     * @brief 发送数据
     *
     * @param controlFrame 需要发送数据的结构体
     * @return int 当返回发送数据的长度的时候就成功
     */
    int send(const ControlFrame& controlFrame);

    /**
     * @brief 接收串口数据
     *
     * @return true
     * @return false
     */
    bool receive();

    /**
     * @brief 获取自身属性
     *
     * @return SSI
     */
    SSI getSSI();


    //FeedBackFrame getFeedBack();


  private:
    /**
     * @brief 解析自身属性
     *
     * @param dataBuffer
     */
    void unpackSSIData(uint8_t* dataBuffer); // TODO：需要将dataBuffer用结构体替换，不然很操蛋

    /**
     * @brief 解析敌方数据
     *
     * @param dataBuffer
     */
    void unpackEHPIData(uint8_t* dataBuffer);

    /**
     * @brief 打包将要发送的数据
     *
     * @param controlFrame
     */
    void packData(const ControlFrame& controlFrame);

    /**
     * @brief 获取CRC8检查表
     *
     * @param dataBuffer 数据缓存
     * @param dataLength 数据长度
     * @return uint8_t
     */
    uint8_t getCrc8CheckSum(uint8_t *dataBuffer, size_t dataLength);

    /**
     * @brief CRC8校验
     *
     * @param dataBuffer 数据缓存
     * @param dataLength 数据长度：data+crc8 (byte)
     * @return int
     */
    int verifyCrc8CheckSum(uint8_t *dataBuffer, size_t dataLength);

    /**
     * @brief 向数据缓存添加CRC8校验码
     *
     * @param dataBuffer 数据缓存
     * @param dataLength 数据长度
     */
    void appendCrc8CheckSum(uint8_t* dataBuffer, size_t dataLength);

    /**
     * @brief 获取CRC16检查表
     *
     * @param dataBuffer 数据缓存
     * @param dataLength 数据长度
     * @return uint16_t
     */
    uint16_t getCrc16CheckSum(uint8_t *dataBuffer,size_t dataLength);

    /**
     * @brief CRC16验证
     *
     * @param dataBuffer 数据缓存
     * @param dataLength 数据长度 data + crc16 byte
     * @return uint32_t
     */
    uint32_t verifyCrc16CheckSum(uint8_t *dataBuffer, size_t dataLength);

    /**
     * @brief 向数据缓存添加CRC16校验码
     *
     * @param dataBuffer 数据缓存
     * @param dataLength 数据长度
     */
    void appendCrc16CheckSum(uint8_t *dataBuffer, size_t dataLength);

  private:
    std::string                         _portName;              // 串口名
    termios                             _option;                // 串口设置
    uint8_t                             *_rxBuffer;             // 接受数据指针头
    uint8_t                             *_txBuffer;             // 发送数据指针头
    int                                 _fileDescription;       // 当串口打开后，串口描述符将会保存在此，详情请查看Linux原生文件读写
    bool                                _portOpenFlag;          // 串口是否打开
    static const uint8_t                _CRC8_TAB[256];
    static const uint16_t               _CRC16_TAB[256];
    SSI                                 _selfStatInfo;
    //FeedBackFrame                       _feedBackFrame;
};


#endif