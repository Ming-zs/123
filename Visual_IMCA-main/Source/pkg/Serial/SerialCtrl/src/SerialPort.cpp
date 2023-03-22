#include <Serial/SerialCtrl/inc/SerialPort.h>

// 构造函数
SerialPort::SerialPort(const std::string portName)
    : _portName(portName), _fileDescription(-1), _portOpenFlag(false) {
    _rxBuffer = new uint8_t[64]();
    _txBuffer = new uint8_t[64]();
}

// 析构函数
SerialPort::~SerialPort() {
    // certain the port can be safty closed;
    if (_portOpenFlag) {
        close();
    }

    delete[] _txBuffer;
    delete[] _rxBuffer;
}

// 打开串口
bool SerialPort::open(speed_t baudRate) {
    // open serialport
    _fileDescription = ::open(_portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    if (_fileDescription == -1) {
        std::cout << "串口打开失败（被占用或者未连接）"
                  << "port name :" << _portName << std::endl;
        return false;
    }

    // 设置串口属性
    memset(&_option, 0, sizeof(_option));
    cfsetspeed(&_option, baudRate);
    _option.c_cflag |= CLOCAL | CREAD; // 设置控制模式
    _option.c_cflag |= CS8;            // 设置数据位
    _option.c_cflag &= ~PARENB;        // set no parity bit
    _option.c_cflag &= ~CSTOPB;        // set one stop bit
    // set raw mode
    _option.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // input
    _option.c_oflag &= ~OPOST;                          // output
    _option.c_cflag &= (~CRTSCTS);              // disable hardware flow control
    _option.c_iflag &= ~(IXON | IXOFF | IXANY); // disable software flow

    tcflush(_fileDescription, TCIOFLUSH); // flush buffer

    // set the new options for the serialport
    if (tcsetattr(_fileDescription, TCSANOW, &_option) != 0) {
        std::cout << "串口设置失败"
                  << "port name is: " << _portName << std::endl;
        return false;
    }
    _portOpenFlag = true;
    std::cout << "succeseful open" << std::endl;
    return true;
}

// 关闭串口
void SerialPort::close() {
    _portOpenFlag = false;
    ::close(_fileDescription);
}

union result {
    float value = 0.0f;
    //    int value = 0;
    uint8_t str[4];
};

// 解析
// todo : 把receive提取到的_rxBuffer数据整理到 SSI _selfStatInfo结构体上
void SerialPort::unpackSSIData(uint8_t *dataBuffer) {
    _selfStatInfo.SOF = dataBuffer[0];           // 帧头
    _selfStatInfo.robotModel = dataBuffer[1];    // 机器人模式
    this->_selfStatInfo.shutdownPC = dataBuffer[2]; // PC状态
    _selfStatInfo.robotColor = dataBuffer[3];    // 机器人颜色
    _selfStatInfo.CRC8 = dataBuffer[4];
    FLOAT32 pitch, yaw;
    FLOAT32_2 forward;
    uint8_t pitch_bytes[4];
    uint8_t yaw_bytes[4];
    uint8_t forward_bytes[2];
    memcpy(yaw_bytes, dataBuffer + 5, 4);  // 4
    memcpy(pitch_bytes, dataBuffer + 9, 4);  // 4
    memcpy(forward_bytes,dataBuffer + 13,2);  // 2
    _selfStatInfo.currentPitch = pitch.setBytes(pitch_bytes);
    _selfStatInfo.currentYaw = yaw.setBytes(yaw_bytes);
    _selfStatInfo.robotForward = forward.setBytes(forward_bytes);
}

// 串口数据接收
bool SerialPort::receive() {
    memset(_rxBuffer, 0, sizeof(_rxBuffer)); //将_rxBuffer（接收数据指针头）全部初始化为0
    int readCount = 0;
    int dataSize = dataLength;  // 数据长度
    while (readCount < dataSize) {
        int onceReadCount;
        // 返回读取_rxBuffer + readCount（缓冲区地址）的字节数，失败-1，文件尾0
        onceReadCount = read(_fileDescription, _rxBuffer + readCount, dataSize - readCount);
//        printf("onceReadCount------>%d\t",onceReadCount);
        // Log_Info << onceReadCount;
        if (onceReadCount == -1) {
            if (errno ==
                EAGAIN) //在write为非阻塞模式中，表示现在无数据可读，这时候就会置全局变量errno为EAGINA，表示可以再次进行读操作
            {
                continue;
            }
            return false;
        }
        readCount += onceReadCount;
    }

//     遍历输出获取到的数据
    for (int i = 0; i < dataLength; ++i) {
        printf("%2X ", _rxBuffer[i]);
    }
    printf("\n");

    tcflush(_fileDescription, TCIFLUSH);  // 清空缓存
    // crc8校验
    if (_rxBuffer[0] == 0xA5 && verifyCrc8CheckSum(_rxBuffer, 5)) {
        // Log_Info << "crc8 ok";
        // crc16校验
        if (verifyCrc16CheckSum(_rxBuffer, dataLength)) {
            if (_rxBuffer[1] == 0) {
                unpackSSIData(_rxBuffer);
            } else if (_rxBuffer[1] == 2) {
                //unpackEHPIData(_rxBuffer);
            }
        }
        return true;
    }
    return false;
}

SSI SerialPort::getSSI() { return _selfStatInfo; }

// 向单片机发送数据
int SerialPort::send(const ControlFrame &controlFrame) {
    packData(controlFrame);
    int sendByteSize = ::write(_fileDescription, _txBuffer, 14);
    memset(_txBuffer, 0, sizeof(_txBuffer));
    return sendByteSize;
}

void SerialPort::packData(const ControlFrame &controlFrame) {
    // frame header
    _txBuffer[0] = 0xA5;
    _txBuffer[1] = controlFrame.seq;
    _txBuffer[2] = controlFrame.model;
    appendCrc8CheckSum(_txBuffer, 4);
    _txBuffer[4] = controlFrame.yaw.bytes[0];
    _txBuffer[5] = controlFrame.yaw.bytes[1];
    _txBuffer[6] = controlFrame.yaw.bytes[2];
    _txBuffer[7] = controlFrame.yaw.bytes[3];
    _txBuffer[8] = controlFrame.pitch.bytes[0];
    _txBuffer[9] = controlFrame.pitch.bytes[1];
    _txBuffer[10] = controlFrame.pitch.bytes[2];
    _txBuffer[11] = controlFrame.pitch.bytes[3];
    // frame tail
    appendCrc16CheckSum(_txBuffer, 14);
}

// 获取CRC8校验值
uint8_t SerialPort::getCrc8CheckSum(uint8_t *dataBuffer, size_t dataLength) {
    uint8_t crc8 = 0xff;
    uint8_t Index;
    while (dataLength--) {
        Index = crc8 ^ (*dataBuffer++);
        crc8 = _CRC8_TAB[Index];
    }
    return crc8;
}

// CRC8校验
int SerialPort::verifyCrc8CheckSum(uint8_t *dataBuffer, size_t dataLength) {
    uint8_t expected = 0;
    if ((dataBuffer == nullptr) || (dataLength <= 2)) {
        return 0;
    }
    expected = getCrc8CheckSum(dataBuffer, dataLength - 1);
    return (expected == dataBuffer[dataLength - 1]);
}

void SerialPort::appendCrc8CheckSum(uint8_t *dataBuffer, size_t dataLength) {
    uint8_t crc8 = 0;
    if ((dataBuffer == nullptr) || (dataLength <= 2)) {
        return;
    }
    crc8 = getCrc8CheckSum(dataBuffer, dataLength - 1);
    dataBuffer[dataLength - 1] = crc8;
}

// 获取CRC16的校验值
uint16_t SerialPort::getCrc16CheckSum(uint8_t *dataBuffer, size_t dataLength) {
    uint16_t crc16 = 0xffff;  // 预置16位CRC寄存器
    uint8_t chData;   // CRC循坏索引
    if (dataBuffer == nullptr) {
        return 0xffff;
    }
    // 传输消息缓冲区
    while (dataLength--) {
        chData = *dataBuffer++;  // 前15位数据
        // 查表法CRC16校验（只校验低八位）
        crc16 = (crc16 >> 8) ^
                _CRC16_TAB[(crc16 ^ static_cast<uint16_t>(chData)) & 0x00ff];
    }
    return crc16;
}
// CRC16校验
uint32_t SerialPort::verifyCrc16CheckSum(uint8_t *dataBuffer,
                                         size_t dataLength) {
    uint16_t expected = 0;
    if ((dataBuffer == NULL) || (dataLength <= 2)) {
       return false;
    }
    // 获取Crc16的检查值（倒数第一，二位数据）
    expected = getCrc16CheckSum(dataBuffer, dataLength - 2);
//    printf("CRC16------>%d\n",expected);
    // 高八位和低八位校验
//    return ((expected & 0xff) == dataBuffer[dataLength - 2] &&
//            ((expected >> 8) & 0xff) == dataBuffer[dataLength - 1]);
    // 低八位校验结果
    return ((expected >> 8) & 0xff) == dataBuffer[dataLength - 1];
}

void SerialPort::appendCrc16CheckSum(uint8_t *dataBuffer, size_t dataLength) {
    uint16_t crc16 = 0;
    if ((dataBuffer == nullptr) || (dataLength <= 2)) {
        return;
    }
    crc16 = getCrc16CheckSum(dataBuffer, dataLength - 2);
    dataBuffer[dataLength - 2] = static_cast<uint8_t>(crc16 & 0x00ff);
    dataBuffer[dataLength - 1] = static_cast<uint8_t>((crc16 >> 8) & 0x00ff);
}

// CRC表
const uint8_t SerialPort::_CRC8_TAB[256]{
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20,
    0xa3, 0xfd, 0x1f, 0x41, 0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23, 0x7d, 0x9f, 0xc1,
    0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e,
    0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67, 0x39,
    0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45,
    0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 0x8c, 0xd2, 0x30, 0x6e,
    0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31,
    0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
    0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e, 0xd0,
    0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea,
    0x69, 0x37, 0xd5, 0x8b, 0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b,
    0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54,
    0xd7, 0x89, 0x6b, 0x35,
};

const uint16_t SerialPort::_CRC16_TAB[256]{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48,
    0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108,
    0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb,
    0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
    0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e,
    0xfae7, 0xc87c, 0xd9f5, 0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e,
    0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd,
    0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285,
    0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44,
    0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 0x6306, 0x728f, 0x4014,
    0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
    0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3,
    0x242a, 0x16b1, 0x0738, 0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862,
    0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e,
    0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1,
    0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483,
    0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 0x2942, 0x38cb, 0x0a50,
    0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
    0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7,
    0x6e6e, 0x5cf5, 0x4d7c, 0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1,
    0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72,
    0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e,
    0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf,
    0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9, 0xf78f, 0xe606, 0xd49d,
    0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
    0x3de3, 0x2c6a, 0x1ef1, 0x0f78,
};
