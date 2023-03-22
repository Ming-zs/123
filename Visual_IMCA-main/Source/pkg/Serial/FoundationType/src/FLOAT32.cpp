//
// Created by experi on 2023/3/4.
//

#include "Serial/FoundationType/inc/FLOAT32.h"
#include <cstring>

//todo��������ֽ�ת���Ż�Ϊ"���������"

FLOAT32::FLOAT32(const float &value) {
    this->value = value;
    union convert2byte{
        float d;
        uint8_t byte[4];
    } convert{};

    convert.d = value;

    memcpy(this->bytes, convert.byte, 4);
}

FLOAT32::FLOAT32(const uint8_t *bytes) {
    memcpy(this->bytes, bytes, 4);

    union convert2float{
        float d;
        uint8_t byte[4];
    } convert{};

    memcpy(convert.byte, this->bytes, 4);

    this->value = convert.d;
}

FLOAT32::FLOAT32() = default;

float FLOAT32::getFloat() {
    return this->value;
}

uint8_t *FLOAT32::getBytes() {
    return this->bytes;
}

FLOAT32 FLOAT32::setFloat(float _value) {
    this->value = _value;
    union convert2byte{
        float d;
        uint8_t byte[4];
    } convert{};

    convert.d = value;

    memcpy(this->bytes, convert.byte, 4);
    return *this;
}

// 四位
FLOAT32 FLOAT32::setBytes(uint8_t *_bytes) {
    memcpy(this->bytes, _bytes, 4);

    union convert2float{
        float d;
        uint8_t byte[4];
    } convert{};

    memcpy(convert.byte, this->bytes, 4);

    this->value = convert.d;
    return *this;
}

// 两位
FLOAT32_2 FLOAT32_2::setBytes(uint8_t *_bytes) {
    memcpy(this->bytes, _bytes,2);  // 取值
    //转换
    union convert2float{
        float d;
        uint8_t byte[2];
    }convert{};

    memcpy(convert.byte, this->bytes, 2);  // 赋值

    this->value = convert.d;
    return *this;
}
