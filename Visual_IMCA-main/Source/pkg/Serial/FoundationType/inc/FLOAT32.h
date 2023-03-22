//
// Created by experi on 2023/3/4.
//

#ifndef FLOAT32_H
#define FLOAT32_H

#include <cstdint>

// 四位
class FLOAT32{

  public:

    float value = 0;
    uint8_t bytes[4]{0};

    explicit FLOAT32(const float & value);

    explicit FLOAT32(const uint8_t bytes[4]);

    explicit FLOAT32();

    FLOAT32 setFloat(float _value);

    FLOAT32 setBytes(uint8_t _bytes[4]);

    float getFloat();

    uint8_t *getBytes();

};

// 两位
class FLOAT32_2{
  public:
    float value = 0;
    uint8_t bytes[2]{0};

    FLOAT32_2 setBytes(uint8_t _bytes[2]);
};

#endif // ROOT_IMCA_FLOAT32_H
