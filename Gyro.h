#ifndef GYRO_H
#define GYRO_H

#include <avr/io.h>

#define GYRO_ADDR 0xD2 //I2C addr, 0b11010010

#define WHO_AM_I_REG  0x0F
#define CTRL_REG1     0x20
#define CTRL_REG2     0x21 
#define CTRL_REG3     0x22
#define CTRL_REG4     0x23
#define CTRL_REG5     0x24
#define REFERENCE     0x25
#define OUT_TEMP      0x26
#define STATUS_REG    0x27
#define OUT_X_L       0x28
#define OUT_X_H       0x29
#define OUT_Y_L       0x2A
#define OUT_Y_H       0x2B
#define OUT_Z_L       0x2C
#define OUT_Z_H       0x2D
#define FIFO_CTRL_REG 0x2E
#define FIFO_SRC_REG  0x2F
#define INT1_CFG      0x30
#define INT1_SRC      0x31
#define INT1_TSH_XH   0x32
#define INT1_TSH_XL   0x33
#define INT1_TSH_YH   0x34
#define INT1_TSH_YL   0x35
#define INT1_TSH_ZH   0x36
#define INT1_TSH_ZL   0x37
#define INT1_DURATION 0x38

typedef struct {
    int16_t xRate;
    int16_t yRate;
    int16_t zRate;
} gyroData;

void Gyro_Setup(void);

void readGyroData(volatile gyroData* data);

#endif
