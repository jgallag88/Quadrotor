#ifndef COMPASS_H
#define COMPASS_H

#include <avr/io.h>

#define COMPASS_ADDR 0x3C  // I2C Write Address (+1 for read)

// Compass registers
#define CONFIG_A    0x00
#define CONFIG_B    0x01
#define MODE_RW     0x02
#define DATA_X_MSB  0x03
#define DATA_X_LSB  0x04
#define DATA_Z_MSB  0x05
#define DATA_Z_LSB  0x06
#define DATA_Y_MSB  0x07
#define DATA_Y_LSB  0x08
#define STATUS      0x09
#define ID_A        0x0A
#define ID_B        0x0B
#define ID_C        0x0C

typedef struct {
    uint16_t x;
    uint16_t z;
    uint16_t y;
} compassData;

void Compass_Setup(void);

void readCompassData(compassData*);

#endif //COMPASS_H
