// Description

#include <avr/io.h>
#include "Compass.h"
#include "I2C.h"

void Compass_Setup(void) {
    
}

void readCompassData(compassData* compassData) {
    I2C_ReadReg(COMPASS_ADDR, ID_A,(uint8_t*)compassData, 3);
}
