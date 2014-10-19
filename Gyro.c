//For L3G4200D Gyroscope

#include "Gyro.h"
#include "I2C.h"
#include <util/delay.h>

void Gyro_Setup() {
    I2C_Setup();
    uint8_t value[] = {13,//0xCF, //CTRL_REG1: data update rate 100Hz
		       0x00,
		       0x00,
		       0x90, //CTRL_REG4: BlockDataUpdate
		       0x00}; 
    I2C_WriteReg(GYRO_ADDR, CTRL_REG1, value, 5);
    _delay_ms(250);

    //value = 0;
    //I2C_ReadReg(GYRO_ADDR, CTRL_REG1, &value, 2);
    //_delay_ms(250);
    //testFlag = value >> 8;
}

void readGyroData(volatile gyroData* gData) {
    I2C_ReadReg(GYRO_ADDR, OUT_X_L, (uint8_t*)(gData), 6);
}

