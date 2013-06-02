/*

DESCRIPRION

 */

#ifndef I2C_H
#define I2C_H

#include <avr/io.h>

extern volatile uint8_t testFlag; //JUST FOR TESTING, NOT PERMANENT

void I2C_Setup(void); 
void I2C_ReadReg(uint8_t device, uint8_t reg, uint8_t* dest, uint8_t bytes);
void I2C_WriteReg(uint8_t device, uint8_t reg, uint8_t* src, uint8_t bytes); 
#endif 
