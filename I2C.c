// Functions to use AVR as I2C master. Supports multi-byte read/writes.

#define F_SLC 400000 // I2C clock frequency
#define SLA_W 0
#define SLA_R 1

#define START 0x08  // Status: START condition has been transmitted
#define RESTART 0x10  // Status: REPEATED START condition has been transmitted
#define SLA_W_ACK 0x18  // Status: transmitted SLA+W, recieved ACK
#define SLA_R_ACK 0x40  // Status: transmitted SLA+R, recieved ACK
#define DATA_R_ACK 0x50  // Status: data recieved, returned ACK
#define DATA_R_NACK 0x58  // Status: data recieved, returned NACK
#define DATA_W_ACK 0x28  // Status: transmitted data, recieved ACK
#define DATA_W_NACK 0x30  // Status: transmitted data, recieved NACK

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "I2C.h"

static uint8_t I2C_Address;
static uint8_t Reg_Address;
static volatile uint8_t readFlag; //Set so that ISR knows whether this is a read or write
static volatile uint8_t I2C_complete; //Let anyone who wants to know that read or write sequence is complete
static volatile uint8_t bytesLeft; //Number of bytes remaining to be written
volatile uint8_t testFlag = 0; //FOR DEBUG
static volatile uint8_t* volatile data; //pointer to the next byte of data to be written

void I2C_Setup(void)
{
    TWSR = 0; //No prescale
    TWBR = (F_CPU/F_SLC - 16)/2; //Set bit rate register to get desired I2C clock frequency
}

void I2C_ReadReg(uint8_t deviceAddr, uint8_t reg, uint8_t* dest, uint8_t bytes)
{
    I2C_Address = deviceAddr;
    Reg_Address = reg + 0x80; //Msb = 1 for multibyte read
    data = dest;
    bytesLeft = bytes;
    
    I2C_complete = 0;
    readFlag = 1; //read operation
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWIE); //Set TWINT bit to clear TWINT flag, Generate START, enable TWI circuits, enable TWI interrupts
}

void I2C_WriteReg(uint8_t deviceAddr, uint8_t reg, uint8_t* src, uint8_t bytes)
{
    I2C_Address = deviceAddr;
    Reg_Address = reg + 0x80; //Msb = 1 for multibyte write
    data = src;
    bytesLeft = bytes;
    
    I2C_complete = 0;
    readFlag = 0; //write operation
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWIE); //Set TWINT bit to clear TWINT flag, Generate START, enable TWI circuits, enable TWI interrupts
}

ISR(TWI_vect)
{
    uint8_t status = TWSR & 0xF8; //Mask prescale bits of status register for future compatibility

    switch (status) {
    case START: 
	TWDR = I2C_Address + SLA_W;
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
    case RESTART:
	if (readFlag) TWDR = I2C_Address + SLA_R;
	else TWDR = I2C_Address +SLA_W; //NOTE Is this even necessary? We should never send restart in write sequence.
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
	break;
    case SLA_W_ACK:
	TWDR = Reg_Address;
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
	break;
    case DATA_W_ACK:
	if (readFlag) TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWIE);  //Repeated Start (and then SLA+R) 
	else 
	{
	    if (bytesLeft == 0) //We have written all data
	    {
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWSTO); //Send STOP
	    }
	    else //send next byte
	    {
		TWDR = *data++; //NOTE Increments pointer, not data (right?)
		bytesLeft--;
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
	    }
	}
	break;
    case DATA_W_NACK:
	break;
    case SLA_R_ACK:
	if(bytesLeft == 1) {
	    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE); //Acknowledge and wait for data, send NACK after data
	}
	else {
	    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA); //Acknowledge and wait for data, send ACK after data
	}
	break;
    case DATA_R_ACK:
	*data++ = TWDR;
	bytesLeft--;
	if(bytesLeft <= 1) {
	    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE); //Acknowledge and wait for data, send NACK after data
	}
	else {
	    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA); //Acknowledge and wait for data, send ACK after data
	}
	break;
    case DATA_R_NACK:
	*data = TWDR;
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWSTO); //Send STOP
	break;
    default:
	break;
    } 
}

