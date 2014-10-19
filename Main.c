/* John Gallagher - jgallag88 _at_ gmail
 *
 * Basic communications for a quadcopter. The AVR reads angular velocity data
 * and acceleration from a digital gyroscope and accelerometer using I2C, and
 * integrates the data to get approximate position and orientation information.
 * For now, the output is read back over UART for analysis and testing.
 */

/************************* DEFINITIONS ********************************/
//#define F_CPU 20000000  //This is now defined in makefile to prevent us from having to define it in multiple source files
#define PWM_FREQ 70
#define CONTROL_FREQ 100
#define TIMER_PRESCALE 256
#define USART_BAUDRATE 19200 //9600
#define BAUD_PRESCALE (((F_CPU / USART_BAUDRATE / 16)) - 1)
#define BUFFER_SIZE 64
#define X_ZERO_RATE 0

/************************** HEADER FILES  ******************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "I2C.h"
#include "Gyro.h"
#include "Compass.h"

/*********************** FUNCTION PROTOTYPES ***********************/
void Timer_Setup(void);
void USART_Setup(void);
void putChar(char);
void outString(char *);
void outUnDec(uint16_t);
void outDec(int16_t);

/*************************   GLOBAL VARIABLES   ********************/
static volatile gyroData gData = {0, 0, 0};
static volatile int16_t rateXShort;
static volatile int32_t angleXLong = 0; //Long version, to preserve accuracy
static volatile int16_t angleXShort = 0; //Truncated version, for math, printing
static uint16_t total_counts, counts_control; //This is the total number of counts neccesary to get desired control PWM frequency 
static volatile char RX_Buffer[BUFFER_SIZE]; //Buffer to hold recieved characters
static volatile char TX_Buffer[BUFFER_SIZE]; //Buffer to hold characters to be transmitted
static volatile unsigned char RX_BufferSize = 0, TX_BufferSize = 0; //These tell us how many characters we have in the buffer
static volatile unsigned char RX_BufferStart = 0, TX_BufferStart = 0; //These tell us where we start in the buffer. From Start to Start + Size is good data, rest is garbage.
static volatile char TX_Transmitting; //Flag to tell whether we are currently transmitting anything

volatile uint8_t newDataFlag;

/******************************* MAIN *********************************/
int main(void) {
    cli(); //Disable global interrupts
    DDRB = 0xFF; //Set all pins on port B to outputs

    //Calculate number of counts for control loop
    counts_control =  F_CPU / (TIMER_PRESCALE*CONTROL_FREQ);
  
    Timer_Setup();
    USART_Setup();

    sei(); //Enable global interrupts

    Gyro_Setup();
    Compass_Setup();

    //volatile gyroData gData;
    PORTB |= (1<<PB1); //Toggle PortB pin 1

    _delay_ms(1000);
    
    char CompassID[4] = {0};
    CompassID[3] = '\0'; // Null terminate
    readCompassData((uint8_t*)CompassID);
    _delay_ms(30);
    outString("Compass ID: ");
    outString(CompassID);

    for (;;) {      
        // Every so often, write out new data. This can be imprecise because it
        // is only used for transmitting data. All critical timing for integration
        // calculations is done using timers and interrupts.
	_delay_ms(10);

	while(!newDataFlag){}

	//BEGIN ATOMIC BLOCK
	// Need atomic block, so store status reg and disable interrputs so that we 
        // can copy 32-value without it getting trashed by ISR mid-copy.
	uint8_t status = SREG;
	cli();
	gyroData dataCopy = gData;
	int32_t tempLongAngle = angleXLong; 
	int16_t rateCopy = rateXShort;
	SREG = status; // Finished with atomic block, so restore interrupts
	//END ATOMIC BLOCK

        // Write out integrated angular velocity over UART
	outDec(rateCopy);
	angleXShort = tempLongAngle/574100; 
	outDec(angleXShort);
	int32_t foo = (angleXLong*250/(32768*100));
	angleXShort = (int16_t)foo;
	putChar(0x0A); //LF
	outString("yRate: ");
	outDec(gData.yRate);
	/outString("zRate: ");
	outDec(gData.zRate);
		
    }

    return(0);
}

/************************** SETUP FUNCTIONS ************************************/
void Timer_Setup(void) {
  // Set up timers
  TIMSK1 |= (1<<OCIE1A);  // Enable Output Compare 1A
  //TIMSK1 |= (1<<OCIE1B);  // Enable Output Compare 1B
  TCCR1B=0x04;  // 256 Prescale. doesn't work without some prescale being set.

}

void USART_Setup(void) {
  UCSR0B |= (1<<RXEN0) | (1<<TXEN0);  // Enable Transmit and Recieve circuits
  UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);  // Use 8 data bits
  
  UBRR0H = (unsigned char) (BAUD_PRESCALE>>8);  // Upper byte of baud prescale
  UBRR0L = (unsigned char) BAUD_PRESCALE;  // Lower byte of baud prescale 

  UCSR0B |= (1<<RXCIE0);  // Enable Rx interrupts

  // We don't want to enable UDRIE yet b/c whenever we are not transmitting, this interrupt would be constantly firing. Only turn it on after we start transmitting.
}


//Function to load a character into Tx buffer, to then be sent as USART gets to them
void putChar(char nextChar)
{
    // Might have to add something to turn off UDRE Interrupts while adding to buffer
    UCSR0B &= ~(1<<UDRIE0);

    if (TX_Transmitting == 0) {
	// Tansmit first character to get us started, Tx ISR will continue once we get started
	UDR0 = nextChar;
	TX_Transmitting = 1;
	UCSR0B |= (1<<UDRIE0);  // Enable UDRE0 Interrupt, which tell us when UDR0 is empty, and we can send the next byte
    }
    else {
	TX_Buffer[(TX_BufferStart + TX_BufferSize)%BUFFER_SIZE] = nextChar; //Add the next ASCII character to the end of buffer
	TX_BufferSize++;
    }

    UCSR0B |= (1<<UDRIE0);  // Re-enable Tx interrupts
}

//Function to print strings one character at time
void outString(char *stringPtr)
{
    char letter;
    while((letter = *stringPtr++)) 
	putChar(letter);
}

//Funtion to print out unsigned variables as decimals
void outUnDec(uint16_t numToPrint) {
    if(numToPrint > 9) {
	outDec(numToPrint/10); // Should this be outUnDec ?????
	outDec(numToPrint%10);
   }
    else {
	putChar('0' + numToPrint);
    }
}

//Funtion to print out signed variables as decimals
void outDec(int16_t numToPrint) {
    if (numToPrint < 0) {
	putChar('-');
	numToPrint =  - numToPrint;
    }
    outUnDec((uint16_t)numToPrint); // Unnecessary cast ??
}

/********************   ISRs ***********************************/
//Interrupt service routine for OC1A
//We are using this to turn on and off the LED
ISR(TIMER1_COMPA_vect) {
    PORTB |= (1<<PB0); //PortB pin1 on
    OCR1A += counts_control;
    rateXShort = gData.xRate - X_ZERO_RATE;
    angleXLong += gData.xRate - X_ZERO_RATE;
    newDataFlag=1;
    readGyroData(&gData);
    PORTB &= ~(1<<PB0); //PortB pin0 off
}


//Interrupt Service Routine for Rx line on USART
ISR(USART_RX_vect) {
  //As of right now, we have no check for a full buffer
  
  RX_Buffer[RX_BufferStart + RX_BufferSize] = UDR0; //Put new byte in next available element of array
  
  RX_BufferSize++; //Stack of data in buffer has increased by 1
}

// Interrupt Service Routine for Tx line on USART
ISR(USART_UDRE_vect) {
    // Wrap around to beginning of buffer if it gets to the end
    if (TX_BufferStart >= BUFFER_SIZE) {
	TX_BufferStart = 0;
    }
  
    // PutChar function should have already put first character into UDR0. Once that is done being transmitted, we keep loading the next character from the 
    // buffer until there are no more characters to send (TX_Buffer_Size == 0)
    if(TX_BufferSize == 0) {
	TX_Transmitting = 0;
	UCSR0B &= ~(1<<UDRIE0); // Turn off UDRE0 so that it isn't constantly firing while we aren't transmitting anything.
    }
    else {
	UDR0 = TX_Buffer[TX_BufferStart];
	TX_BufferStart++;
	TX_BufferSize--;
    }
}
