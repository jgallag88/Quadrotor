CC=avr-gcc
MCU=-mmcu=atmega48
OLEVEL=-Os
CFLAGS=$(MCU) $(OLEVEL) -Wall -std=c99 -Wl,--relax -g -D F_CPU=20000000
OCOPYFLAGS=avr-objcopy -j .text -j .data 

OBJECTS=Main.o I2C.o Gyro.o Compass.o

default: build upload

build: Main.hex

Main.hex: Main.elf
	 $(OCOPYFLAGS) -O ihex $< $@

Main.elf: $(OBJECTS)
	$(CC) $(CFLAGS) -o $@ $(OBJECTS)

Main.o: Main.c
	$(CC) $(CFLAGS) -c -o $@ $<

I2C.o: I2C.c
	$(CC) $(CFLAGS) -c -o $@ $<

Gyro.o: Gyro.c
	$(CC) $(CFLAGS) -c -o $@ $<

Compass.o: Compass.c
	$(CC) $(CFLAGS) -c -o $@ $<

upload: 
	avrdude -c avrispmkII -P usb -p m48 -U flash:w:Main.hex