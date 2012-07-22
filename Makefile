.PHONY: readfuses setfuses terminal

# two programmers were used:
# Arduino as an ISP programmer compatible with the stk500
# A real STK500 with High-Voltage Programmer option
PROGRAMMER=stk500v1 
PROGRAMMER_HVP=stk500hvsp
PORT=/dev/ttyUSB0
PORT_HVP=/dev/ttyUSB0

#AVRDUDE=avrdude -P ${PORT} -p t13 -c ${PROGRAMMER} -b 19200 -v -v -v -v -i 1
AVRDUDE=avrdude -P ${PORT} -p t13 -c ${PROGRAMMER} -b 19200 -v -v -v -v
AVRDUDE_HVP=avrdude -P ${PORT_HVP} -p t13 -c ${PROGRAMMER_HVP} -b 115200 -v -v -v -v

all: program
	@echo "program with `make program`"

firmware.hex: firmware.elf
	avr-objcopy -O ihex -R .eeprom firmware.elf $@

firmware.elf: Firmware.c
	/usr/bin/avr-gcc -std=c99 -Wall -Os -mmcu=attiny13 -o $@ Firmware.c

program: firmware.hex
	${AVRDUDE} -U flash:w:firmware.hex	

program_slow: firmware.hex
	${AVRDUDE} -U flash:w:firmware.hex	

program_hvp: firmware.hex
	${AVRDUDE_HVP} -U flash:w:firmware.hex

terminal: 
	${AVRDUDE} -tuF

readfuses:
	${AVRDUDE} -U hfuse:r:high.txt:s -U lfuse:r:low.txt:s -D 

setfuses:
	# see http://www.engbedded.com/fusecalc/
	# Int. RC Osc. 9.6 MHz; SYSCLK = 1.2 MHz Start-up time: 14 CK + 0 ms = 11.67 us (+ Voltage Ramp up to 2.7V time)
	# SPI downloading enabled
	# BOD = 01, 2.7V
	${AVRDUDE} -C/usr/share/arduino/hardware/tools/avrdude.conf -e -U lfuse:w:0x62:m -U hfuse:w:0xfb:m 

setfuses_hvp:
	${AVRDUDE_HVP} -C/usr/share/arduino/hardware/tools/avrdude.conf -e -U lfuse:w:0x62:m -U hfuse:w:0xfb:m -B250 -F

setfuses-slow:
	${AVRDUDE} -C/usr/share/arduino/hardware/tools/avrdude.conf -e -U lfuse:w:0x62:m -U hfuse:w:0xfb:m -B250 -F
