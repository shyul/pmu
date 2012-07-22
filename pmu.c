/*
 * Firmware.c (Atmel AVR Studio 5) - Power button control logic
 *
 *  Author: Lophilo
 *
 * 1) VCC: 
      Always powered on by 5V when the board is plugged 
	  (NOT powered by the backup battery).
 * 2) PWRBTN: 
      An input from the ON/OFF button.
 * 3) PWRCTRL: 
	  An output to turn on and off the power regulators (MOSFET driver)
	  (toggle mode, initially shutdown the regulators when the board gets plugged).
 * 4) SHDn: 
      An input from ARM SHDNn to let ARM also trigger the shutdown event.
 * 5) DISABLE_PWRCTRL: 
      An input from ARM GPIO to let ARM prohibit the ON/OFF button to shutdown the board, 
      let ARM has enough time to save everything and trigger the SHDNn while SHDNn has 
	  the highest priority for shutting down.	  
 * 6) BTNEVT: 
      It has an output to ARM GPIO to tell the ON/OFF button is pressed while DISABLE_PWRCTRL 
	  is prohibiting the ON/OFF button directly control the shutdown.
 * 7) OTHER
      A pin connected to another ARM GPIO.
 * 8) SCK, MISO, MOSI
      Item 5, 6, 7 should be connected to the programming SPI pins of ATtiny, 
	  and let the ATtiny chip "live update-able"  
	  
This is the state table:

BOARD STATUS	DISABLE_PWRCTRL	BUTTON PRESS						
ON				ON				Delayed toggle off (5s long press)	
ON				OFF				Immediate toggle off
OFF				X				Immediate toggle on

SHDn should shutdown the board immediately in all states.

ATtiny <-> ARM must be open drain model due to different voltage. 

Weak pull up 4.7Kohm resistance to 3.3V between the 5V output 
pin of the ATtiny and the input pins of ARM if the ATtiny supports 
the open drain model.

ATtiny13A SOIC8 package, "Pico power" stuff
Soldered on XEM card and replaces the initial prototype DIPx4 switch

PROGRAMMING USING AVRDUDE and Arduino as ISP on Windows

c:\arduino-1.0\hardware\tools\avr\bin\avrdude.exe \
	-P COM21 -p t13 -b 19200 -c avrisp \
	-C c:\arduino-1.0\hardware\tools\avr\etc\avrdude.conf -U Firmware.hex 

PROGRAMMING ON LINUX

See Makefile, use `make program`

PROGRAMMING ISSUES

You may need to start/stop the programming a few times (5+) before the chip takes up the program.

These kind of errors are common (the Arduino programmer looks to be a bit unreliable):

	avrdude: stk500_initialize(): programmer not in sync, resp=0x15

If the LED on pin 13 comes on and/or you get an unrecognized device error:

	avrdude: Device signature = 0xffff00
	avrdude: Expected signature for ATtiny13 is 1E 90 07
	         Double check chip, or use -F to override this check

It possibly means that the chip is defective or incorrectly connected.

TODO

Programming pins definitely need to be connected to ARM
SHDn not connected to the programming pins as not GPIO

ATtiny:
PB0: input (Connects to SHDNn, series resistor, and another GPIO pin for ISP)
PB1: output (Wake-up trigger, pull up, and another GPIO pin for ISP)
PB2: input SCK (Never connects to SHDNn), GPIO, btn event inhibitor.
PB3 Power-btn input, pull up to VDD5V
PB4: Power-EN output, big-capacitor for EN delay at power-on, optional pull-up (100kOhm) to VDD5V
PB5 / nReset: Only connects to ARM general GPIO, with no startup activity; Pull-up with 4.7k

ATtiny13 PINOUT
                    ATtiny13 
OTHER (RESET)   -> PB5    VCC <- 5V
WKUP-BTN        -> PB3    PB2 <- DISABLE_PWRCTRL (<-SCK)
SHDNn-OUT 		<- PB4    PB1  -> SYS-WKUP (->MISO)
GND             <- GND    PB0 <- SYS-SHDNn (<-MOSI)

PB5 <- YELLOW <- RESET (10) [25]
PB0 <- GREEN <- MOSI (11) [20]
PB1 < ORANGE -> MISO (12) [21]
PB2 <- BLUE <- SCK (13) [22]


VCC <- RED <- 5V [5V]
GND -> BLACK -> GND [G]

USING ARDUINO AS ISP

Program:

https://raw.github.com/adafruit/ArduinoISP/master/ArduinoISP2.pde

Once programmed, insert the 10uF capacitor between Arduino RESET and GND pin (longer pin in GND).

TESTED WITH

ATtiny13-20PU: supply, 2.7 - 5.5, package -SU

TODO: 

NOTE: 

If the SCK pin is connected to the programmer (Arduino ISP)
the SCK/DISABLE_PWRCTRL will be enabled meaning that 
the ATtiny will go to CPU_CONTROL.

SELFPRGEN = [ ]
DWEN = [ ]
BODLEVEL = 4V3
RSTDISBL = [X]  (If the reset is not disabled, weird things happens a lot, eg: pull down SHDNn_OUT will randomly reset the PMU -- ATtiny13A)
SPIEN = [X]
EESAVE = [ ]
WDTON = [ ]
CKDIV8 = [X]
SUT_CKSEL = INTRCOSC_9MHZ6_14CK_0MS

HIGH = 0xF8 (valid)
LOW = 0x62 (valid)

 */ 

#define F_CPU 1200000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define SYS_SHDNn				PB0
#define SYS_WKUP				PB1
#define BTNDIS					PB2
#define WKUP_BTN				PB3
#define SHDNn_OUT				PB4

#define ANTI_GLITCH_NUM			10
#define BTN_HOLD_DELAY_NUM		2000
#define ANTI_GLITCH_MS			1
#define WKUP_PULSE_MS			100

static uint8_t					controller_state;
static uint8_t					last_state;
static uint8_t					next_state;
static uint32_t					i;
static uint8_t					btn_fail;

enum {OFF, OFF_SOFT, ON, BTN_PRESSED, BTN_PRESSED_BUT_CPU_HOLD};

void delay_detect(void)
{
	_delay_ms(20);
}

void delay_ANTI_GLITCH_MS(void)
{
	_delay_ms(ANTI_GLITCH_MS);
}

uint8_t is_button_pressed(void)
{
	return bit_is_clear(PINB, WKUP_BTN);
}

uint8_t is_shutdown_requested(void)
{
	return bit_is_clear(PINB, SYS_SHDNn);
}

void button_pressed_counter(void)
{
	if(is_button_pressed())
	{
		if(i < 65535) i++;
	} 
	else
	{
		if(i > 0) i--;
	}
}

void button_released_counter(void)
{
	if(!is_button_pressed())
	{
		if(i < 65535) i++;
	}
	else 
	{
		if(i > 0) i--;
	}
}

int main(void)
{
	DDRB |= (1 << SHDNn_OUT);		// Set SHDNn_OUT as output pin
	PORTB &= ~(1 << SHDNn_OUT);		// quickly turn off by setting to zero
	_delay_ms(500);					// Delay 500ms to stable the MCU when in high or low temperature condition
	
	cli(); 							// DISABLE INTERRUPTS
	DDRB &= ~(1 << SYS_WKUP);		// SYS_WKUP as Hi-Z input with pulled up resistor to output logic 1
	DDRB &= ~(1 << SYS_SHDNn);		// CPU shutdown-request input
	DDRB &= ~(1 << BTNDIS);			// CPU button-disable-request input
	DDRB &= ~(1 << WKUP_BTN);		// CPU Wake-up event and button event
	
	/* Assign the default values of the state machine */
	controller_state = OFF;
	last_state = OFF;
	next_state = ON;

	while(1)
	{
		DDRB &= ~(1 << SYS_WKUP);	// SYS_WKUP as Hi-Z input with pulled up resistor to output logic 1	
		i = 0;
		btn_fail = 0;
		
		switch(controller_state)
		{	
			case OFF:
				last_state = OFF;
				next_state = ON;
				PORTB &= ~(1 << SHDNn_OUT);
				if(is_button_pressed()) controller_state = BTN_PRESSED;
				else if(is_shutdown_requested())
				{
					delay_detect();
					if(is_shutdown_requested()) controller_state = OFF_SOFT;
				}
				break;
				
			case OFF_SOFT:
				last_state = OFF_SOFT;
				next_state = ON;
				PORTB &= ~(1 << SHDNn_OUT);
				if(is_button_pressed()) controller_state = BTN_PRESSED;
				else if(!is_shutdown_requested()) 
				{
					delay_detect();
					if(!is_shutdown_requested()) controller_state = ON;
				}
				break;
		
			case ON:
				last_state = ON;
				next_state = OFF;
				PORTB |= (1 << SHDNn_OUT);
				if(is_button_pressed())
				{
					if(bit_is_clear(PINB, BTNDIS)) controller_state = BTN_PRESSED_BUT_CPU_HOLD;
					else controller_state = BTN_PRESSED;
				}
				else if(is_shutdown_requested()) 
				{
					delay_detect();
					if(is_shutdown_requested()) controller_state = OFF_SOFT;
				}
				break;
			
			case BTN_PRESSED:
				i = 1;
				while(i < ANTI_GLITCH_NUM)
				{
					delay_ANTI_GLITCH_MS();
					button_pressed_counter();	// variable i will be operated in this function.
					if(i == 0)
					{
						btn_fail = 1;
						controller_state = last_state;
						break;
					}
				}
				if(btn_fail) break;
				i = 1;
				while(i < ANTI_GLITCH_NUM)
				{
					delay_ANTI_GLITCH_MS();
					button_released_counter();	// variable i will be operated in this function.
				}
				/* Send the WKUP button pressed event */
				DDRB |= (1 << SYS_WKUP);		// Set SYS_WKUP to output mode
				PORTB &= ~(1 << SYS_WKUP);		// Then pull it low
				_delay_ms(WKUP_PULSE_MS);		// Just make the pulse longer.
				controller_state = next_state;
				break;

			case BTN_PRESSED_BUT_CPU_HOLD:
				i = 1;
				while(i < ANTI_GLITCH_NUM)
				{
					delay_ANTI_GLITCH_MS();
					button_pressed_counter();	// variable i will be operated in this function.
					if(i == 0)
					{
						btn_fail = 1;
						controller_state = ON;
						break;
					}
				}
				if(btn_fail) break;
				/* Send the WKUP button pressed event */
				DDRB |= (1 << SYS_WKUP);		// Set SYS_WKUP to output mode
				PORTB &= ~(1 << SYS_WKUP);		// Then pull it low
				i = 1;
				//btn_fail = 0; 
				while(i < BTN_HOLD_DELAY_NUM)
				{
					delay_ANTI_GLITCH_MS();
					button_pressed_counter();	// variable i will be operated in this function.
					if(i == 0)
					{
						btn_fail = 1;
						controller_state = ON;
						DDRB &= ~(1 << SYS_WKUP);	// SYS_WKUP as Hi-Z input with pulled up resistor to output logic 1	
						break;
					}
				}
				//DDRB &= ~(1 << SYS_WKUP);	// SYS_WKUP as Hi-Z input with pulled up resistor to output logic 1	
				if(btn_fail != 1)
				{
					PORTB &= ~(1 << SHDNn_OUT);
					i = 1;
					while(i < ANTI_GLITCH_NUM)
					{
						delay_ANTI_GLITCH_MS();
						button_released_counter();	// variable i will be operated in this function.
					}
					controller_state = OFF;			
				}
				//while(1)
				//{
					//DDRB |= (1 << SYS_WKUP);
					//DDRB &= ~(1 << SYS_WKUP);
				//}
				break;
			
			default:
				//PORTB &= ~(1 << SHDNn_OUT);
				controller_state = OFF;
				break;
		}
	}
	
	return 0;
}




