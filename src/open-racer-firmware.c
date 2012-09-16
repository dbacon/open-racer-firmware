//
//   Copyright 2012 Dave Bacon
//
//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this file except in compliance with the License.
//   You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.
//

//
// open-racer-firmware.c
//
//  Created on: Sep 8, 2012
//      Author: dave
//


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

#include "uart.h"


//
//  This is a firmware for the Dagu WirelessControl Car or i-racer,
//  sold by sparkfun as the i-Racer:
//  https://www.sparkfun.com/products/11162
//
//  This board has the ISP header routed so we can program it.
//
//  F_CPU=8000000UL // shipped with 8 MHz clock configured
//
//  A USART is connected to an on-board bluetooth module for a wireless
//  serial port.  It's worth noting the bluetooth module's reset is not
//  connected.  Mostly this means the MCU can't expect the bluetooth
//  module to be in command-mode after an MCU reset.  The bluetooth
//  module will only accept command-mode communication after board is
//  power-cycled.
//
//  The original firmware provided source had the "bluetooth-connected"
//  signal on port PB0, presumably matching an earlier board revision.
//  The board revision assumed in this program seems to have this
//  signal on PD4.
//
//  It has both PWM channels (0a & 0b) of timer1 dedicated to the
//  steering motor, one PWM to 'left', and one PWM to 'right' (as
//  opposed to having a direction bit control an H-bridge with a single
//  PWM controlling speed).  Same with 2 more PWM channels on timer2
//  for the drive motor - 1 PWM for forward, 1 more PWM for reverse
//  drive.
//
//  Then a 5th PWM is/can be used for the "breathing" blue LED.
//
//  Four more pins are routed to headers on the board, and there are
//  pads for an accelerometer or something.
//
//  The basic function of the first versions of this program is to
//  feature-match the provided firmware from the i-Racer product.
//  Some implementation (such as uart_expect()) is directly emulated or
//  based on the provided code.
//
// Original firmware function feature-matching checklist:
//
// Done:
//  = PWM steer left, PWM steer right
//  = PWM drive forward, PWM drive reverse
//  = blue breathing LED
//  = electro-mechanical self-test/-demo
//  = watch-dog timer
//  = calibrated busy-wait delay functions
//  = eeprom read/store init vars
//  = setup usart to bluetooth module
//  = command-loop - commands from usart (bluetooth module)
//  = command motors for selected speed/steer
//  = set bluetooth name
//  = battery voltage monitor via ADC
//  = low battery warn state
//  = setup bluetooth connected signal input pin
//  = bluetooth disconnect state detect and motor halt
//  + info display of cause of reset
//
// Left TODO:
//
//  = display selected speed grade
//
//  + convert serial comm from polled to interrupt, out of the mainloop
//  + low-battery state should sleep, not continue to busy-loop poll
//  + optimize power usage - extend battery life
//  * move string constants to flash section
//  * Remote control software - Android app
//


uint8_t resetflags __attribute__((section(".noinit")));

void save_and_clear_mcucsr()
	__attribute__((section(".init3")))
	__attribute__((naked));

void save_and_clear_mcucsr() {
	resetflags = MCUSR;
	MCUSR = 0;
	wdt_disable();
}

static void delay(uint8_t p) {
	while (p-- > 0) __asm__ volatile("nop");
}

/** delaying 10 == 1ms */
static void delay_100us(uint8_t p) {
	while (p-- > 0) delay(133);
}

/** delaying 100 == 1s */
static void delay_10ms(uint8_t hundredths) {
	while (hundredths-- > 0) delay_100us(100);
}

#define setb(_PORT, _PIN) do { _PORT |=  (1 << _PIN); } while (0)
#define clrb(_PORT, _PIN) do { _PORT &= (unsigned char)~(1 << _PIN); } while (0)



#define led1_port PORTC
#define led1_pin 5
#define led2_port PORTC
#define led2_pin 4
#define led3_port PORTC
#define led3_pin 3
#define led4_port PORTC
#define led4_pin 2
#define led5_port PORTD
#define led5_pin 3

#define led1on()  setb(led1_port, led1_pin)
#define led1off() clrb(led1_port, led1_pin)
#define led2on()  setb(led2_port, led2_pin)
#define led2off() clrb(led2_port, led2_pin)
#define led3on()  setb(led3_port, led3_pin)
#define led3off() clrb(led3_port, led3_pin)
#define led4on()  setb(led4_port, led4_pin)
#define led4off() clrb(led4_port, led4_pin)
#define led5on()  setb(led5_port, led5_pin)
#define led5off() clrb(led5_port, led5_pin)

static void flash_reset_flag_info(uint8_t resetflags, uint8_t repeat) {

	// first, show which is led1 for reference
	for (uint8_t i = 0; i < 5; ++i) {
		led1on();
		delay_10ms(2);
		led1off();
		delay_10ms(2);
	}

	// led1-4 reflect which reset flags were set
	for (uint8_t i = 0; i < repeat; ++i) {

		if ((resetflags & _BV(PORF)) == _BV(PORF)) led1on();
		if ((resetflags & _BV(EXTRF)) == _BV(EXTRF)) led2on();
		if ((resetflags & _BV(BORF)) == _BV(BORF)) led3on();
		if ((resetflags & _BV(WDRF)) == _BV(WDRF)) led4on();
		delay_10ms(4);

		led1off();
		led2off();
		led3off();
		led4off();
		delay_10ms(4);
	}

	delay_10ms(50);
}

static void flash_led1(uint8_t count, uint8_t rate) {
	led1off();
	delay_10ms(10);

	uint8_t d = 10;
	if (rate == 1) d = 5;
	if (rate == 2) d = 2;

	for (uint8_t i = 0; i < count; ++i) {
		led1on();
		delay_10ms(d);
		led1off();
		delay_10ms(d);
	}
}


#define OURMAGIC 0x47

uint8_t EEMEM ourmagic = OURMAGIC;
uint8_t EEMEM ourage = 0x00;

void check_magic_and_show_age() {

	flash_led1(4, 1);
	delay_10ms(20);


	uint8_t m = eeprom_read_byte(&ourmagic);
	if (m != OURMAGIC) {
		flash_led1(20, 2);
	}


	uint8_t age = eeprom_read_byte(&ourage);

	// show the age counter in 4 bits on the LEDs for 1 second
	led1off();
	led2off();
	led3off();
	led4off();

	if (age & _BV(0)) led1on();
	if (age & _BV(1)) led2on();
	if (age & _BV(2)) led3on();
	if (age & _BV(3)) led4on();

	delay_10ms(100);

	led1off();
	led2off();
	led3off();
	led4off();

	delay_10ms(50);
}

static void age_once() {
	uint8_t a = eeprom_read_byte(&ourage);
	eeprom_write_byte(&ourage, a+1);
}




static void motor_drive_forward(int16_t speed) {
	OCR1A = 0;
	OCR1B = speed;
}

static void motor_drive_reverse(int16_t speed) {
	OCR1B = 0;
	OCR1A = speed;
}

static void rev_motor_drive(uint8_t repeat) {

	uint8_t speed = 0;

	while (++speed < 255) {
		OCR1A = speed;
		delay_100us(50);
	}
	while (--speed > 0) {
		OCR1A = speed;
		delay_100us(50);
	}
	OCR1A = 0;

	while (++speed < 255) {
		OCR1B = speed;
		delay_100us(50);
	}
	while (--speed > 0) {
		OCR1B = speed;
		delay_100us(50);
	}
	OCR1B = 0;
}

static void pulse_motor_drive(uint8_t speed, uint8_t repeat) {
	OCR1A = 0x0000;
	OCR1B = 0x0000;

	for (uint8_t i = 0; i < repeat; ++ i) {
		OCR1A = speed;
		delay_10ms(10);
		OCR1A = 0;
		delay_10ms(10);
		OCR1B = speed;
		delay_10ms(10);
		OCR1B = 0;
		delay_10ms(10);
	}
}

static void motor_steer_right(uint8_t value) {
	OCR0B = 0;
	OCR0A = value;
}

static void motor_steer_left(uint8_t value) {
	OCR0A = 0;
	OCR0B = value;
}

static void pulse_motor_steering(uint8_t speed, uint8_t repeat) {

	// always 1 dir
	OCR0A = 0;
	OCR0B = 0;

	for (uint8_t i = 0; i < repeat; ++i) {

		OCR0A = speed;
		led1on();
		led2off();
		led3off();
		delay_10ms(10);

		OCR0A = 0;
		led1off();
		led2on();
		led3off();
		delay_10ms(10);

		OCR0B = speed;
		led1off();
		led2off();
		led3on();
		delay_10ms(10);

		OCR0B = 0;
		led1off();
		led2on();
		led3off();
		delay_10ms(10);

	}

	led1off();
	led2off();
	led3off();
	delay_10ms(20);
}


static int16_t velocity = 0;

static void motor_drive_set_velocity(int16_t newvelocity) {
	if (newvelocity > 255) newvelocity = 255;
	if (newvelocity < -255) newvelocity = -255;
	velocity = newvelocity;
	uart_send("drive="); uart_sendint(velocity); uart_sendch('\n');
	if (velocity >= 0) {
		motor_drive_forward(velocity & 0xff);
	} else {
		motor_drive_reverse(0xff - (velocity & 0xff));
	}
}


static int16_t steerposition = 0;

static void motor_steer_set_velocity(int16_t newsteerposition) {
	if (newsteerposition > 255) newsteerposition = 255;
	if (newsteerposition < -255) newsteerposition = -255;
	steerposition = newsteerposition;
	uart_send("steer="); uart_sendint(steerposition); uart_sendch('\n');
	if (steerposition >= 0) {
		motor_steer_right(steerposition & 0xff);
	} else {
		motor_steer_left(0xff - (steerposition & 0xff));
	}
}


static uint8_t bluetooth_connected() {
	// The original source shows it was on B0, but it
	// now seems to be found on D4.
	return bit_is_set(PIND, PIND4);
}


static void bluetooth_setname(char *name)
{
	uart_send("AT+NAME");
	uart_send(name);

	if (!uart_expect("OKsetname")) {
		flash_led1(2, 0);
	}
}

#define DAGU_DIR_0_STOP_STRAIGHT  (0)
#define DAGU_DIR_1_FORW_STRAIGHT  (1)
#define DAGU_DIR_2_BACK_STRAIGHT  (2)
#define DAGU_DIR_3_STOP_LEFT      (3)
#define DAGU_DIR_4_STOP_RIGHT     (4)
#define DAGU_DIR_5_FORW_LEFT      (5)
#define DAGU_DIR_6_FORW_RIGHT     (6)
#define DAGU_DIR_7_BACK_LEFT      (7)
#define DAGU_DIR_8_BACK_RIGHT     (8)

static void handle_char_compat_dagu(uint8_t command) {
	uint8_t speed = 105 + (command & 0x0f) * 1;
	uint8_t direction = (command & 0xf0) >> 4;

	switch (direction) {
	case DAGU_DIR_0_STOP_STRAIGHT:
		motor_steer_set_velocity(0);
		motor_drive_set_velocity(0);
		break;
	case DAGU_DIR_1_FORW_STRAIGHT:
		motor_steer_set_velocity(0);
		motor_drive_set_velocity(speed);
		break;
	case DAGU_DIR_2_BACK_STRAIGHT:
		motor_steer_set_velocity(0);
		motor_drive_set_velocity(-speed);
		break;
	case DAGU_DIR_3_STOP_LEFT:
		motor_steer_set_velocity(-255);
		motor_drive_set_velocity(0);
		break;
	case DAGU_DIR_4_STOP_RIGHT:
		motor_steer_set_velocity(255);
		motor_drive_set_velocity(0);
		break;
	case DAGU_DIR_5_FORW_LEFT:
		motor_steer_set_velocity(-255);
		motor_drive_set_velocity(speed);
		break;
	case DAGU_DIR_6_FORW_RIGHT:
		motor_steer_set_velocity(255);
		motor_drive_set_velocity(speed);
		break;
	case DAGU_DIR_7_BACK_LEFT:
		motor_steer_set_velocity(-255);
		motor_drive_set_velocity(-speed);
		break;
	case DAGU_DIR_8_BACK_RIGHT:
		motor_steer_set_velocity(255);
		motor_drive_set_velocity(-speed);
		break;
	}
}

static void handle_char(uint8_t command) {

	switch (command) {
	case 'R': motor_steer_set_velocity( 255); break;
	case 'r': motor_steer_set_velocity( 127); break;
	case 's': motor_steer_set_velocity(   0); break;
	case 'l': motor_steer_set_velocity(-127); break;
	case 'L': motor_steer_set_velocity(-255); break;

	case 'F': motor_drive_set_velocity( 255); break;
	case 'f': motor_drive_set_velocity( 127); break;
	case 'h': motor_drive_set_velocity(   0); break;
	case 'b': motor_drive_set_velocity(-127); break;
	case 'B': motor_drive_set_velocity(-255); break;

//	case 'u': motor_drive_set_velocity(velocity + 5); break;
//	case 'd': motor_drive_set_velocity(velocity - 5); break;
//	case '>': motor_steer_set_velocity(steerposition + 5); break;
//	case '<': motor_steer_set_velocity(steerposition - 5); break;

	// sorry, dvorak for now.
	case 'a': motor_steer_set_velocity(-255); break;
	case 'o': motor_steer_set_velocity(0); break;
	case 'e': motor_steer_set_velocity(255); break;

	case 'p': motor_drive_set_velocity(velocity + 5); break;
	case 'u': motor_drive_set_velocity(velocity - 5); break;

	case ' ':
		motor_drive_set_velocity(0);
		motor_steer_set_velocity(0);
		break;

	case 'A':
		check_magic_and_show_age();
		age_once();
		check_magic_and_show_age();
		break;

	case '?':
		uart_send("steering: RrslL\ngas: FfhbB\n");
		uart_send("batt="); uart_sendint(ADCH); uart_sendch('\n');
		break;

	default: uart_send("?\n"); break;
	}
}

static uint8_t battlevel = 11;

static void batt_sample() {
	uint8_t battsample = ADCH;

	// need to scale 142+0..142+48 -> 0..255, so *5
	battsample -= 142;
	if (battsample < 0) battsample = 0;
	battsample *= 5;
	if (battsample > 255) battsample = 255;

	battlevel = battsample;
}

static uint8_t batt_low_consistently(uint8_t threshold) {
	uint8_t warn = 0;
	uart_send("batt long check:");
	for (uint8_t i = 0; i < 20; ++i) {

		delay_100us(10);
		batt_sample();

		if (battlevel < threshold) {
			uart_send(" batt="); uart_sendint(battlevel);
			warn++;
		}
	}
	uart_send(" done.\n");

	return warn > 17;
}


#define battlowthreshold (10)

//// breathing blue led support
//uint8_t breathelevel = 0;
//uint8_t breathedirection = 1;

#define voltagedisplaytoggleperiodlength  (200)
static uint8_t voltagedisplaytogglecountdown = 10;
static uint8_t voltagedisplaytogglestate = 0;

#define battwarntoggleperiodlength (20)
static uint8_t battwarntogglecountdown = 0;
static uint8_t battwarntogglestate = 0;
// assume at least more than warning-threshold until 1st sample

#define mainloopdelay (40)


int main(void) {

	uart_init(9600);

	// ------------------------------------------------------------------------------

	//
	// Before main()
	//   * SREG is set to 0x00, and
	//   * stack-pointer SPH & SPL is assigned
	// 6 instructions plus jump to main
	//

	// ------------------------------------------------------------------------------

	cli();

	// sets outputs to high or enables internal pull-ups.
	PORTB = 0xff;
	PORTC = 0xff;
	PORTD = 0xff;

	// sets pins as inputs initially
	DDRB = 0;
	DDRC = 0;
	DDRD = 0;

	// ------------------------------------------------------------------------------

	// ADC channel monitoring battery charge

	ADMUX = 0;
	ADMUX |= _BV(REFS0);
	ADMUX |= _BV(ADLAR);

	ADCSRA = 0;
	ADCSRA |= _BV(ADPS1) | _BV(ADPS0); // prescale /8
	ADCSRA |= _BV(ADATE);
	ADCSRB = 0; // leave in free-running mode
	ADCSRA |= _BV(ADEN); // enable ADC
	ADCSRA |= _BV(ADSC); // start 1st conversion, triggering free-running mode

	// ------------------------------------------------------------------------------

	//  PWMs on steering motor

	// phase-correct 8-bit PWM (0xf1)
	TCCR0A = _BV(WGM00) | _BV(COM0A1) | _BV(COM0A0) | _BV(COM0B1) | _BV(COM0B0);
	TCCR0B = 0; // no clock yet // must not clobber and set WGM02
	OCR0A = 0;
	OCR0B = 0;
	TIMSK0 = 0; // no interrupts
	TIFR0 = 0xff; // clears match & overflow interrupt flags
	DDRD |= _BV(DD5); // (PCINT21/OC0B/T1) PD5
	DDRD |= _BV(DD6); // (PCINT22/OC0A/AIN0) PD6
	TCCR0B |= _BV(CS00); // clock on, no scaling


	// PWMs on drive motor

	// phase-correct 8-bit PWM (0xf1)
	TCCR1A = _BV(WGM10) | _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0);
	TCCR1B = 0; // no clock yet // must not clobber other pins by setting
	OCR1A = 0;
	OCR1B = 0;
	TIMSK1 = 0; // no interrupts
	TIFR1 = 0xff; // clears match & overflow interrupt flags
	DDRB |= _BV(DD1); // PB1 (OC1A/PCINT1)
	DDRB |= _BV(DD2); // PB2 (SS/OC1B/PCINT2)
	TCCR1B |= _BV(CS10); // clock on, no scaling


	// PWM for 'breathing' blue led

	TCCR2A = _BV(WGM20) | _BV(COM2B1) | _BV(COM2B0);
	TCCR2B = 0; // no clock yet // must not clobber and set other bits (WGM*)
	OCR2A = 0x00ff;
	OCR2B = 0x00ff;
	TIMSK2 = 0x00; // no interrupts
	TIFR2 = 0xff; // clears match & overflow interrupt flags
	DDRD |= _BV(DD3);
	//DDRD |= _BV(DD?); // OC2B not used
	TCCR2B = _BV(CS20); // clock on, no scaling

	// ------------------------------------------------------------------------------

	DDRC |= _BV(led1_pin);
	DDRC |= _BV(led2_pin);
	DDRC |= _BV(led3_pin);
	DDRC |= _BV(led4_pin);
	DDRD |= _BV(led5_pin);


	led1off();
	led2off();
	led3off();
	led4off();
	led5off();

	// ------------------------------------------------------------------------------

	// Can only do this on board power-cycle, not just any cpu reset.
	// If tried after only cpu reset, the bluetooth module isn't in command mode,
	// and awaiting a response would hang the show.
	if (0) bluetooth_setname("OpenRacer");

	// ------------------------------------------------------------------------------

	flash_reset_flag_info(resetflags, 10);
	pulse_motor_steering(0x60, 1);
	pulse_motor_drive(0x60, 1);
	if (0) rev_motor_drive(2);

	if (0)
	{
		check_magic_and_show_age();
		age_once();
		check_magic_and_show_age();
	}

	// ------------------------------------------------------------------------------

	wdt_enable(WDTO_8S);

	// ------------------------------------------------------------------------------

	while (1) {

		wdt_reset();

//		// breathing blue led support
//		breathelevel += breathedirection;
//		if (breathelevel <= 0) breathedirection = 1;
//		if (breathelevel >= 255) breathedirection = -1;
//		OCR2B = breathelevel;


		// Bluetooth signal indicator sampling.
		//
		// This is solid high when connected, and blinks when
		// unconnected--it does not directly reflect 'connected' state.
		//
		// So a low-level means we are disconnected, but a high level
		// does not mean we are connected.

		if (bluetooth_connected()) {
			//led2on();
		} else {
			//led2off();
			motor_drive_set_velocity(0);
			motor_steer_set_velocity(0);
		}


		if (uart_hasch()) {
			led4on();
			uint8_t command = uart_getch();
			led4off();

//			handle_char(command);
			handle_char_compat_dagu(command);
		}


		voltagedisplaytogglecountdown--;
		if (voltagedisplaytogglecountdown <= 0) {
			voltagedisplaytogglecountdown = voltagedisplaytoggleperiodlength;
			voltagedisplaytogglestate = !voltagedisplaytogglestate;
		}

		if (voltagedisplaytogglestate) {
			batt_sample();

			OCR2B = 0xff - battlevel;
		} else {
			OCR2B = 0xff - 0xff;
		}


		battwarntogglecountdown--;
		if (battwarntogglecountdown <= 0) {
			battwarntogglecountdown = battwarntoggleperiodlength;
			battwarntogglestate = !battwarntogglestate;
		}

		if (battlevel < battlowthreshold) {
			uart_send("batt="); uart_sendint(battlevel); uart_sendch('\n');
			if (batt_low_consistently(battlowthreshold)) {
				if (battwarntogglestate) led3on(); else led3off();
				motor_drive_set_velocity(0);
				motor_steer_set_velocity(0);
			}
		}


		delay_100us(mainloopdelay);
	}

	return 0;
}
