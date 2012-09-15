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

#include "uart.h"

#include <avr/io.h>

void uart_init(uint16_t baud) {

	UBRR0 = 8000000UL / 16 / baud - 1; // e.g. 51 for 9600, datasheet says 51 for 8mHz FOSC, 9600, single-rate
	UCSR0B = _BV(TXEN0) | _BV(RXEN0); // enable tx & rx
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // asynchronous, no parity, 1 stop bit, 8 bit char size
}

void uart_sendch(uint8_t ch) {
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = ch;
}

uint8_t uart_hasch() {
	return bit_is_set(UCSR0A, RXC0);
}

uint8_t uart_getch() {

	loop_until_bit_is_set(UCSR0A, RXC0);

//	if (bit_is_set(UCSR0A, FE0))  ; // TODO: handle frame error.
//	if (bit_is_set(UCSR0A, DOR0)) ; // TODO: handle data overrun error.
//	if (bit_is_set(UCSR0A, UPE0)) ; // TODO: parity error.

	return UDR0;
}

// this may hang if the uart doesn't have the amount of data you ask
uint8_t uart_expect(char *data) {
	uint8_t i = 0;
	uint8_t matches = 0;
	uint8_t count = 0;
	while (data[i] != 0) {
		uint8_t ch = uart_getch();
		++count;
		if (ch == data[i]) ++matches;
		++i;
	}
	return matches == count;
}

void uart_send(char *data) {
	for (uint8_t i = 0; data[i] != 0; ++i) {
		uart_sendch(data[i]);
	}
}

void uart_sendint(int16_t v) {
	if (v < 0) {
		v = -v;
		uart_sendch('-');
	}

	uint8_t th = '0' + (v%10000)/1000;
	uint8_t h  = '0' + (v%1000 )/ 100;
	uint8_t t  = '0' + (v%100  )/  10;
	uint8_t o  = '0' + (v%10   )/   1;

	uint8_t n = 0;
	if (n || th != '0') { n = 1; uart_sendch(th); }
	if (n || h != '0') { n = 1; uart_sendch(h); }
	if (n || t != '0') { n = 1; uart_sendch(t); }
	uart_sendch(o);
}

