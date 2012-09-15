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

#ifndef __uart_h__
#define __uart_h__

#include <inttypes.h>


void uart_init(uint16_t baud);


void uart_sendch(uint8_t ch);

uint8_t uart_hasch();

uint8_t uart_getch();


uint8_t uart_expect(char *data);

void uart_send(char *data);

void uart_sendint(int16_t v);


#endif // __uart_h__
