/*
 * TTerm
 *
 * Copyright (c) 2020 Thorben Zethoff
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

 #if PIC32 == 1
#include <xc.h>
#endif  
#include <stdint.h>

#include "TTerm.h"
#include "TTerm_AC.h"

#ifndef TTERM_CMD
#define TTERM_CMD

extern AC_LIST_HEAD * head;

uint8_t CMD_testCommandHandler(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t TERM_testCommandAutoCompleter(TERMINAL_HANDLE * handle, void * params);
uint8_t CMD_help(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_cls(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_reset(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_top(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
void CMD_top_task(void * handle);
uint8_t CMD_top_handleInput(TERMINAL_HANDLE * handle, uint16_t c);

#endif