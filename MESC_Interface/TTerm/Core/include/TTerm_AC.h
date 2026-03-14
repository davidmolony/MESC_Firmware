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
    
#ifndef Term_ACH
#define Term_ACH

#if PIC32 == 1
#include <xc.h>
#endif  
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "TTerm.h"

typedef struct __ACL__ AC_LIST_ELEMENT;
typedef struct __ACL_HEAD__ AC_LIST_HEAD;

struct __ACL__{
    AC_LIST_ELEMENT * next;
    char * string;
};

struct __ACL_HEAD__{
    unsigned isConst;
    uint32_t elementCount;
    AC_LIST_ELEMENT * first;
};

AC_LIST_HEAD * ACL_create();
AC_LIST_HEAD * ACL_createConst(char ** strings, uint32_t count);
AC_LIST_ELEMENT * ACL_getNext(AC_LIST_ELEMENT * currElement);
void ACL_add(AC_LIST_HEAD * head, char * string);
void ACL_remove(AC_LIST_HEAD * head, char * string);
uint8_t TERM_doListAC(AC_LIST_HEAD * head, char * currInput, uint8_t length, char ** buff);
uint8_t ACL_defaultCompleter(TERMINAL_HANDLE * handle, void * params);
unsigned ACL_isSorted(char * a, char * b);
void ACL_remove(AC_LIST_HEAD * head, char * string);
void ACL_add(AC_LIST_HEAD * head, char * string);

#define TERM_addCommandConstAC(CMDhandler, command, helptext, ACList, CmdDesc) TERM_addCommandAC(TERM_addCommand(CMDhandler, command,helptext,0,CmdDesc) \
                                                                                , ACL_defaultCompleter, ACL_createConst((char**)ACList, sizeof(ACList)/sizeof(char*)))


#endif