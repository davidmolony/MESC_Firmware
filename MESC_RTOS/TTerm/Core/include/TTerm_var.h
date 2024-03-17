/*
 * TTerm
 *
 * Copyright (c) 2022 Jens Kerrinnes
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


#ifndef TERM_VAR_H_
#define TERM_VAR_H_

#include <stdint.h>
#include "TTerm.h"

typedef enum {
    HELPER_FLAG_FLASH,
	HELPER_FLAG_DETAIL,
	HELPER_FLAG_DEFAULT,
} HelperFlagType;


//TermVariableDescriptor * TERM_addVar(void* variable, TermVariableType type, uint16_t typeSize , const char * name, const char * description, uint8_t rw, TermVariableDescriptor * head);
uint8_t CMD_varList(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_varSet(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_varSave(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_varLoad(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_su(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_varChown(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);

TermVariableDescriptor * TERM_addVarUnsigned(void* variable, uint16_t typeSize, uint32_t min, uint32_t max, const char * name, const char * description, uint8_t rw, term_var_cb cb, TermVariableDescriptor * head);
TermVariableDescriptor * TERM_addVarSigned(void* variable, uint16_t typeSize, int32_t min, int32_t max, const char * name, const char * description, uint8_t rw, term_var_cb cb, TermVariableDescriptor * head);
TermVariableDescriptor * TERM_addVarFloat(void* variable, uint16_t typeSize, float min, float max, const char * name, const char * description, uint8_t rw, term_var_cb cb, TermVariableDescriptor * head);
TermVariableDescriptor * TERM_addVarString(void* variable, uint16_t typeSize, uint32_t min, uint32_t max, const char * name, const char * description, uint8_t rw, term_var_cb cb, TermVariableDescriptor * head);
TermVariableDescriptor * TERM_addVarChar(void* variable, uint16_t typeSize, uint32_t min, uint32_t max, const char * name, const char * description, uint8_t rw, term_var_cb cb, TermVariableDescriptor * head);
TermVariableDescriptor * TERM_addVarBool(void* variable, uint16_t typeSize, uint32_t min, uint32_t max, const char * name, const char * description, uint8_t rw, term_var_cb cb, TermVariableDescriptor * head);
TermVariableDescriptor * TERM_addVarArrayFloat(void* variable, uint32_t size,  float min, float max, const char * name, const char * description, uint8_t rw, term_var_cb cb, TermVariableDescriptor * head);

void print_var_helperfunc(TERMINAL_HANDLE * handle, TermVariableDescriptor * var, HelperFlagType flag );
uint32_t TERM_var2str(TERMINAL_HANDLE * handle, TermVariableDescriptor * var, char * buffer, int32_t len );

void TERM_setFlag(TermVariableDescriptor * desc, TermFlagType flag);
void TERM_clearFlag(TermVariableDescriptor * desc, TermFlagType flag);


TermVariableHandle * TERM_VAR_init(TERMINAL_HANDLE * handle, void * nvm_address, uint32_t nvm_size, nvm_clear, nvm_start_write, nvm_write, nvm_end_write);

uint8_t TERM_varCompleter(TERMINAL_HANDLE * handle, void * params);


#endif
