/*
 * TTerm
 *
 * Copyright (c) 2020 Thorben Zethoff, Jens Kerrinnes
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

#ifndef TTerm_H
#define TTerm_H

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"

#include "TTerm_VT100.h"
#include "TTerm/TTerm_config.h"
#include "stdint.h"

#define EXTENDED_PRINTF 1
#define TERM_VERSION_STRING "V0.9"
#define TERM_PROG_BUFFER_SIZE 32

#define NO_OUTPUT 0xFF

#define CTRL_C 0x03

#if PIC32 == 1 
    #define START_OF_FLASH  0xa0000000
    #define END_OF_FLASH    0xa000ffff
#else
    #define START_OF_FLASH  0x00000000
    #define END_OF_FLASH    0x1FFF8000
#endif

#define TERM_HISTORYSIZE 16
#define TERM_INPUTBUFFER_SIZE 128

                       
#define TERM_ARGS_ERROR_STRING_LITERAL 0xffff

#define TERM_CMD_EXIT_ERROR 0
#define TERM_CMD_EXIT_NOT_FOUND 1
#define TERM_CMD_EXIT_SUCCESS 0xff
#define TERM_CMD_EXIT_PROC_STARTED 0xfe
#define TERM_CMD_CONTINUE 0x80

#if TERM_SUPPORT_CWD == 1
    #define TERM_DEVICE_NAME handle->cwdPath
#else
#ifndef TERM_DEVICE_NAME
    #define TERM_DEVICE_NAME "MESC"
#endif
#endif

#ifdef TERM_ENABLE_STARTUP_TEXT
const extern char TERM_startupText1[];
const extern char TERM_startupText2[];
const extern char TERM_startupText3[];
#endif


#if EXTENDED_PRINTF == 1
#define ttprintfEcho(format, ...) if(handle->echoEnabled) (*handle->print)(handle->port, format, ##__VA_ARGS__)
#else
#define ttprintfEcho(format, ...) if(handle->echoEnabled) (*handle->print)(format, ##__VA_ARGS__)
#endif

#if EXTENDED_PRINTF == 1
#define ttprintf(format, ...) (*handle->print)(handle->port, format, ##__VA_ARGS__)
#else
#define ttprintf(format, ...) (*handle->print)(format, ##__VA_ARGS__)
#endif

//entity holding data of an open terminal
typedef struct __TERMINAL_HANDLE__ TERMINAL_HANDLE;


typedef struct __TermCommandDescriptor__ TermCommandDescriptor;

typedef enum {
    TERM_VARIABLE_INT,
	TERM_VARIABLE_INT_ARRAY,
    TERM_VARIABLE_UINT,
	TERM_VARIABLE_UINT_ARRAY,
	TERM_VARIABLE_FLOAT,
	TERM_VARIABLE_FLOAT_ARRAY,
	TERM_VARIABLE_CHAR,
	TERM_VARIABLE_STRING,
	TERM_VARIABLE_BOOL,
} TermVariableType;

typedef enum {
    FLAG_TELEMETRY_ON = 0x01,			//Bit 0
} TermFlagType;

#define TERM_addVar(var, min, max, name, description, rw, cb, listHandle) _Generic((var), \
    uint8_t:    TERM_addVarUnsigned, \
    uint16_t:   TERM_addVarUnsigned, \
    uint32_t:   TERM_addVarUnsigned, \
    int8_t:     TERM_addVarSigned, \
    int16_t:    TERM_addVarSigned, \
    int32_t:    TERM_addVarSigned, \
    float:      TERM_addVarFloat, \
	float*:		TERM_addVarArrayFloat, \
	bool:		TERM_addVarBool, \
    char:       TERM_addVarChar, \
    char*:      TERM_addVarString)(&var, sizeof(var), min, max, name, description, rw, cb, listHandle)


typedef uint32_t (* nvm_clear)(void * address, uint32_t len);
typedef uint32_t (* nvm_start_write)(void * address, void * buffer, uint32_t len);
typedef uint32_t (* nvm_write)(void * address, void * buffer, uint32_t len);
typedef uint32_t (* nvm_end_write)(void * address, void * buffer, uint32_t len);


typedef struct __TermVariableDescriptor__ TermVariableDescriptor;

typedef void (* term_var_cb)(TermVariableDescriptor * var);

struct __TermVariableDescriptor__{
    void * variable;
    TermVariableType type;
    uint16_t typeSize;
    const char * prefix;
    const char * name;
    uint32_t nameLength;
    const char * variableDescription;
    union{
    	int32_t min_signed;
		uint32_t min_unsigned;
		float min_float;
    };
    union{
       	int32_t max_signed;
   		uint32_t max_unsigned;
   		float max_float;
    };
    uint8_t rw;
    uint32_t flags;
    term_var_cb cb;
    TermVariableDescriptor * nextVar;
};

typedef struct __TermVariableHandle__ TermVariableHandle;

struct __TermVariableHandle__{
	TermVariableDescriptor * varListHead;
	uint32_t nvm_revision;
	uint32_t nvm_size;
	void * nvm_address;
	nvm_clear nvm_clear;
	nvm_start_write nvm_start_write;
	nvm_write nvm_write;
	nvm_end_write nvm_end_write;
};

uint16_t toLowerCase(uint16_t c);


#if TERM_SUPPORT_APPS
#include "Tasks/apps.h"
#endif

#if TERM_SUPPORT_VARIABLES
#include <TTerm/Core/include/TTerm_var.h>

enum TTermVarAccess
{
    VAR_ACCESS_NONE = 0x0,

    VAR_ACCESS_R    = 0x1,
    VAR_ACCESS_W    = 0x2,
	VAR_ACCESS_T 	= 0x4,

    VAR_ACCESS_RO   = VAR_ACCESS_R,
    VAR_ACCESS_WO   = VAR_ACCESS_W,
    VAR_ACCESS_RW   = (VAR_ACCESS_R | VAR_ACCESS_W),
	VAR_ACCESS_TR   = (VAR_ACCESS_R | VAR_ACCESS_T),
};

#endif

typedef uint8_t (* TermCommandFunction)(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
typedef uint8_t (* TermCommandInputHandler)(TERMINAL_HANDLE * handle, uint16_t c);
typedef uint8_t (* TermErrorPrinter)(TERMINAL_HANDLE * handle, uint32_t retCode);

#if EXTENDED_PRINTF == 1
typedef uint32_t (* TermPrintHandler)(void * port, const char * format, ...);
#else
typedef int (* TermPrintHandler)(const char * format, ...);
#endif
typedef uint8_t (* TermAutoCompHandler)(TERMINAL_HANDLE * handle, void * params);

typedef struct{
    TaskHandle_t task;
    TermCommandInputHandler inputHandler;
    uint8_t raw_input;
    StreamBufferHandle_t inputStream;
    char ** args;
    uint8_t argCount;
} TermProgram;

struct __TermCommandDescriptor__{
    TermCommandFunction function;
    const char * command;
    const char * commandDescription;
    uint32_t commandLength;
    uint8_t minPermissionLevel;
    TermAutoCompHandler ACHandler;
    void * ACParams;
    
    TermCommandDescriptor * nextCmd;
};

#define TTERM_ESC_SEQ_BUFFER_SIZE 16

struct __TERMINAL_HANDLE__{
    char * inputBuffer;
    #if EXTENDED_PRINTF == 1
    void * port;
    #endif        
    uint32_t currBufferPosition;
    uint32_t currBufferLength;
    uint32_t currAutocompleteCount;
    TermProgram * currProgram;
    char ** autocompleteBuffer;
    uint32_t autocompleteBufferLength;
    uint32_t autocompleteStart;    
    TermPrintHandler print;
    char * currUserName;
    uint8_t userPermissionLevel;
    uint8_t currPermissionLevel;
    char * historyBuffer[TERM_HISTORYSIZE];
    uint32_t currHistoryWritePosition;
    uint32_t currHistoryReadPosition;
    uint8_t currEscSeqPos;
    uint8_t escSeqBuff[TTERM_ESC_SEQ_BUFFER_SIZE];
    unsigned echoEnabled;
    TermCommandDescriptor * cmdListHead;
#if TERM_SUPPORT_VARIABLES
    TermVariableHandle * varHandle;
#endif
    TermErrorPrinter errorPrinter;
//TODO actually finish implementing this...
#if TERM_SUPPORT_CWD == 1
    //DIR cwd;
    char * cwdPath;
#endif
};

typedef enum{
    TERM_CHECK_COMP_AND_HIST = 0b11, TERM_CHECK_COMP = 0b01, TERM_CHECK_HIST = 0b10, 
} COPYCHECK_MODE;

extern TermCommandDescriptor TERM_defaultList; 

#if TERM_SUPPORT_VARIABLES
extern TermVariableDescriptor TERM_varList;
#endif

extern TERMINAL_HANDLE null_handle;

#if EXTENDED_PRINTF == 1
TERMINAL_HANDLE * TERM_createNewHandle(TermPrintHandler printFunction, void * port, unsigned echoEnabled, TermCommandDescriptor * cmdListHead, TermErrorPrinter errorPrinter, const char * usr);
#else
TERMINAL_HANDLE * TERM_createNewHandle(TermPrintHandler printFunction, unsigned echoEnabled, TermCommandDescriptor * cmdListHead, TermErrorPrinter errorPrinter, const char * usr);    
#endif    
void TERM_destroyHandle(TERMINAL_HANDLE * handle);
uint8_t TERM_processBuffer(uint8_t * data, uint16_t length, TERMINAL_HANDLE * handle);
unsigned isACIILetter(char c);
uint8_t TERM_handleInput(uint16_t c, TERMINAL_HANDLE * handle);
char * strnchr(char * str, char c, uint32_t length);
void strsft(char * src, int32_t startByte, int32_t offset);
void TERM_printBootMessage(TERMINAL_HANDLE * handle);
void TERM_freeCommandList(TermCommandDescriptor ** cl, uint16_t length);
uint8_t TERM_buildCMDList();
TermCommandDescriptor * TERM_addCommand(TermCommandFunction function, const char * command, const char * description, uint8_t minPermissionLevel, TermCommandDescriptor * head);
void TERM_addCommandAC(TermCommandDescriptor * cmd, TermAutoCompHandler ACH, void * ACParams);
unsigned TERM_isSorted(TermCommandDescriptor * a, TermCommandDescriptor * b);
void TERM_setCursorPos(TERMINAL_HANDLE * handle, uint16_t x, uint16_t y);
void TERM_sendVT100Code(TERMINAL_HANDLE * handle, uint16_t cmd, uint8_t var);
const char * TERM_getVT100Code(uint16_t cmd, uint8_t var);
uint16_t TERM_countArgs(const char * data, uint16_t dataLength);
uint8_t TERM_interpretCMD(char * data, uint16_t dataLength, TERMINAL_HANDLE * handle);
uint8_t TERM_seperateArgs(char * data, uint16_t dataLength, char ** buff);
void TERM_checkForCopy(TERMINAL_HANDLE * handle, COPYCHECK_MODE mode);
void TERM_printDebug(TERMINAL_HANDLE * handle, char * format, ...);
void TERM_removeProgramm(TERMINAL_HANDLE * handle);
void TERM_attachProgramm(TERMINAL_HANDLE * handle, TermProgram * prog);
void TERM_killProgramm(TERMINAL_HANDLE * handle);
uint8_t TERM_doAutoComplete(TERMINAL_HANDLE * handle);
uint8_t TERM_findMatchingCMDs(char * currInput, uint8_t length, char ** buff, TermCommandDescriptor * cmdListHead);
TermCommandDescriptor * TERM_findCMD(TERMINAL_HANDLE * handle);
uint8_t TERM_findLastArg(TERMINAL_HANDLE * handle, char * buff, uint8_t * lenBuff);
BaseType_t ptr_is_in_ram(void* ptr);
uint8_t TERM_defaultErrorPrinter(TERMINAL_HANDLE * handle, uint32_t retCode);
void TERM_LIST_add(TermCommandDescriptor * item, TermCommandDescriptor * head);
void TERM_Box(TERMINAL_HANDLE * handle, uint8_t row1, uint8_t col1, uint8_t row2, uint8_t col2);

#include "TTerm_cwd.h"

#endif
