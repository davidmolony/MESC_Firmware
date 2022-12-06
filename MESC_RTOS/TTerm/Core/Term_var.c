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

#include "TTerm/Core/include/Term_var.h"
#include "TTerm/Core/include/TTerm.h"

#include <string.h>
#include <stdbool.h>

char toLower(char c){
    if(c > 65 && c < 90){
        return c + 32;
    }

    switch(c){
        case 'Ü':
            return 'ü';
        case 'Ä':
            return 'ä';
        case 'Ö':
            return 'ö';
        default:
            return c;
    }
}

static unsigned TERM_VAR_isSorted(TermVariableDescriptor * a, TermVariableDescriptor * b){
    uint8_t currPos = 0;
    //compare the lowercase ASCII values of each character in the command (They are alphabetically sorted)
    for(;currPos < a->nameLength && currPos < b->nameLength; currPos++){
        char letterA = toLower(a->name[currPos]);
        char letterB = toLower(b->name[currPos]);
        //if the letters are different we return 1 if a is smaller than b (they are correctly sorted) or zero if its the other way around
        if(letterA > letterB){
            return 1;
        }else if(letterB > letterA){
            return 0;
        }
    }

    //the two commands have identical letters for their entire length we check which one is longer (the shortest should come first)
    if(a->nameLength > b->nameLength){
        return 1;
    }else if(b->nameLength > a->nameLength){
        return 0;
    }else{
        //it might happen that a command is added twice (or two identical ones are added), in which case we just say they are sorted correctly and print an error in the console
        //TODO implement an alarm here
        //UART_print("WARNING: Found identical commands: \"%S\" and \"%S\"\r\n", a->command, b->command);
        return 1;
    }
}


void TERM_VAR_LIST_add(TermVariableDescriptor * item, TermVariableDescriptor * head){
    uint32_t currPos = 0;
    TermVariableDescriptor ** lastComp = &head->nextVar;
    TermVariableDescriptor * currComp = head->nextVar;

    while(currPos < head->nameLength){
        if(TERM_VAR_isSorted(currComp, item)){
            *lastComp = item;
            item->nextVar = currComp;
            head->nameLength ++;

            return;
        }
        if(currComp->nextVar == 0){
            item->nextVar = currComp->nextVar;
            currComp->nextVar = item;
            head->nameLength ++;
            return;
        }
        lastComp = &currComp->nextVar;
        currComp = currComp->nextVar;
    }

    item->nextVar = 0;
    *lastComp = item;
    head->nameLength ++;
}


//TermVariableDescriptor * TERM_addVar(void* variable, TermVariableType type, uint16_t typeSize, const char * name, const char * description, uint8_t rw, TermVariableDescriptor * head){
//    //if(head == NULL) head = TERM_defaultList;
//
//    if(head->nameLength == 0xff) return 0;
//
//    TermVariableDescriptor * newVAR = pvPortMalloc(sizeof(TermVariableDescriptor));
//
//    newVAR->variable = variable;
//    newVAR->type = type;
//    newVAR->typeSize = typeSize;
//    newVAR->name = name;
//    newVAR->nameLength = strlen(name);
//    newVAR->variableDescription = description;
//    newVAR->rw = rw;
//
//    TERM_VAR_LIST_add(newVAR, head);
//    return newVAR;
//}

TermVariableDescriptor * TERM_addVarUnsigned(void* variable, uint16_t typeSize, uint32_t min, uint32_t max, const char * name, const char * description, uint8_t rw, TermVariableDescriptor * head){
    //if(head == NULL) head = TERM_defaultList;
    if(head->nameLength == 0xff) return 0;

    TermVariableDescriptor * newVAR = pvPortMalloc(sizeof(TermVariableDescriptor));

    newVAR->variable = variable;
    newVAR->min_unsigned = min;
    newVAR->max_unsigned = max;
    newVAR->type = TERM_VARIABLE_UINT;
    newVAR->typeSize = typeSize;
    newVAR->name = name;
    newVAR->nameLength = strlen(name);
    newVAR->variableDescription = description;
    newVAR->rw = rw;

    TERM_VAR_LIST_add(newVAR, head);
    return newVAR;
}

TermVariableDescriptor * TERM_addVarSigned(void* variable, uint16_t typeSize, int32_t min, int32_t max, const char * name, const char * description, uint8_t rw, TermVariableDescriptor * head){
    //if(head == NULL) head = TERM_defaultList;
    if(head->nameLength == 0xff) return 0;

    TermVariableDescriptor * newVAR = pvPortMalloc(sizeof(TermVariableDescriptor));

    newVAR->variable = variable;
    newVAR->min_signed = min;
    newVAR->max_signed = max;
    newVAR->type = TERM_VARIABLE_INT;
    newVAR->typeSize = typeSize;
    newVAR->name = name;
    newVAR->nameLength = strlen(name);
    newVAR->variableDescription = description;
    newVAR->rw = rw;

    TERM_VAR_LIST_add(newVAR, head);
    return newVAR;
}

TermVariableDescriptor * TERM_addVarFloat(void* variable, float min, float max, const char * name, const char * description, uint8_t rw, TermVariableDescriptor * head){
    //if(head == NULL) head = TERM_defaultList;
    if(head->nameLength == 0xff) return 0;

    TermVariableDescriptor * newVAR = pvPortMalloc(sizeof(TermVariableDescriptor));

    newVAR->variable = variable;
    newVAR->min_float = min;
    newVAR->max_float = max;
    newVAR->type = TERM_VARIABLE_FLOAT;
    newVAR->typeSize = sizeof(float);
    newVAR->name = name;
    newVAR->nameLength = strlen(name);
    newVAR->variableDescription = description;
    newVAR->rw = rw;

    TERM_VAR_LIST_add(newVAR, head);
    return newVAR;
}

TermVariableDescriptor * TERM_addVarString(void* variable, uint16_t typeSize, const char * name, const char * description, uint8_t rw, TermVariableDescriptor * head){
    //if(head == NULL) head = TERM_defaultList;
    if(head->nameLength == 0xff) return 0;

    TermVariableDescriptor * newVAR = pvPortMalloc(sizeof(TermVariableDescriptor));

    newVAR->variable = variable;
    newVAR->min_unsigned = 0;
    newVAR->max_unsigned = 0;
    newVAR->type = TERM_VARIABLE_STRING;
    newVAR->typeSize = typeSize;
    newVAR->name = name;
    newVAR->nameLength = strlen(name);
    newVAR->variableDescription = description;
    newVAR->rw = rw;

    TERM_VAR_LIST_add(newVAR, head);
    return newVAR;
}

TermVariableDescriptor * TERM_addVarChar(void* variable, const char * name, const char * description, uint8_t rw, TermVariableDescriptor * head){
    //if(head == NULL) head = TERM_defaultList;
    if(head->nameLength == 0xff) return 0;

    TermVariableDescriptor * newVAR = pvPortMalloc(sizeof(TermVariableDescriptor));

    newVAR->variable = variable;
    newVAR->min_unsigned = 0;
    newVAR->max_unsigned = 0;
    newVAR->type = TERM_VARIABLE_CHAR;
    newVAR->typeSize = 1;
    newVAR->name = name;
    newVAR->nameLength = strlen(name);
    newVAR->variableDescription = description;
    newVAR->rw = rw;

    TERM_VAR_LIST_add(newVAR, head);
    return newVAR;
}

TermVariableDescriptor * TERM_addVarBool(void* variable, const char * name, const char * description, uint8_t rw, TermVariableDescriptor * head){
    //if(head == NULL) head = TERM_defaultList;
    if(head->nameLength == 0xff) return 0;

    TermVariableDescriptor * newVAR = pvPortMalloc(sizeof(TermVariableDescriptor));

    newVAR->variable = variable;
    newVAR->min_unsigned = 0;
    newVAR->max_unsigned = 1;
    newVAR->type = TERM_VARIABLE_BOOL;
    newVAR->typeSize = sizeof(bool);
    newVAR->name = name;
    newVAR->nameLength = strlen(name);
    newVAR->variableDescription = description;
    newVAR->rw = rw;

    TERM_VAR_LIST_add(newVAR, head);
    return newVAR;
}


#define COL_A 3
#define COL_B 15
#define COL_C 35
#define COL_D 46
#define COL_E 57

static void print_var_header(TERMINAL_HANDLE * handle){
	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_A);
	ttprintf("Parameter");
	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
	ttprintf("| Value");
	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_C);
	ttprintf("| Min");
	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_D);
	ttprintf("| Max");
	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_E);
	ttprintf("| Description\r\n");
}

static void print_var_helperfunc(TERMINAL_HANDLE * handle, TermVariableDescriptor * var ){

    uint8_t current_parameter;
    uint32_t u_temp_buffer=0;
    int32_t i_temp_buffer=0;


	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_A);
	ttprintf("\033[36m%s", var->name);

	switch (var->type){
	case TERM_VARIABLE_UINT:
		switch (var->typeSize){
		case 1:
			u_temp_buffer = *(uint8_t*)var->variable;
			break;
		case 2:
			u_temp_buffer = *(uint16_t*)var->variable;
			break;
		case 4:
			u_temp_buffer = *(uint32_t*)var->variable;
			break;
		}

		TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
		ttprintf("\033[37m| \033[32m%u", u_temp_buffer);

		break;
	case TERM_VARIABLE_INT:
		switch (var->typeSize){
		case 1:
			i_temp_buffer = *(int8_t*)var->variable;
			break;
		case 2:
			i_temp_buffer = *(int16_t*)var->variable;
			break;
		case 4:
			i_temp_buffer = *(int32_t*)var->variable;
			break;
		}

		TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
		ttprintf("\033[37m| \033[32m%i", i_temp_buffer);

		break;
	case TERM_VARIABLE_FLOAT:

		TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
		ttprintf("\033[37m| \033[32m%f", *(float*)var->variable);

		break;
	case TERM_VARIABLE_CHAR:

		TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
		ttprintf("\033[37m| \033[32m%c", *(char*)var->variable);

		break;
	case TERM_VARIABLE_STRING:

		TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
		ttprintf("\033[37m| \033[32m%s", (char*)var->variable);

		break;
	case TERM_VARIABLE_BOOL:

		TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
		ttprintf("\033[37m| \033[32m%s", *(bool*)var->variable ? "true" : "false");

		break;

	}

	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_C);
	switch (var->type){
		case TERM_VARIABLE_UINT:
			ttprintf("\033[37m| %u", var->min_unsigned);
			break;
		case TERM_VARIABLE_INT:
			ttprintf("\033[37m| %i", var->min_signed);
			break;
		case TERM_VARIABLE_FLOAT:
			ttprintf("\033[37m| %.2f", var->min_float);
			break;
		default:
			ttprintf("\033[37m| -");
			break;
	}

	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_D);
	switch (var->type){
		case TERM_VARIABLE_UINT:
			ttprintf("\033[37m| %u", var->max_unsigned);
			break;
		case TERM_VARIABLE_INT:
			ttprintf("\033[37m| %i", var->max_signed);
			break;
		case TERM_VARIABLE_FLOAT:
			ttprintf("\033[37m| %.2f", var->max_float);
			break;
		default:
			ttprintf("\033[37m| -");
			break;
	}
	TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_E);
	ttprintf("\033[37m| %s\r\n", var->variableDescription);

}

uint8_t CMD_varList(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){

	uint8_t currPos = 0;

	print_var_header(handle);

	TermVariableDescriptor * currCmd = handle->varListHead->nextVar;
    for(;currPos < handle->varListHead->nameLength; currPos++){

    	print_var_helperfunc(handle, currCmd);
        currCmd = currCmd->nextVar;
    }

	return TERM_CMD_EXIT_SUCCESS;
}


