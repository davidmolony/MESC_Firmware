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

#include "top.h"
#include "string.h"

#ifdef MESC
#include "MESCfoc.h"
#endif

#define APP_NAME "top"
#define APP_DESCRIPTION "shows performance stats"
#define APP_STACK 400

static uint8_t CMD_main(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
static void TASK_main(void *pvParameters);
static uint8_t INPUT_handler(TERMINAL_HANDLE * handle, uint16_t c);

static uint32_t SYS_getCPULoadFine(TaskStatus_t * taskStats, uint32_t taskCount, uint32_t sysTime);
static const char * SYS_getTaskStateString(eTaskState state);

uint8_t REGISTER_top(TermCommandDescriptor * desc){
    TERM_addCommand(CMD_main, APP_NAME, APP_DESCRIPTION, 0, desc); 
    return pdTRUE;
}

static uint8_t CMD_main(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    
    uint8_t currArg = 0;
    uint8_t returnCode = TERM_CMD_EXIT_SUCCESS;
    char ** cpy_args=NULL;
    argCount++;
    if(argCount){
        cpy_args = pvPortMalloc(sizeof(char*)*argCount);
        cpy_args[0] = pvPortMalloc(sizeof(APP_NAME));
        cpy_args[0]=memcpy(cpy_args[0], APP_NAME, sizeof(APP_NAME));
        for(;currArg<argCount-1; currArg++){
            uint16_t len = strlen(args[currArg])+1;
            cpy_args[currArg+1] = pvPortMalloc(len);
            memcpy(cpy_args[currArg+1], args[currArg], len);
        }
    }
    TermProgram * prog = pvPortMalloc(sizeof(TermProgram));
    prog->inputHandler = INPUT_handler;
    prog->args = cpy_args;
    prog->argCount = argCount;
    TERM_sendVT100Code(handle, _VT100_RESET, 0); TERM_sendVT100Code(handle, _VT100_CURSOR_POS1, 0);
    returnCode = xTaskCreate(TASK_main, APP_NAME, APP_STACK, handle, tskIDLE_PRIORITY + 1, &prog->task) ? TERM_CMD_EXIT_PROC_STARTED : TERM_CMD_EXIT_ERROR;
    if(returnCode == TERM_CMD_EXIT_PROC_STARTED) TERM_attachProgramm(handle, prog);
    return returnCode;
}

static void TASK_main(void *pvParameters){
    TERMINAL_HANDLE * handle = (TERMINAL_HANDLE*)pvParameters;

    char c=0;
    do{
        
        TaskStatus_t * taskStats;
        uint32_t taskCount = uxTaskGetNumberOfTasks();
        uint32_t sysTime;
                
        taskStats = pvPortMalloc( taskCount * sizeof( TaskStatus_t ) );
        if(taskStats){
            taskCount = uxTaskGetSystemState(taskStats, taskCount, &sysTime);
            
            TERM_sendVT100Code(handle, _VT100_CURSOR_POS1, 0);
        
            uint32_t cpuLoad = SYS_getCPULoadFine(taskStats, taskCount, sysTime);
            ttprintf("%sbottom - %d\r\n%sTasks: \t%d\r\n%sCPU: \t%d,%d%%\r\n", TERM_getVT100Code(_VT100_ERASE_LINE_END, 0), xTaskGetTickCount(), TERM_getVT100Code(_VT100_ERASE_LINE_END, 0), taskCount, TERM_getVT100Code(_VT100_ERASE_LINE_END, 0), cpuLoad / 10, cpuLoad % 10);
            
#ifdef MESC
            uint32_t sum_fastloop=0;
            uint32_t sum_pwmloop=0;
            uint32_t max_pwm=0;

            for(int i=0;i<NUM_MOTORS;i++){
            	sum_fastloop += mtr[i].FOC.cycles_fastloop;
            	sum_pwmloop += mtr[i].FOC.cycles_pwmloop;
            	if(max_pwm < mtr[i].FOC.pwm_frequency){
            		max_pwm = mtr[i].FOC.pwm_frequency;
            	}
            }

            uint32_t max_cycles= sum_fastloop > sum_pwmloop ? sum_fastloop : sum_pwmloop;
            uint32_t cycles_available = HAL_RCC_GetHCLKFreq() / max_pwm;

            uint32_t cycles_left = cycles_available - max_cycles;
            float foc_load = 100.0f / cycles_available * max_cycles;
            ttprintf("%sFOC load %3.0f%% - PWMloop: %5d Fastloop: %5d Cycles to overrun: %5d\r\n", TERM_getVT100Code(_VT100_ERASE_LINE_END, 0),foc_load , sum_pwmloop, sum_fastloop, cycles_left);
#endif
            uint32_t heapRemaining = xPortGetFreeHeapSize();
            ttprintf("%sMem: \t%db total,\t %db free,\t %db used (%d%%)\r\n", TERM_getVT100Code(_VT100_ERASE_LINE_END, 0), configTOTAL_HEAP_SIZE, heapRemaining, configTOTAL_HEAP_SIZE - heapRemaining, ((configTOTAL_HEAP_SIZE - heapRemaining) * 100) / configTOTAL_HEAP_SIZE);
            //taskStats[0].
            ttprintf("%s%s%s", TERM_getVT100Code(_VT100_BACKGROUND_COLOR, _VT100_WHITE), TERM_getVT100Code(_VT100_ERASE_LINE_END, 0), TERM_getVT100Code(_VT100_FOREGROUND_COLOR, _VT100_BLACK));
            ttprintf("PID \r\x1b[%dCName \r\x1b[%dCstate \r\x1b[%dC%%Cpu \r\x1b[%dCtime  \r\x1b[%dCStack \r\x1b[%dCHeap\r\n", 6, 7 + configMAX_TASK_NAME_LEN, 20 + configMAX_TASK_NAME_LEN, 27 + configMAX_TASK_NAME_LEN, 38 + configMAX_TASK_NAME_LEN, 45 + configMAX_TASK_NAME_LEN);
            ttprintf("%s", TERM_getVT100Code(_VT100_RESET_ATTRIB, 0));
            
            uint32_t currTask = 0;
            for(;currTask < taskCount; currTask++){
                if(strlen(taskStats[currTask].pcTaskName) != 4 || strcmp(taskStats[currTask].pcTaskName, "IDLE") != 0){
                    char name[configMAX_TASK_NAME_LEN+1];
                    strncpy(name, taskStats[currTask].pcTaskName, configMAX_TASK_NAME_LEN);
                     uint32_t load=0;
                    if(sysTime>1000){
                        load = (taskStats[currTask].ulRunTimeCounter) / (sysTime/configTICK_RATE_HZ);
                    }
                    ttprintf("%s%d\r\x1b[%dC%s\r\x1b[%dC%s\r\x1b[%dC%d,%d\r\x1b[%dC%d\r\x1b[%dC%u\r\x1b[%dC%d\r\n", TERM_getVT100Code(_VT100_ERASE_LINE_END, 0), taskStats[currTask].xTaskNumber, 6, name, 7 + configMAX_TASK_NAME_LEN
                            , SYS_getTaskStateString(taskStats[currTask].eCurrentState), 20 + configMAX_TASK_NAME_LEN, load / 10, load % 10, 27 + configMAX_TASK_NAME_LEN, taskStats[currTask].ulRunTimeCounter
                            , 38 + configMAX_TASK_NAME_LEN, taskStats[currTask].usStackHighWaterMark, 45 + configMAX_TASK_NAME_LEN, 0);
                }
            }
            vPortFree(taskStats);
        }else{
            ttprintf("Malloc failed\r\n");
        }
        
        xStreamBufferReceive(handle->currProgram->inputStream,&c,sizeof(c),pdMS_TO_TICKS(1000));
    }while(c!=CTRL_C);
    TERM_killProgramm(handle);
}

static uint8_t INPUT_handler(TERMINAL_HANDLE * handle, uint16_t c){
    if(handle->currProgram->inputStream==NULL) return TERM_CMD_EXIT_SUCCESS;
    switch(c){
        case 'q':
        case CTRL_C:
            c=CTRL_C;
            xStreamBufferSend(handle->currProgram->inputStream,&c,1,20);
            return TERM_CMD_EXIT_SUCCESS;
        default:
            return TERM_CMD_CONTINUE;
    }
}

static uint32_t SYS_getCPULoadFine(TaskStatus_t * taskStats, uint32_t taskCount, uint32_t sysTime){
    if(sysTime<500) return 0;
    uint32_t currTask = 0;
    for(;currTask < taskCount; currTask++){
        if(strlen(taskStats[currTask].pcTaskName) == 4 && strcmp(taskStats[currTask].pcTaskName, "IDLE") == 0){
            return configTICK_RATE_HZ - ((taskStats[currTask].ulRunTimeCounter) / (sysTime/configTICK_RATE_HZ));
        }
    }
    return -1;
}

static const char * SYS_getTaskStateString(eTaskState state){
    switch(state){
        case eRunning:
            return "running";
        case eReady:
            return "ready";
        case eBlocked:
            return "blocked";
        case eSuspended:
            return "suspended";
        case eDeleted:
            return "deleted";
        default:
            return "invalid";
    }
}
