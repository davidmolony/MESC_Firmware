/*
 **
 ******************************************************************************
 * @file           : MESCinterface.c
 * @brief          : Initializing RTOS system and parameters
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Jens Kerrinnes.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 ******************************************************************************
 *In addition to the usual 3 BSD clauses, it is explicitly noted that you
 *do NOT have the right to take sections of this code for other projects
 *without attribution and credit to the source. Specifically, if you copy into
 *copyleft licenced code without attribution and retention of the permissive BSD
 *3 clause licence, you grant a perpetual licence to do the same regarding turning sections of your code
 *permissive, and lose any rights to use of this code previously granted or assumed.
 *
 *This code is intended to remain permissively licensed wherever it goes,
 *maintaining the freedom to distribute compiled binaries WITHOUT a requirement to supply source.
 *
 *This is to ensure this code can at any point be used commercially, on products that may require
 *such restriction to meet regulatory requirements, or to avoid damage to hardware, or to ensure
 *warranties can reasonably be honoured.
 ******************************************************************************/

#include "MESCinterface.h"
#include "main.h"
#include "Tasks/init.h"
#include "TTerm/Core/include/TTerm.h"
#include "Tasks/task_cli.h"
#include "Tasks/task_can.h"
#include "Tasks/task_overlay.h"
#include "Dash/MESCmotor_state.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "TTerm/Core/include/TTerm_cwd.h"


#include "fatfs.h"
#include "tcp_serv.h"

#include "ssd1306.h"
#include "ssd1306_tests.h"

#include <stdio.h>


const char TERM_startupText1[] = "\r\n";
const char TERM_startupText2[] = "\r\n[D][A][S][H] - Dashboard";
const char TERM_startupText3[] = "\r\n";

dash_data dash;


#define N_COLS 8
#define N_ROWS 1000

#ifdef STM32F407xx
__attribute__((section (".ccmram")))
#endif
float sample_data[N_COLS][N_ROWS];

#ifdef STM32F407xx
__attribute__((section (".ccmram")))
#endif
uint32_t n_rows;

void * TASK_CAN_allocate_node(TASK_CAN_handle * handle, TASK_CAN_node * node){
	if(memcmp(node->short_name, "ESC", 3)==0){
		node->type = NODE_TYPE_ESC;
		node->data = pvPortMalloc(sizeof(esc_data));
		((esc_data*)node->data)->node = node;
	}
	return NULL;
}

void * TASK_CAN_free_node(TASK_CAN_handle * handle, TASK_CAN_node * node){
	vPortFree(node->data);
	return NULL;
}

char fl_file_prefix[64] = "fl";
char sl_file_prefix[64] = "sl";
bool enable_display = false;
volatile bool enable_slow_log = false;

void populate_vars(){
	can1.node_id = 255;

	//		   | Variable							| MIN		| MAX		| NAME			| DESCRIPTION							| RW			| CALLBACK	| VAR LIST HANDLE
	TERM_addVar(can1.node_id						, 1			, 254		, "node_id"		, "Node ID"								, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(dash.id_speed						, 1			, 254		, "id_speed"	, "Obtain speed from ID"				, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(dash.id_voltage						, 1			, 254		, "id_voltage"	, "Obtain bus voltage from ID"			, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(fl_file_prefix						, 0			, 0			, "fl_file"		, "Fastloop file template"				, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(sl_file_prefix						, 0			, 0			, "sl_file"		, "Slowloop file template"				, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(enable_display						, 0			, 1			, "ena_disp"	, "Enable OLED"							, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(enable_slow_log						, 0			, 1			, "ena_log"		, "Enable slow log"						, VAR_ACCESS_RW	, NULL		, &TERM_varList);

}

typedef struct{
	uint16_t rows;
	uint16_t start;
	esc_data * esc;
} save_arg;

volatile TaskHandle_t save_fastloop_handle;

void TASK_CAN_save_fastloop_data(void * argument){

	save_arg * arg = argument;

	TASK_CAN_node * node = arg->esc->node;

    FIL out;
    char timecode[90];
    snprintf(timecode, sizeof(timecode), "%s_%u_%u.csv", fl_file_prefix, node->id, (uint32_t)xTaskGetTickCount());


    FRESULT res = f_open(&out,timecode,FA_WRITE | FA_CREATE_ALWAYS);
    if(res != FR_OK){
    	goto CLEANUP;
    }


    char buffer[200];
    uint32_t len;
    unsigned int written=0;
    len = sprintf(buffer, "timestamp,Vbus,Iu,Iv,Iw,Vd,Vq,angle\r\n");
    f_write(&out, buffer, len, &written);
    for(uint8_t i=0;i<arg->rows;i++){

    	len = snprintf(buffer, sizeof(buffer), "%f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.0f\r\n", sample_data[0][i], sample_data[1][i], sample_data[2][i], sample_data[3][i],
																							sample_data[4][i], sample_data[5][i], sample_data[6][i], sample_data[7][i]);
		f_write(&out, buffer, len, &written);

    }

    f_close(&out);


    CLEANUP:
	vPortFree(arg);
	save_fastloop_handle = NULL;
    vTaskDelete(NULL);
    vTaskDelay(portMAX_DELAY);
}

extern volatile TERMINAL_HANDLE * debug;
uint32_t count=0;

void TASK_CAN_packet_esc(esc_data * esc, uint32_t id, uint8_t* data, uint32_t len){
	switch(id){
		case CAN_ID_ADC1_2_REQ:
			esc->adc1 = PACK_buf_to_float(data);
			esc->adc2 = PACK_buf_to_float(data+4);
			break;
		case CAN_ID_SPEED:
			esc->speed = PACK_buf_to_float(data);
			break;
		case CAN_ID_BUS_VOLT_CURR:
			esc->bus_voltage = PACK_buf_to_float(data);
			esc->bus_current = PACK_buf_to_float(data+4);
			break;
		case CAN_ID_TEMP_MOT_MOS1:
			esc->temp_motor = PACK_buf_to_float(data);
			esc->temp_mos1 = PACK_buf_to_float(data+4);
			break;
		case CAN_ID_TEMP_MOS2_MOS3:
			esc->temp_mos2 = PACK_buf_to_float(data);
			esc->temp_mos3 = PACK_buf_to_float(data+4);
			break;
		case CAN_ID_MOTOR_CURRENT:
			esc->Iq = PACK_buf_to_float(data);
			esc->Id = PACK_buf_to_float(data+4);
			break;
		case CAN_ID_MOTOR_VOLTAGE:
			esc->Vq = PACK_buf_to_float(data);
			esc->Vd = PACK_buf_to_float(data+4);
			break;
		case CAN_ID_STATUS:
			esc->status = PACK_buf_to_u32(data);
			break;
		case CAN_ID_FOC_HYPER:
			esc->cycles_fastloop = PACK_buf_to_u32(data);
			esc->cycles_hyperloop = PACK_buf_to_u32(data+4);
			break;
		case CAN_ID_SAMPLE:{

			uint16_t row = PACK_buf_to_u16(data);
			uint8_t col = PACK_buf_to_u8(data+2);
			uint8_t flags = PACK_buf_to_u8(data+3);
			float val = PACK_buf_to_float(data+4);

			if(flags==CAN_SAMPLE_FLAG_START){
				count=0;
			}else if(flags==CAN_SAMPLE_FLAG_END){
				if(strcmp(fl_file_prefix, "") != 0){ //Only save if a filename is set
					save_arg * arg = pvPortMalloc(sizeof(save_arg));
					arg->start = 0;
					arg->rows = n_rows;
					arg->esc = esc;
					if(arg){
						xTaskCreate(TASK_CAN_save_fastloop_data, "task_save", 512, arg, osPriorityNormal, &save_fastloop_handle);
					}
				}
				//Activate save function
				count=0;
			}else if(row < N_ROWS && col < N_COLS){
				sample_data[col][row] = val;
				count++;
				if(row > n_rows){
					n_rows = row + 1;
				}
			}
		}
	}

}



void TASK_CAN_packet_cb(TASK_CAN_handle * handle, uint32_t id, uint8_t sender, uint8_t receiver, uint8_t* data, uint32_t len){
	TASK_CAN_node * node = TASK_CAN_get_node_from_id(sender);

	if(node != NULL && node->type == NODE_TYPE_ESC && node->data != NULL){
		esc_data * esc = node->data;
		TASK_CAN_packet_esc(esc, id, data, len);  //Process ESC data
		return;
	}

	switch(id){

		default:
			break;
	}
}

uint8_t CMD_sample(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
	uint32_t id=0;

	for(int i=0;i<argCount;i++){
		if(strcmp(args[i], "-?")==0){
			ttprintf("Usage: sample [flags]\r\n");
			ttprintf("\t -i [id]\t Get a fastloop log from id\r\n");
			ttprintf("\t -a\t Get a fastloop log from all ESCs at once\r\n");
		}
		if(strcmp(args[i], "-i")==0){
			if(i+1 < argCount){
				id = strtoul(args[i+1], NULL, 0);
				TASK_CAN_add_uint32(&can1, CAN_ID_SAMPLE_NOW, id, 0, 0, 100);
				vTaskDelay(2);
				TASK_CAN_add_uint32(&can1, CAN_ID_SAMPLE_SEND, id, 0, 0, 100);
				ttprintf("Sent sample now to ID: %u\r\n", id);
			}
		}
		if(strcmp(args[i], "-a")==0){
			TASK_CAN_add_uint32(&can1, CAN_ID_SAMPLE_NOW, 0, 0, 0, 100); //Send sample now as broadcast to all ESCs
			for(uint32_t id=1;id<NUM_NODES;id++){
				TASK_CAN_node * node = TASK_CAN_get_node_from_id(id);
				if(node != NULL && node->type == NODE_TYPE_ESC && node->data != NULL){
					uint32_t timeout = 1000;
					while(save_fastloop_handle != NULL){
						timeout--;
						if(timeout==0){
							ttprintf("Cannot free save task\r\n");
							return TERM_CMD_EXIT_SUCCESS;
						}
						vTaskDelay(1);
					}
					TASK_CAN_add_uint32(&can1, CAN_ID_SAMPLE_SEND, id, 0, 0, 100);
					timeout = 2000;
					ttprintf("Saving fastloop data from ESC %u\r\n", id);
					while(save_fastloop_handle == NULL && timeout){
						timeout--;
						vTaskDelay(1);
					}
					if(timeout==0){
						ttprintf("Timeout on ESC %u\r\n", id);
						continue;
					}
				}
			}
		}
	}
	return TERM_CMD_EXIT_SUCCESS;
}


void TASK_SLOW_LOG(void * argument){

    char timecode[79];

    FRESULT res;
    uint32_t count=0;

    uint32_t filesize=0;

    FIL out;
    do{
    	count++;
    	snprintf(timecode, sizeof(timecode), "%s_%u.csv", sl_file_prefix, count);
    	res = f_open(&out,timecode,FA_READ);
    	f_close(&out);
    	vTaskDelay(5);
    }while(res ==FR_OK);



    res = f_open(&out,timecode,FA_WRITE | FA_CREATE_ALWAYS);
    if(res != FR_OK){
        goto CLEANUP; //Terminate the task
    }

    char buffer[512];
    uint32_t len;
    unsigned int written=0;
    len = sprintf(buffer, "id,ticks,speed,adc1,adc2,bus_voltage,bus_current,motor_current,temp_motor,temp_mos1,temp_mos2,temp_mos3,status,Iq,Id,Vq,Vd,cycles_fl,cycles_hl\r\n");
    filesize +=len ;
    f_write(&out, buffer, len, &written);
    f_close(&out);

    while(1){
   	 for(uint32_t id=1;id<NUM_NODES;id++){
			 TASK_CAN_node * node = TASK_CAN_get_node_from_id(id);
			 if(node != NULL && node->type == NODE_TYPE_ESC && node->data != NULL){
				esc_data * esc = node->data;
				len = sprintf(buffer, "%u,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%.3f,%.3f,%.3f,%.3f,%u,%u\r\n", id, xTaskGetTickCount() ,esc->speed, esc->adc1,
						esc->adc2, esc->bus_voltage, esc->bus_current, esc->motor_current,
						esc->temp_motor, esc->temp_mos1, esc->temp_mos2, esc->temp_mos3, esc->status, esc->Iq, esc->Id, esc->Vq, esc->Vd, esc->cycles_fastloop, esc->cycles_hyperloop);
				res = f_open(&out,timecode,FA_WRITE | FA_OPEN_APPEND);
				if(res == FR_OK){
					f_write(&out, buffer, len, &written);
					filesize += len;
					f_close(&out);
				}
			 }
   	 }
   	 vTaskDelay(100);
   	 if(enable_slow_log==false){
   		 goto CLEANUP;  //Terminate the task
   	 }

   	 if(filesize > 1024 * 1024 * 200){  //200 Megabyte
   		 count++;
   		 snprintf(timecode, sizeof(timecode), "%s_%u.csv", sl_file_prefix, count);
   		 res = f_open(&out,timecode,FA_WRITE | FA_CREATE_ALWAYS);
   		 if(res == FR_OK){
   			filesize=0;
   		    len = sprintf(buffer, "id,ticks,speed,adc1,adc2,bus_voltage,bus_current,motor_current,temp_motor,temp_mos1,temp_mos2,temp_mos3,status,Iq,Id,Vq,Vd,cycles_fl,cycles_hl\r\n");
   		    filesize +=len ;
   		    f_write(&out, buffer, len, &written);
   		    f_close(&out);
   		 }
   	 }



    }


    CLEANUP:
    vTaskDelete(NULL);
    vTaskDelay(portMAX_DELAY);
}


float adc[4];

void task_ssd(void * argument){
	ssd1306_Init();
	char buffer[64];

	TASK_CAN_node * node = NULL;
	esc_data * esc;
	float val;

	while(1){

		uint32_t y = 0;

	    ssd1306_Fill(Black);

	    ssd1306_SetCursor(2, y);

	    node = TASK_CAN_get_node_from_id(dash.id_speed);
	    if(node!=NULL && node->data!=NULL){
	    	esc = node->data;
	    	val = esc->speed;
	    }else{
	    	val = NAN;
	    }

	    snprintf(buffer,sizeof(buffer),"Speed: %2.2f", val);
	    ssd1306_WriteString(buffer, Font_11x18, White);
	    y += 19;

	    ssd1306_SetCursor(2, y);

	    node = TASK_CAN_get_node_from_id(dash.id_voltage);
	    if(node!=NULL && node->data!=NULL){
	    	esc = node->data;
	    	val = esc->motor_current;
	    }else{
	    	val = NAN;
	    }

	    snprintf(buffer,sizeof(buffer),"Curr : %2.2f", val);
	    ssd1306_WriteString(buffer, Font_11x18, White);
	    y += 19;

	    ssd1306_SetCursor(2, y);

	    node = TASK_CAN_get_node_from_id(dash.id_voltage);
	    if(node!=NULL && node->data!=NULL){
	    	esc = node->data;
	    	val = esc->bus_voltage;
	    }else{
	    	val = NAN;
	    }
	    snprintf(buffer,sizeof(buffer),"Volt : %2.2f", val);
		ssd1306_WriteString(buffer, Font_11x18, White);
		y += 19;

	    ssd1306_UpdateScreen();
	    vTaskDelay(20);

	}
}

extern ADC_HandleTypeDef hadc1;


void task_analog(void * argument){

	while(1){
		adc[0] = (float)ADC1->JDR1 / 4095.0f;
		adc[1] = (float)ADC1->JDR2 / 4095.0f;
		adc[2] = (float)ADC1->JDR3 / 4095.0f;
		adc[3] = (float)ADC1->JDR4 / 4095.0f;

		HAL_ADCEx_InjectedStart(&hadc1);
	    vTaskDelay(10);
	}
}

void TASK_CAN_telemetry_fast(TASK_CAN_handle * handle){
	TASK_CAN_add_float(handle	, CAN_ID_ADC1_2_REQ	  	, CAN_BROADCAST, adc[0]		, adc[1]	, 0);
}

void TASK_CAN_telemetry_slow(TASK_CAN_handle * handle){

}

uint8_t CMD_esc_info(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
	ttprintf("ESC detail info:\r\n");

	for(uint32_t i=0;i<NUM_NODES;i++){
		TASK_CAN_node * node = TASK_CAN_get_node_from_id(i);
		if(node!=NULL && node != NODE_OVERRUN){
			ttprintf("ID: %u\tType: %s\r\n", node->id, node->short_name);

			if(node->data && node->type == NODE_TYPE_ESC){
				esc_data * esc = node->data;
				ttprintf("\tSpeed: %f\r\n", esc->speed, esc->motor_current);
				ttprintf("\tIq: %f \tId: %f \tVq: %f Vd: %f\t\r\n", esc->Iq, esc->Id, esc->Vq, esc->Vd);
				ttprintf("\tADC1: %f \tADC2: %f\r\n", esc->adc1, esc->adc2);
				ttprintf("\tTemp Motor: %f \tTemp Mos1: %f \tTemp Mos2: %f \tTemp Mos3: %f\r\n", esc->temp_motor, esc->temp_mos1, esc->temp_mos2, esc->temp_mos3);
				ttprintf("\tBus voltage: %f \tBus current: %f\r\n", esc->bus_voltage, esc->bus_current);
				ttprintf("\tCycles fastloop: %u \tCycles hyperloop: %u\r\n", esc->cycles_fastloop, esc->cycles_hyperloop);

				ttprintf("\tMESC status: ");

				switch(esc->status){
				case MOTOR_STATE_INITIALISING:
					ttprintf("INITIALISING");
					break;
				case MOTOR_STATE_DETECTING:
					ttprintf("DETECTING");
					break;
				case MOTOR_STATE_MEASURING:
					ttprintf("MEASURING");
					break;
				case MOTOR_STATE_ALIGN:
					ttprintf("ALIGN");
					break;
				case MOTOR_STATE_OPEN_LOOP_STARTUP:
					ttprintf("OL STARTUP");
					break;
				case MOTOR_STATE_OPEN_LOOP_TRANSITION:
					ttprintf("OL TRANSITION");
					break;
				case MOTOR_STATE_TRACKING:
					ttprintf("TRACKING");
					break;
				case MOTOR_STATE_RUN:
					ttprintf("RUN");
					break;
				case MOTOR_STATE_GET_KV:
					ttprintf("GET KV");
					break;
				case MOTOR_STATE_TEST:
					ttprintf("TEST");
					break;
				case MOTOR_STATE_ERROR:
					ttprintf("ERROR");
					break;
				case MOTOR_STATE_RECOVERING:
					ttprintf("RECOVERING");
					break;
				case MOTOR_STATE_IDLE:
					ttprintf("IDLE");
					break;
				case MOTOR_STATE_SLAMBRAKE:
					ttprintf("SLAMBRAKE");
					break;
				}
				ttprintf("\r\n");

			}


		}
	}

	return TERM_CMD_EXIT_SUCCESS;

}


void MESCinterface_init(TERMINAL_HANDLE * handle){
	static bool is_init=false;
	if(is_init) return;
	is_init=true;

	n_rows = 0;

	populate_vars();

	if(CMD_varLoad(&null_handle, 0, NULL) == TERM_CMD_EXIT_ERROR){


	}

	TERM_addCommand(CMD_ifconfig, "ifconfig", "ifconfig", 0, &TERM_defaultList);

	TERM_addCommand(CMD_nodes, "nodes", "Node info", 0, &TERM_defaultList);
	TERM_addCommand(CMD_can_send, "can_send", "Send CAN message", 0, &TERM_defaultList);

	TERM_addCommand(CMD_esc_info, "esc", "ESC info", 0, &TERM_defaultList);

	TERM_addCommand(CMD_sample, "sample", "Sample now", 0, &TERM_defaultList);

	REGISTER_apps(&TERM_defaultList);


	tcp_serv_init();



	vTaskDelay(200);
	f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);



	vTaskDelay(100);
	if(enable_display){
		xTaskCreate(task_ssd, "tskSSD", 1024, NULL, osPriorityNormal, NULL);
	}
	xTaskCreate(task_analog, "tskAnalog", 128, NULL, osPriorityAboveNormal, NULL);

	if(enable_slow_log){
		xTaskCreate(TASK_SLOW_LOG, "tskSLOW", 1024, NULL, osPriorityNormal, NULL);
	}

}
