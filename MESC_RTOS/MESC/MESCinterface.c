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

#include "main.h"
#include "Tasks/init.h"
#include "TTerm/Core/include/TTerm.h"
#include "Tasks/task_cli.h"
#include "Tasks/task_overlay.h"
#include "MESCmotor_state.h"
#include "MESCmotor.h"
#include "MESCflash.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <MESC/MESCinterface.h>

extern uint16_t deadtime_comp;

uint8_t CMD_measure(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){

	MESC_motor_typedef * motor_curr = &mtr[0];
	port_str * port = handle->port;

	motor.measure_current = I_MEASURE;
	motor.measure_voltage = V_MEASURE;

	bool measure_res = false;
	bool measure_kv  = false;
	bool measure_hfi = false;
	bool measure_dt = false;

	if(argCount==0){
		measure_res =true;
		measure_kv  =true;
		measure_hfi =true;
	}

	for(int i=0;i<argCount;i++){
		if(strcmp(args[i], "-a")==0){
			measure_res =true;
			measure_kv  =true;
			measure_hfi =true;
		}
		if(strcmp(args[i], "-r")==0){
			measure_res = true;
		}
		if(strcmp(args[i], "-h")==0){
			measure_hfi = true;
		}
		if(strcmp(args[i], "-f")==0){
			measure_kv = true;
		}
		if(strcmp(args[i], "-d")==0){
			measure_dt = true;
		}
		if(strcmp(args[i], "-?")==0){
			ttprintf("Usage: measure [flags]\r\n");
			ttprintf("\t -a\t Measure all\r\n");
			ttprintf("\t -r\t Measure resistance and inductance\r\n");
			ttprintf("\t -f\t Measure flux linkage\r\n");
			ttprintf("\t -h\t Measure HFI threshold\r\n");
			ttprintf("\t -d\t Measure deadtime compensation\r\n");
			ttprintf("\t -c\t Specify openloop current\r\n");
			ttprintf("\t -v\t Specify HFI voltage\r\n");
			return TERM_CMD_EXIT_SUCCESS;
		}
		if(strcmp(args[i], "-v")==0){
			if(i+1 < argCount){
				motor.measure_voltage = strtof(args[i+1], NULL);
			}
		}
		if(strcmp(args[i], "-c")==0){
			if(i+1 < argCount){
				motor.measure_current = strtof(args[i+1], NULL);
			}
		}
	}

	if(measure_res){
		//Measure resistance and inductance
		motor_curr->MotorState = MOTOR_STATE_MEASURING;
		ttprintf("Measuring resistance and inductance\r\nWaiting for result");

		while(motor_curr->MotorState == MOTOR_STATE_MEASURING){
			xSemaphoreGive(port->term_block);
			vTaskDelay(200);
			xQueueSemaphoreTake(port->term_block, portMAX_DELAY);
			ttprintf(".");
		}

		TERM_sendVT100Code(handle,_VT100_ERASE_LINE, 0);
		TERM_sendVT100Code(handle,_VT100_CURSOR_SET_COLUMN, 0);

		float R, Lq, Ld;
		char* Runit;
		char* Lunit;
		if(motor_curr->m.R > 0){
			R = motor_curr->m.R;
			Runit = "Ohm";
		}else{
			R = motor_curr->m.R*1000.0f;
			Runit = "mOhm";
		}
		if(motor_curr->m.L_Q > 0.001f){
			Ld = motor_curr->m.L_D*1000.0f;
			Lq = motor_curr->m.L_Q*1000.0f;
			Lunit = "mH";
		}else{
			Ld = motor_curr->m.L_D*1000.0f*1000.0f;
			Lq = motor_curr->m.L_Q*1000.0f*1000.0f;
			Lunit = "uH";
		}


		ttprintf("R = %f %s\r\nLd = %f %s\r\nLq = %f %s\r\n\r\n", R, Runit, Ld, Lunit, Lq, Lunit);

		vTaskDelay(1000);
	}

	if(measure_kv){
		//Measure kV

		motor_curr->MotorState = MOTOR_STATE_GET_KV;
		ttprintf("Measuring flux linkage\r\nWaiting for result");

		while(motor_curr->MotorState == MOTOR_STATE_GET_KV){
			xSemaphoreGive(port->term_block);
			vTaskDelay(200);
			xQueueSemaphoreTake(port->term_block, portMAX_DELAY);
			ttprintf(".");
		}

		TERM_sendVT100Code(handle,_VT100_ERASE_LINE, 0);
		TERM_sendVT100Code(handle,_VT100_CURSOR_SET_COLUMN, 0);

		ttprintf("Flux linkage = %f mWb\r\n\r\n", motor_curr->m.flux_linkage * 1000.0);
		vTaskDelay(2000);
	}


	if(measure_hfi){
		ttprintf("Measuring HFI threshold\r\n");
		float HFI_Threshold = detectHFI(motor_curr);

		ttprintf("HFI threshold: %f\r\n", (double)HFI_Threshold);

		vTaskDelay(500);
	}

	if(measure_dt){
		ttprintf("Measuring deadtime compensation\r\nWaiting for result");

		while(motor_curr->MotorState == MOTOR_STATE_TEST){
			xSemaphoreGive(port->term_block);
			vTaskDelay(200);
			xQueueSemaphoreTake(port->term_block, portMAX_DELAY);
			ttprintf(".");
		}

		TERM_sendVT100Code(handle,_VT100_ERASE_LINE, 0);
		TERM_sendVT100Code(handle,_VT100_CURSOR_SET_COLUMN, 0);

		ttprintf("Deadtime register: %d\r\n", deadtime_comp);
		vTaskDelay(500);
	}


    return TERM_CMD_EXIT_SUCCESS;
}


extern TIM_HandleTypeDef htim1;


void callback(TermVariableDescriptor * var){
	calculateFlux(&mtr[0]);
	calculateGains(&mtr[0]);
	calculateVoltageGain(&mtr[0]);
	InputInit();
}

void populate_vars(){
	//		   | Variable							| MIN		| MAX		| NAME			| DESCRIPTION							| RW			| CALLBACK	| VAR LIST HANDLE
	TERM_addVar(mtr[0].m.Imax						, 0.0f		, 500.0f	, "i_max"		, "Max current"							, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].m.Pmax						, 0.0f		, 50000.0f	, "p_max"		, "Max power"							, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].m.direction					, 0			, 1			, "direction"	, "Motor direction"						, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].m.pole_pairs					, 0			, 255		, "pole_pairs"	, "Motor pole pairs"					, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].m.RPMmax						, 0			, 300000	, "rpm_max"		, "Max RPM"								, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].m.Vmax						, 0.0f		, 600.0f	, "v_max"		, "Max voltage"							, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].m.flux_linkage				, 0.0f		, 100.0f	, "flux"		, "Flux linkage"						, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].m.flux_linkage_gain			, 0.0f		, 100.0f	, "flux_gain"	, "Flux linkage gain"					, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].m.non_linear_centering_gain	, 0.0f		, 10000.0f	, "flux_n_lin"	, "Flux centering gain"					, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].m.R							, 0.0f		, 10.0f		, "r_phase"		, "Phase resistance"					, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].m.L_D						, 0.0f		, 10.0f		, "ld_phase"	, "Phase inductance"					, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].m.L_Q						, 0.0f		, 10.0f		, "lq_phase"	, "Phase inductance"					, VAR_ACCESS_RW	, callback  , &TERM_varList);
	TERM_addVar(mtr[0].HFIType						, 0			, 3			, "hfi"			, "HFI type [0=None, 1=45deg, 2=d axis]", VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].meas.hfi_voltage				, 0.0f		, 50.0f		, "hfi_volt"	, "HFI voltage"							, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].FOC.HFI45_mod_didq			, 0.0f		, 2.0f		, "hfi_mod_didq", "HFI mod didq"						, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].FOC.HFI_Gain					, 0.0f		, 5000.0f	, "hfi_gain"	, "HFI gain"							, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(input_vars.adc1_MAX					, 0			, 4096		, "adc1_max"	, "ADC1 max val"						, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(input_vars.adc1_MIN					, 0			, 4096		, "adc1_min"	, "ADC1 min val"						, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(input_vars.ADC1_polarity			, -1.0f		, 1.0f		, "adc1_pol"	, "ADC1 polarity"						, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(input_vars.adc2_MAX					, 0			, 4096		, "adc2_max"	, "ADC2 max val"						, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(input_vars.adc2_MIN					, 0			, 4096		, "adc2_min"	, "ADC2 min val"						, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(input_vars.ADC2_polarity			, -1.0f		, 1.0f		, "adc2_pol"	, "ADC2 polarity"						, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(input_vars.max_request_Idq.q		, 0.0f		, 300.0f	, "curr_max"	, "Max motor current"					, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(input_vars.min_request_Idq.q		, -300.0f	, 0.0f		, "curr_min"	, "Min motor current"					, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].FOC.pwm_frequency			, 0.0f		, 50000.0f	, "pwm_freq"	, "PWM frequency"						, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(input_vars.UART_req					, -1000.0f	, 1000.0f	, "UART_req"	, "Uart input"							, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].FOC.FW_curr_max				, 0.0f		, 200.0f	, "FW_curr"		, "Field Weakening Current"				, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(input_vars.input_options			, 0			, 16		, "input_opt"	, "Inputs [1=ADC1 2=ADC2 4=PPM 8=UART]"	, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(can1.node_id						, 0			, 65536		, "node_id"	    , "Node ID"							, VAR_ACCESS_RW	, callback	, &TERM_varList);

	TermVariableDescriptor * desc;
	desc = TERM_addVar(mtr[0].Conv.Vbus					, 0.0f		, HUGE_VAL  , "vbus"		, "Read input voltage"					, VAR_ACCESS_TR  , NULL		, &TERM_varList);
	TERM_setFlag(desc, FLAG_TELEMETRY_ON);

	desc = TERM_addVar(mtr[0].FOC.eHz					    , -HUGE_VAL , HUGE_VAL  , "ehz"			, "Motor electrical hz"					, VAR_ACCESS_TR  , NULL		, &TERM_varList);
	TERM_setFlag(desc, FLAG_TELEMETRY_ON);

}


void TASK_CAN_packet_cb(TASK_CAN_handle * handle, uint32_t id, uint8_t sender, uint8_t receiver, uint8_t* data, uint32_t len){
	switch(id){
		case CAN_ID_IQREQ:{
			float req = buffer_to_float(data);
			if(req > 0){
				input_vars.UART_req = req * input_vars.max_request_Idq.q;
			}else{
				input_vars.UART_req = req * input_vars.min_request_Idq.q * -1.0;
			}
			break;
		}
		case CAN_ID_CONNECT:{

			break;
		}
		default:
			break;
	}
}

void TASK_CAN_telemetry_fast(TASK_CAN_handle * handle){

	MESC_motor_typedef * motor_curr = &mtr[0];

	TASK_CAN_add_float(handle	, CAN_ID_ADC1_2_REQ	  	, CAN_BROADCAST, input_vars.ADC1_req		, input_vars.ADC2_req	, 0);
	TASK_CAN_add_float(handle	, CAN_ID_SPEED		  	, CAN_BROADCAST, motor_curr->FOC.eHz		, 0.0f					, 0);
	TASK_CAN_add_float(handle	, CAN_ID_BUS_VOLT_CURR 	, CAN_BROADCAST, motor_curr->Conv.Vbus		, motor_curr->FOC.Ibus	, 0);
	TASK_CAN_add_uint32(handle	, CAN_ID_STATUS	  		, CAN_BROADCAST, motor_curr->MotorState		, 0						, 0);
	TASK_CAN_add_float(handle	, CAN_ID_MOTOR_CURRENT 	, CAN_BROADCAST, motor_curr->FOC.Idq.q		, motor_curr->FOC.Idq.d	, 0);
	TASK_CAN_add_float(handle	, CAN_ID_MOTOR_VOLTAGE 	, CAN_BROADCAST, motor_curr->FOC.Vdq.q		, motor_curr->FOC.Vdq.d	, 0);

}

void TASK_CAN_telemetry_slow(TASK_CAN_handle * handle){

	MESC_motor_typedef * motor_curr = &mtr[0];

	TASK_CAN_add_float(handle	, CAN_ID_TEMP_MOT_MOS1	, CAN_BROADCAST, motor_curr->Conv.Motor_T			, motor_curr->Conv.MOSu_T			, 0);
	TASK_CAN_add_float(handle	, CAN_ID_TEMP_MOS2_MOS3	, CAN_BROADCAST, motor_curr->Conv.MOSv_T			, motor_curr->Conv.MOSw_T			, 0);
	TASK_CAN_add_uint32(handle	, CAN_ID_FOC_HYPER		, CAN_BROADCAST, motor_curr->FOC.cycles_fastloop	, motor_curr->FOC.cycles_hyperloop	, 0);

}



void MESCinterface_init(TERMINAL_HANDLE * handle){
	static bool is_init=false;
	if(is_init) return;


	populate_vars();

	if(CMD_varLoad(&null_handle, 0, NULL) == TERM_CMD_EXIT_ERROR){
		for(int i = 0; i<NUM_MOTORS; i++){
			mtr[i].conf_is_valid = false;
		}
	}

	calculateGains(&mtr[0]);
	calculateVoltageGain(&mtr[0]);
	InputInit();

	motor_profile->L_QD = motor_profile->L_Q-motor_profile->L_D;
	motor_profile->flux_linkage_max = 1.3f*motor_profile->flux_linkage;
	motor_profile->flux_linkage_min = 0.7f*motor_profile->flux_linkage;
	motor_profile->flux_linkage_gain = 10.0f * sqrtf(motor_profile->flux_linkage);

	mtr[0].m.flux_linkage_max = motor_profile->flux_linkage_max;
	mtr[0].m.flux_linkage_min = motor_profile->flux_linkage_min;
	mtr[0].m.flux_linkage_gain = motor_profile->flux_linkage_gain;

	TERM_addCommand(CMD_measure, "measure", "Measure motor R+L", 0, &TERM_defaultList);

	TERM_addCommand(CMD_status, "status", "Realtime data", 0, &TERM_defaultList);

	TERM_addCommand(CMD_nodes, "nodes", "Node info", 0, &TERM_defaultList);
	TERM_addCommand(CMD_can_send, "can_send", "Send CAN message", 0, &TERM_defaultList);

	TermCommandDescriptor * varAC = TERM_addCommand(CMD_log, "log", "Configure logging", 0, &TERM_defaultList);
	TERM_addCommandAC(varAC, TERM_varCompleter, null_handle.varHandle->varListHead);

	REGISTER_apps(&TERM_defaultList);

	is_init=true;
}
