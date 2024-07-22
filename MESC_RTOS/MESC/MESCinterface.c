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
#include <stdint.h>
#include <math.h>
#include <MESC/MESCinterface.h>

extern uint16_t deadtime_comp;
void handleEscape(TERMINAL_HANDLE *handle){
	input_vars.UART_req = 0;
	input_vars.UART_dreq = 0;
}
uint8_t CMD_measure(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){

	MESC_motor_typedef * motor_curr = &mtr[0];
	port_str * port = handle->port;

//	motor.measure_current = I_MEASURE;
//	motor.measure_voltage = V_MEASURE;

	bool measure_RL = false;
	bool measure_res = false;
	bool measure_ind = false;
	bool measure_kv  = false;
	bool measure_linkage = false;
	bool measure_hfi = false;
	bool measure_dt = false;

	if(argCount==0){
		measure_RL = true;
		measure_linkage = true;
		//measure_hfi =true;
	}

	for(int i=0;i<argCount;i++){
		if(strcmp(args[i], "-a")==0){
			measure_RL = true;
			measure_kv = true;
			//measure_hfi =true;
		}
		if(strcmp(args[i], "-r")==0){
			measure_RL = true;
		}
		if(strcmp(args[i], "-s")==0){
			measure_res = true;
		}
		if(strcmp(args[i], "-l")==0){
			measure_ind = true;
		}
		if(strcmp(args[i], "-h")==0){
			measure_hfi = true;
		}
		if(strcmp(args[i], "-f")==0){
			measure_kv = true;
		}
		if(strcmp(args[i], "-g")==0){
			measure_linkage = true;
		}
		if(strcmp(args[i], "-d")==0){
			measure_dt = true;
		}
		if(strcmp(args[i], "-?")==0){
			ttprintf("Usage: measure [flags]\r\n");
			ttprintf("\t -a\t Measure all\r\n");
			ttprintf("\t -r\t Measure resistance and inductance\r\n");
			ttprintf("\t -f\t Measure flux linkage\r\n");
			ttprintf("\t -g\t Measure flux linkage threshold v2\r\n");
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

	if(measure_RL){
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


		ttprintf("R = %f %s\r\nLd = %f %s\r\nLq = %f %s\r\n\r\n", (double)R, Runit, (double)Ld, Lunit, (double)Lq, Lunit);
		calculateGains(motor_curr);
		vTaskDelay(1000);
	}

	if(measure_res){
		//Measure resistance
		float old_L_D = motor_curr->m.L_D;
		motor_curr->m.R = 0.0001f;//0.1mohm, really low
		motor_curr->m.L_D = 0.000001f;//1uH, really low
		calculateGains(motor_curr);
		calculateVoltageGain(motor_curr);

		motor_curr->MotorState = MOTOR_STATE_RUN;
		motor_curr->MotorSensorMode = MOTOR_SENSOR_MODE_OPENLOOP;

		motor_curr->FOC.openloop_step = 0;

		ttprintf("Measuring resistance \r\nWaiting for result");
		int a=200;
		float Itop = 0.0f;
		float Ibot = 0.0f;
		float Vtop = 0.0f;
		float Vbot = 0.0f;
		input_vars.UART_req = 0.45f*motor_curr->m.Imax;

		while(a){
			xSemaphoreGive(port->term_block);
			vTaskDelay(5);
			xQueueSemaphoreTake(port->term_block, portMAX_DELAY);
			ttprintf(".");
			Ibot = Ibot+motor_curr->FOC.Idq.q;
			Vbot = Vbot+motor_curr->FOC.Vdq.q;
			a--;
			motor_curr->FOC.FOCAngle +=300;
		}
		a=200;
		input_vars.UART_req = 0.55f*motor_curr->m.Imax;
		while(a){
			xSemaphoreGive(port->term_block);
			vTaskDelay(5);
			xQueueSemaphoreTake(port->term_block, portMAX_DELAY);
			ttprintf(".");
			Itop = Itop+motor_curr->FOC.Idq.q;
			Vtop = Vtop+motor_curr->FOC.Vdq.q;
			a--;
			motor_curr->FOC.FOCAngle +=300;
		}

		motor_curr->m.R = (Vtop-Vbot)/((Itop-Ibot)); //Calculate the resistance

		input_vars.UART_req = 0.0f;
		motor_curr->MotorState = MOTOR_STATE_TRACKING;
		motor_curr->MotorSensorMode = MOTOR_SENSOR_MODE_SENSORLESS;

		TERM_sendVT100Code(handle,_VT100_ERASE_LINE, 0);
		TERM_sendVT100Code(handle,_VT100_CURSOR_SET_COLUMN, 0);

		float R;
		char* Runit;
		if(motor_curr->m.R > 0){
			R = motor_curr->m.R;
			Runit = "Ohm";
		}else{
			R = motor_curr->m.R*1000.0f;
			Runit = "mOhm";
		}


		ttprintf("R = %f %s\r\n\r\n", (double)R, Runit);
		motor_curr->m.L_D =old_L_D;
		calculateGains(motor_curr);
		vTaskDelay(1000);
	}

	if(measure_ind){
		//Measure inductance, second method

		ttprintf("Measuring Inductance\r\nWaiting for result");
		int a=200;
		float Loffset[3];
		float Lqoffset[3];


		//set things up to do the L measurement
		motor_curr->MotorState = MOTOR_STATE_RUN;
		input_vars.UART_req = 0.25f; //Stop it going into tracking mode
		motor_curr->HFIType = HFI_TYPE_SPECIAL;
		motor_curr->FOC.special_injectionVd = 0.2f;
		motor_curr->FOC.special_injectionVq = 0.0f;
		motor_curr->MotorSensorMode = MOTOR_SENSOR_MODE_OPENLOOP;
		motor_curr->FOC.openloop_step = 0;
		motor_curr->FOC.FOCAngle = 0;
		input_vars.UART_dreq = -5.0f;
		motor_curr->FOC.didq.d = 0.0f;
		vTaskDelay(10);

		//Determine the voltage required
		while(a){
			if(fabsf(motor_curr->FOC.didq.d)<5.0f){
				motor_curr->FOC.special_injectionVd *=1.05f;
				if(motor_curr->FOC.special_injectionVd > (0.5f * motor_curr->Conv.Vbus))
				{
					motor_curr->FOC.special_injectionVd = 0.5f * motor_curr->Conv.Vbus;
				}
			}
			xSemaphoreGive(port->term_block);
			vTaskDelay(5);
			xQueueSemaphoreTake(port->term_block, portMAX_DELAY);
			ttprintf(".");
			a--;
		}
		int b;
		//Measure the Ld
		for(b=0;b<3; b++){
			Loffset[b] = 0.0f;
			a=200;
			input_vars.UART_dreq = -motor_curr->m.Imax * 0.25f * (float)b;
			while(a){
				Loffset[b] = Loffset[b] + motor_curr->FOC.didq.d;
				xSemaphoreGive(port->term_block);
				vTaskDelay(5);
				xQueueSemaphoreTake(port->term_block, portMAX_DELAY);
				ttprintf(".");
				a--;
			}
		Loffset[b] = Loffset[b]/200;
		Loffset[b] = motor_curr->FOC.pwm_period * motor_curr->FOC.special_injectionVd/Loffset[b];
		}

		TERM_sendVT100Code(handle,_VT100_ERASE_LINE, 0);
		TERM_sendVT100Code(handle,_VT100_CURSOR_SET_COLUMN, 0);
		ttprintf("D-Inductance = %f , %f , %f  H\r\n voltage was %f \r\n", (double)Loffset[0], (double)Loffset[1], (double)Loffset[2], (double)motor_curr->FOC.special_injectionVd);

		//Now do Lq
		motor_curr->FOC.special_injectionVq = motor_curr->FOC.special_injectionVd;
		float c = motor_curr->FOC.special_injectionVq;
		motor_curr->FOC.special_injectionVd = 0.0f;
		for(b=0;b<3; b++){
			Lqoffset[b] = 0.0f;
			a=200;
			input_vars.UART_dreq = -10.0f ;
			motor_curr->FOC.special_injectionVq = c * (1+(float)b);
			if(motor_curr->FOC.special_injectionVq > motor_curr->Conv.Vbus*0.5f){motor_curr->FOC.special_injectionVq = motor_curr->Conv.Vbus*0.5f;}
			while(a){
				Lqoffset[b] = Lqoffset[b] + motor_curr->FOC.didq.q;
				xSemaphoreGive(port->term_block);
				vTaskDelay(5);
				xQueueSemaphoreTake(port->term_block, portMAX_DELAY);
				ttprintf(".");
				a--;
			}
		Lqoffset[b] = Lqoffset[b]/200;
		Lqoffset[b] = motor_curr->FOC.pwm_period * motor_curr->FOC.special_injectionVq/Lqoffset[b];
		}

		//Put things back to runable
		motor_curr->HFIType = HFI_TYPE_NONE;
		motor_curr->MotorSensorMode = MOTOR_SENSOR_MODE_SENSORLESS;
		input_vars.UART_req = 0.0f; //Turn it off.
		input_vars.UART_dreq = 0.0f;
		motor_curr->MotorState = MOTOR_STATE_TRACKING;


		TERM_sendVT100Code(handle,_VT100_ERASE_LINE, 0);
		TERM_sendVT100Code(handle,_VT100_CURSOR_SET_COLUMN, 0);
		ttprintf("Q-Inductance = %f , %f , %f  H\r\n voltage was %f \r\n", (double)Lqoffset[0], (double)Lqoffset[1], (double)Lqoffset[2], (double)motor_curr->FOC.special_injectionVq);

		motor_curr->FOC.special_injectionVd = 0.0f;
		motor_curr->FOC.special_injectionVq = 0.0f;

		vTaskDelay(200);
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

		ttprintf("Flux linkage = %f mWb\r\n\r\n", (double)(motor_curr->m.flux_linkage * 1000.0f));
		vTaskDelay(2000);
	}

	if(measure_linkage){
		//Measure kV
		motor_curr->MotorState = MOTOR_STATE_RUN;
		ttprintf("Measuring flux linkage\r\nWaiting for result");
		motor_curr->m.flux_linkage_max = 0.1001f;//Start it low
		motor_curr->m.flux_linkage_min = 0.00005f;//Start it low

		motor_curr->HFIType = HFI_TYPE_NONE;
		motor_curr->FOC.FW_curr_max = 0.1f;
		input_vars.UART_req = 10.0f; //Parametise later, closedloop current

		while((motor_curr->m.flux_linkage_max > 0.0001f) && (motor_curr->FOC.eHz<100)){
			xSemaphoreGive(port->term_block);
			vTaskDelay(10);
			xQueueSemaphoreTake(port->term_block, portMAX_DELAY);
			ttprintf(".");
			motor_curr->m.flux_linkage_max = motor_curr->m.flux_linkage_max*0.997f;// + 0.0001f;
			motor_curr->FOC.flux_a = motor_curr->FOC.flux_a	+ 0.01*motor_curr->FOC.flux_b;
			motor_curr->FOC.flux_b = motor_curr->FOC.flux_b	- 0.01*motor_curr->FOC.flux_a;//Since the two fluxes are derivatives of each other, this kicks them around and avoids stalls
			if(motor_curr->MotorState == MOTOR_STATE_ERROR){
				break;
			}
		}
		int a=200;
		motor_curr->m.flux_linkage_max = motor_curr->m.flux_linkage_max*1.5f;
		while(a){
			xSemaphoreGive(port->term_block);
			vTaskDelay(10);
			xQueueSemaphoreTake(port->term_block, portMAX_DELAY);
			ttprintf(".");
			a--;
		}
		motor_curr->m.flux_linkage_max = motor_curr->FOC.flux_observed*1.5f;
		motor_curr->m.flux_linkage_min = motor_curr->FOC.flux_observed*0.5f;
		motor_curr->m.flux_linkage = motor_curr->FOC.flux_observed;

		TERM_sendVT100Code(handle,_VT100_ERASE_LINE, 0);
		TERM_sendVT100Code(handle,_VT100_CURSOR_SET_COLUMN, 0);

		ttprintf("Flux linkage = %f mWb\r\n\r\n", (double)(motor_curr->m.flux_linkage * 1000.0f));
		ttprintf("Did the motor spin for >2seconds?");

		vTaskDelay(200);

		input_vars.UART_req = 0.0f;
		motor_curr->MotorState = MOTOR_STATE_TRACKING;
	}

	if(measure_hfi){
		ttprintf("Measuring HFI threshold\r\n");
		float HFI_Threshold = detectHFI(motor_curr);

		ttprintf("HFI threshold: %f\r\n", (double)HFI_Threshold);

		vTaskDelay(500);
	}

	if(measure_dt){
		ttprintf("Measuring deadtime compensation\r\nWaiting for result");
		motor_curr->MotorState = MOTOR_STATE_TEST;
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
	TERM_addVar(mtr[0].HFIType						, 0			, 3			, "hfi_type"	, "HFI type [0=None, 1=45deg, 2=d axis]", VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].meas.hfi_voltage				, 0.0f		, 50.0f		, "hfi_volt"	, "HFI voltage"							, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].FOC.HFI45_mod_didq			, 0.0f		, 2.0f		, "hfi_mod_didq", "HFI mod didq"						, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].FOC.HFI_Gain					, 0.0f		, 5000.0f	, "hfi_gain"	, "HFI gain"							, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].FOC.FW_curr_max				, 0.0f		, 300.0f	, "fw_curr"		, "Max field weakenning current"		, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].meas.measure_current			, 0.5f		, 100.0f	, "meas_curr"	, "Measuring current"					, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].meas.measure_closedloop_current, 0.5f	, 100.0f	, "meas_cl_curr"		, "Measuring q closed loop current"		, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].meas.measure_voltage			, 0.5f		, 100.0f	, "meas_volt"	, "Measuring voltage"					, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(input_vars.adc1_MAX					, 0			, 4096		, "adc1_max"	, "ADC1 max val"						, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(input_vars.adc1_MIN					, 0			, 4096		, "adc1_min"	, "ADC1 min val"						, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(input_vars.ADC1_polarity			, -1.0f		, 1.0f		, "adc1_pol"	, "ADC1 polarity"						, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(input_vars.adc2_MAX					, 0			, 4096		, "adc2_max"	, "ADC2 max val"						, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(input_vars.adc2_MIN					, 0			, 4096		, "adc2_min"	, "ADC2 min val"						, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(input_vars.ADC2_polarity			, -1.0f		, 1.0f		, "adc2_pol"	, "ADC2 polarity"						, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(input_vars.max_request_Idq.q		, 0.0f		, 300.0f	, "curr_max"	, "Max motor current"					, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(input_vars.min_request_Idq.q		, -300.0f	, 0.0f		, "curr_min"	, "Min motor current"					, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].FOC.pwm_frequency			, 0.0f		, 100000.0f	, "pwm_freq"	, "PWM frequency"						, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(input_vars.UART_req					, -1000.0f	, 1000.0f	, "uart_req"	, "Uart input"							, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(input_vars.UART_dreq				, -1000.0f	, 1000.0f	, "uart_dreq"	, "Uart input"							, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(input_vars.input_options			, 0			, 128		, "input_opt"	, "Inputs [1=ADC1 2=ADC2 4=PPM 8=UART 16=Killswitch 32=CANADC1 64=CANADC2 128=ADC12DIFF]"	, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].safe_start[0]				, 0			, 1000		, "safe_start"	, "Countdown before allowing throttle"	, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].safe_start[1]				, 0			, 1000		, "safe_count"	, "Live count before allowing throttle"	, VAR_ACCESS_R	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].FOC.enc_offset				, 0			, 65535		, "enc_offset"	, "Encoder alignment angle"				, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].FOC.FOCAngle					, 0			, 65535		, "FOC_angle"	, "FOC angle now"						, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].FOC.enc_angle				, 0			, 65535		, "enc_angle"	, "Encoder angle now"					, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].m.enc_counts					, 0			, 65535		, "enc_counts"	, "Encoder ABI PPR"						, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].FOC.encoder_polarity_invert	, 0			, 1			, "enc_polarity", "Encoder polarity"					, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].m.pole_pairs					, 0			, 30		, "motor_pp"	, "Number of motor pole PAIRS"			, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].MotorSensorMode				, 0			, 30		, "motor_sensor", "0=SL, 1=Hall, 2=OL, 3=ABSENC, 4=INC_ENC, 5=HFI"	, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].SLStartupSensor				, 0			, 30		, "SL_sensor"	, "0=OL, 1=Hall, 2=PWMENC, 3=HFI"		, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].FOC.openloop_step			, 0.0f		, 6000.0f	, "ol_step"		, "Angle per PWM period openloop"		, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].FOC.FW_ehz_max				, 0.0f		, 6000.0f	, "fw_ehz"		, "max eHz under field weakenning"		, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(mtr[0].FOC.park_current				, 0.0f		, 300.0f	, "park_curr"	, "max current for handbrake"			, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(MESC_all_errors						, -HUGE_VAL	, HUGE_VAL	, "error_all"	, "All errors encountered"				, VAR_ACCESS_R	, NULL		, &TERM_varList);
	TERM_addVar(mtr[0].FOC.hall_initialised			, 0		, 1	, "Hall_initialised"		, "hall start flag"						, VAR_ACCESS_RW	, NULL		, &TERM_varList);
	TERM_addVarArrayFloat(mtr[0].m.hall_flux, sizeof(mtr[0].m.hall_flux),  -10.0f, 10.0f, "Hall_flux", "hall start table", VAR_ACCESS_RW, NULL, &TERM_varList);


	#ifdef HAL_CAN_MODULE_ENABLED
	TERM_addVar(can1.node_id						, 1			, 254		, "node_id"	    , "Node ID"								, VAR_ACCESS_RW	, callback	, &TERM_varList);
	TERM_addVar(input_vars.remote_ADC_can_id		, 0			, 254		, "can_adc"	    , "CAN ADC ID  0=disabled"				, VAR_ACCESS_RW	, callback	, &TERM_varList);
#endif

	TermVariableDescriptor * desc;
		desc = TERM_addVar(mtr[0].Conv.Vbus         , 0.0f      , HUGE_VAL  , "vbus"        , "Read input voltage"                  , VAR_ACCESS_TR  , NULL         , &TERM_varList);
		TERM_setFlag(desc, FLAG_TELEMETRY_ON);

		desc = TERM_addVar(mtr[0].FOC.eHz           , -HUGE_VAL , HUGE_VAL  , "ehz"         , "Motor electrical hz"                 , VAR_ACCESS_TR  , NULL         , &TERM_varList);
		TERM_setFlag(desc, FLAG_TELEMETRY_ON);

		desc = TERM_addVar(mtr[0].FOC.Idq_smoothed.d , -HUGE_VAL , HUGE_VAL  , "id"      , "Phase Idq_d smoothed"                   , VAR_ACCESS_TR  , NULL         , &TERM_varList);
		TERM_setFlag(desc, FLAG_TELEMETRY_ON);

		desc = TERM_addVar(mtr[0].FOC.Idq_smoothed.q , -HUGE_VAL , HUGE_VAL  , "iq"      , "Phase Idq_q smoothed"                   , VAR_ACCESS_TR  , NULL         , &TERM_varList);
		TERM_setFlag(desc, FLAG_TELEMETRY_ON);

		desc = TERM_addVar(mtr[0].Raw.ADC_in_ext1    , 0		, 4096       , "adc1"   , "Raw ADC throttle"                    	, VAR_ACCESS_TR  , NULL         , &TERM_varList);
		TERM_setFlag(desc, FLAG_TELEMETRY_ON);

		desc = TERM_addVar(mtr[0].Conv.MOSu_T        , 0.0f		, 4096.0f    , "TMOS"   , "MOSFET temp, kelvin"                     , VAR_ACCESS_TR  , NULL         , &TERM_varList);
		TERM_setFlag(desc, FLAG_TELEMETRY_ON);

		desc = TERM_addVar(mtr[0].Conv.Motor_T       , 0.0f 	, 4096.0f    , "TMOT"   , "Motor temp, kelvin"                      , VAR_ACCESS_TR  , NULL         , &TERM_varList);
		TERM_setFlag(desc, FLAG_TELEMETRY_ON);

		desc = TERM_addVar(MESC_errors          	, -HUGE_VAL , HUGE_VAL  , "error" 	, "System errors now"       					, VAR_ACCESS_TR  , NULL			, &TERM_varList);
		TERM_setFlag(desc, FLAG_TELEMETRY_ON);

		desc = TERM_addVar(mtr[0].FOC.Vdq.q     	, -4096.0f , 4096.0f  	, "Vq"    	, "FOC_Vdq_q"     							, VAR_ACCESS_TR  , NULL         , &TERM_varList);
		TERM_setFlag(desc, FLAG_TELEMETRY_ON);

		desc = TERM_addVar(mtr[0].FOC.Vdq.d     	, -4096.0f , 4096.0f  	, "Vd"    , "FOC_Vdq_d"     							, VAR_ACCESS_TR  , NULL         , &TERM_varList);
		TERM_setFlag(desc, FLAG_TELEMETRY_ON);

		desc = TERM_addVar(mtr[0].FOC.Idq_req.q 	, -4096.0f , 4096.0f  	, "iqreq" , "mtr[0].FOC.Idq_req.q"     					, VAR_ACCESS_TR  , NULL         , &TERM_varList);
		TERM_setFlag(desc, FLAG_TELEMETRY_ON);

}

#ifdef HAL_CAN_MODULE_ENABLED

#define REMOTE_ADC_TIMEOUT 1000

void TASK_CAN_packet_cb(TASK_CAN_handle * handle, uint32_t id, uint8_t sender, uint8_t receiver, uint8_t* data, uint32_t len){
	MESC_motor_typedef * motor_curr = &mtr[0];

	switch(id){
		case CAN_ID_IQREQ:{
			float req = PACK_buf_to_float(data);
			if(req > 0){
				input_vars.UART_req = req * input_vars.max_request_Idq.q;
			}else{
				input_vars.UART_req = req * input_vars.min_request_Idq.q * -1.0;
			}
			break;
		}
		case CAN_ID_SAMPLE_NOW:
			motor_curr->sample_no_auto_send = true;
			motor_curr->sample_now = true;
			break;
		case CAN_ID_SAMPLE_SEND:
			motor_curr->sample_no_auto_send = false;
			break;
		case CAN_ID_ADC1_2_REQ:{
			if(sender == input_vars.remote_ADC_can_id && input_vars.remote_ADC_can_id > 0){
				input_vars.remote_ADC_timeout = REMOTE_ADC_TIMEOUT;
				input_vars.remote_ADC1_req = PACK_buf_to_float(data);
				input_vars.remote_ADC2_req = PACK_buf_to_float(data+4);
			}
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
	TASK_CAN_add_uint32(handle	, CAN_ID_FOC_HYPER		, CAN_BROADCAST, motor_curr->FOC.cycles_fastloop	, motor_curr->FOC.cycles_pwmloop	, 0);

}


#define POST_ERROR_SAMPLES 		LOGLENGTH/2

void TASK_CAN_aux_data(TASK_CAN_handle * handle){
	static int samples_sent=-1;
	static int current_pos=0;
	static float timestamp;

	MESC_motor_typedef * motor_curr = &mtr[0];

	if(print_samples_now && motor_curr->sample_no_auto_send == false){
		if(samples_sent == -1){
			current_pos = sampled_vars.current_sample;
			TASK_CAN_add_sample(handle, CAN_ID_SAMPLE, 0, 0, 0, CAN_SAMPLE_FLAG_START, (float)LOGLENGTH, 100);
			samples_sent=0;
			timestamp = motor_curr->FOC.pwm_period * (float)POST_ERROR_SAMPLES * -1.0f;
			return;
		}

		timestamp += motor_curr->FOC.pwm_period;
		TASK_CAN_add_sample(handle, CAN_ID_SAMPLE, 0, samples_sent, 0, 0, timestamp, 100);
		TASK_CAN_add_sample(handle, CAN_ID_SAMPLE, 0, samples_sent, 1, 0, sampled_vars.Vbus[current_pos], 100);
		TASK_CAN_add_sample(handle, CAN_ID_SAMPLE, 0, samples_sent, 2, 0, sampled_vars.Iu[current_pos], 100);
		TASK_CAN_add_sample(handle, CAN_ID_SAMPLE, 0, samples_sent, 3, 0, sampled_vars.Iv[current_pos], 100);
		TASK_CAN_add_sample(handle, CAN_ID_SAMPLE, 0, samples_sent, 4, 0, sampled_vars.Iw[current_pos], 100);
		TASK_CAN_add_sample(handle, CAN_ID_SAMPLE, 0, samples_sent, 5, 0, sampled_vars.Vd[current_pos], 100);
		TASK_CAN_add_sample(handle, CAN_ID_SAMPLE, 0, samples_sent, 6, 0, sampled_vars.Vq[current_pos], 100);
		TASK_CAN_add_sample(handle, CAN_ID_SAMPLE, 0, samples_sent, 7, 0, sampled_vars.angle[current_pos], 100);

		samples_sent++;
		current_pos++;
		if(current_pos == LOGLENGTH){
			current_pos = 0;
		}
		if(samples_sent == LOGLENGTH){
			timestamp = 0;
			samples_sent = -2;
			print_samples_now = 0;
			lognow = 1;
			return;
		}
	}
	if(samples_sent == -2){
		samples_sent = -1;
		TASK_CAN_add_sample(handle, CAN_ID_SAMPLE, 0, 0, 0, CAN_SAMPLE_FLAG_END, 0.0f, 100);
	}

}
#endif


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
	calculateFlux(&mtr[0]);
	InputInit();

	TERM_addCommand(CMD_measure, "measure", "Measure motor R+L", 0, &TERM_defaultList);

	TERM_addCommand(CMD_status, "status", "Realtime data", 0, &TERM_defaultList);

#ifdef HAL_CAN_MODULE_ENABLED
	TERM_addCommand(CMD_nodes, "nodes", "Node info", 0, &TERM_defaultList);
	TERM_addCommand(CMD_can_send, "can_send", "Send CAN message", 0, &TERM_defaultList);
#endif

	TermCommandDescriptor * varAC = TERM_addCommand(CMD_log, "log", "Configure logging", 0, &TERM_defaultList);
	TERM_addCommandAC(varAC, TERM_varCompleter, null_handle.varHandle->varListHead);

	REGISTER_apps(&TERM_defaultList);

	is_init=true;
}
