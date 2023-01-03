

#include "main.h"
#include "TTerm/Core/include/TTerm.h"
#include "task_cli.h"
#include "task_overlay.h"
#include "MESCmotor_state.h"
#include "MESCmotor.h"
#include "MESCflash.h"
//#include "MESCcli.h"

#include <stdlib.h>
#include <string.h>

uint8_t CMD_measure(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){

	MESC_motor_typedef * motor_curr = &mtr[0];

	motor.measure_current = I_MEASURE;
	motor.measure_voltage = V_MEASURE;

	if(argCount==1){
		if(strcmp(args[0], "-?")==0){
			ttprintf("Usage: measure   - optional: [measure current] [optional: measure voltage]\r\n");
			return TERM_CMD_EXIT_SUCCESS;
		}
	}

	if(argCount==2){
		motor.measure_current = atoff(args[0]);
		motor.measure_voltage = atoff(args[1]);
	}

	motor_curr->MotorState = MOTOR_STATE_MEASURING;
    ttprintf("Waiting for result");

    port_str * port = handle->port;
    while(motor_curr->MotorState == MOTOR_STATE_MEASURING){
    	xSemaphoreGive(port->term_block);
    	vTaskDelay(200);
    	xQueueSemaphoreTake(port->term_block, portMAX_DELAY);
    	ttprintf(".");
    }

    ttprintf("\r\nResult:\r\n");

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

  //  motor_profile->R = motor.Rphase;
  //  motor_profile->L_D = motor.Lphase;
  //  motor_profile->L_Q = motor.Lqphase;

    ttprintf("R = %f %s\r\nLd = %f %s\r\nLq = %f %s\r\n", R, Runit, Ld, Lunit, Lq, Lunit);

    return TERM_CMD_EXIT_SUCCESS;
}

uint8_t CMD_getkv(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){

	MESC_motor_typedef * motor_curr = &mtr[0];

	motor.measure_current = I_MEASURE;
	motor.measure_voltage = V_MEASURE;

	if(argCount==1){
		if(strcmp(args[0], "-?")==0){
			ttprintf("Usage: getkv [optional: measure current]\r\n");
			return TERM_CMD_EXIT_SUCCESS;
		}

		motor.measure_current = atoff(args[0]);

	}

	motor_curr->MotorState = MOTOR_STATE_GET_KV;
    ttprintf("Waiting for result");

    port_str * port = handle->port;
    while(motor_curr->MotorState == MOTOR_STATE_GET_KV){
    	xSemaphoreGive(port->term_block);
    	vTaskDelay(200);
    	xQueueSemaphoreTake(port->term_block, portMAX_DELAY);
		ttprintf(".");
	}

	ttprintf("\r\nResult:\r\n");

	//motor_profile->flux_linkage = motor.motor_flux;

    ttprintf("Flux = %f mWb\r\n", motor_curr->m.flux_linkage * 1000.0);

    return TERM_CMD_EXIT_SUCCESS;
}


extern uint16_t deadtime_comp;
uint8_t CMD_detect(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){

	MESC_motor_typedef * motor_curr = &mtr[0];

	TestMode = TEST_TYPE_DEAD_TIME_IDENT;
	MESCmotor_state_set(MOTOR_STATE_TEST);

	ttprintf("Waiting for result");

	port_str * port = handle->port;
	while(motor_curr->MotorState == MOTOR_STATE_TEST){
		xSemaphoreGive(port->term_block);
		vTaskDelay(200);
		xQueueSemaphoreTake(port->term_block, portMAX_DELAY);
		ttprintf(".");
	}

	ttprintf("Deadtime register: %d\r\n", deadtime_comp);


    return TERM_CMD_EXIT_SUCCESS;
}

uint8_t CMD_detectHFI(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){

	MESC_motor_typedef * motor_curr = &mtr[0];

	float HFI_Threshold = detectHFI(motor_curr);

	ttprintf("HFI threshold: %f\r\n", HFI_Threshold);

    return TERM_CMD_EXIT_SUCCESS;
}

extern TIM_HandleTypeDef htim1;

uint8_t CMD_flash(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){

	if(argCount){
		if(strcmp(args[0], "w")==0){
			uint32_t len = sizeof(MOTORProfile);

			ProfileStatus ret = profile_put_entry( "MTR", MOTOR_PROFILE_SIGNATURE, motor_profile, &len );
			profile_commit();
			ttprintf("Writing to flash %s\r\n", ret==PROFILE_STATUS_SUCCESS? "successfully" : "failed");
		}
		if(strcmp(args[0], "d")==0){
			HAL_FLASH_Unlock();
			vTaskDelay(100);
			uint32_t      const addr = getFlashBaseAddress();
			ProfileStatus const ret  = eraseFlash( addr, PROFILE_MAX_SIZE );
			ttprintf("Flash erase %s\r\n", ret==PROFILE_STATUS_SUCCESS? "successful" : "failed");
			vTaskDelay(100);
			HAL_FLASH_Lock();
			profile_init();
		}

	}


    return TERM_CMD_EXIT_SUCCESS;
}


void populate_vars(){

	TERM_addVar(mtr[0].m.R, 0.0f, 10.0f, "r_phase", "Phase resistance", 0, &TERM_varList);
	TERM_addVar(mtr[0].m.L_D, 0.0f, 10.0f, "ld_phase", "Phase inductance", 0, &TERM_varList);
	TERM_addVar(mtr[0].m.L_Q, 0.0f, 10.0f, "lq_phase", "Phase inductance", 0, &TERM_varList);
	TERM_addVar(mtr[0].HFIType, 0, 3, "hfi", "HFI type [0=None, 1=45deg, 2=d axis]", 0, &TERM_varList);
	TERM_addVar(mtr[0].FOC.HFI_Threshold, 0.0f, 2.0f, "hfi_thresh", "HFI Threshold", 0, &TERM_varList);
	TERM_addVar(motor_profile->flux_linkage, 0.0f, 200.0f, "flux", "Flux linkage", 0, &TERM_varList);
}



void MESCinterface_init(void){
	static bool is_init=false;
	if(is_init) return;

	populate_vars();

	CMD_varLoad(&null_handle, 0, NULL);

	calculateGains(&mtr[0]);
	calculateVoltageGain(&mtr[0]);

	motor_profile->L_QD = motor_profile->L_Q-motor_profile->L_D;
	motor_profile->flux_linkage_max = 1.3f*motor_profile->flux_linkage;
	motor_profile->flux_linkage_min = 0.7f*motor_profile->flux_linkage;
	motor_profile->flux_linkage_gain = 10.0f * sqrtf(motor_profile->flux_linkage);

	mtr[0].m.flux_linkage_max = motor_profile->flux_linkage_max;
	mtr[0].m.flux_linkage_min = motor_profile->flux_linkage_min;
	mtr[0].m.flux_linkage_gain = motor_profile->flux_linkage_gain;

	//mtr[0].m.flux_linkage = motor_profile->flux_linkage;

	TERM_addCommand(CMD_measure, "measure", "Measure motor R+L", 0, &TERM_defaultList);
	TERM_addCommand(CMD_getkv, "getkv", "Measure motor kV", 0, &TERM_defaultList);
	TERM_addCommand(CMD_detect, "deadtime", "Detect deadtime compensation", 0, &TERM_defaultList);
	TERM_addCommand(CMD_status, "status", "Realtime data", 0, &TERM_defaultList);
	TERM_addCommand(CMD_flash, "flash", "Flash write", 0, &TERM_defaultList);
	TERM_addCommand(CMD_detectHFI, "hfi_detect", "Detect HFI", 0, &TERM_defaultList);

	REGISTER_apps(&TERM_defaultList);

	is_init=true;
}
