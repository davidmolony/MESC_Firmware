

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
	motor.measure_current = I_MEASURE;
	motor.measure_voltage = V_MEASURE;

	bool old_hfi_state = motor1.FOC.hfi_enable;
	motor1.FOC.hfi_enable = true;

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

	MESCmotor_state_set(MOTOR_STATE_MEASURING);
    ttprintf("Waiting for result");

    port_str * port = handle->port;
    while(MotorState == MOTOR_STATE_MEASURING){
    	xSemaphoreGive(port->term_block);
    	vTaskDelay(200);
    	xQueueSemaphoreTake(port->term_block, portMAX_DELAY);
    	ttprintf(".");
    }

    ttprintf("\r\nResult:\r\n");

    float R, Lq, Ld;
    char* Runit;
    char* Lunit;
    if(motor.Rphase > 0){
    	R = motor.Rphase;
    	Runit = "Ohm";
    }else{
    	R = motor.Rphase*1000.0f;
    	Runit = "mOhm";
    }
    if(motor.Lphase > 0.001f){
		Ld = motor.Lphase*1000.0f;
		Lq = motor.Lqphase*1000.0f;
		Lunit = "mH";
	}else{
		Ld = motor.Lphase*1000.0f*1000.0f;
		Lq = motor.Lqphase*1000.0f*1000.0f;
		Lunit = "uH";
	}

    motor1.FOC.hfi_enable = old_hfi_state;

    motor_profile->R = motor.Rphase;
    motor_profile->L_D = motor.Lphase;
    motor_profile->L_Q = motor.Lqphase;

    ttprintf("R = %f %s\r\nLd = %f %s\r\nLq = %f %s\r\n", R, Runit, Ld, Lunit, Lq, Lunit);

    return TERM_CMD_EXIT_SUCCESS;
}

uint8_t CMD_getkv(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
	motor.measure_current = I_MEASURE;
	motor.measure_voltage = V_MEASURE;

	if(argCount==1){
		if(strcmp(args[0], "-?")==0){
			ttprintf("Usage: getkv [optional: measure current]\r\n");
			return TERM_CMD_EXIT_SUCCESS;
		}

		motor.measure_current = atoff(args[0]);

	}

	MESCmotor_state_set(MOTOR_STATE_GET_KV);
    ttprintf("Waiting for result");

    port_str * port = handle->port;
    while(MotorState == MOTOR_STATE_GET_KV){
    	xSemaphoreGive(port->term_block);
    	vTaskDelay(200);
    	xQueueSemaphoreTake(port->term_block, portMAX_DELAY);
		ttprintf(".");
	}

	ttprintf("\r\nResult:\r\n");

	motor_profile->flux_linkage = motor.motor_flux;

    ttprintf("Flux = %f mWb\r\n", motor.motor_flux * 1000.0);

    return TERM_CMD_EXIT_SUCCESS;
}


extern uint16_t deadtime_comp;
uint8_t CMD_detect(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
	TestMode = TEST_TYPE_DEAD_TIME_IDENT;
	MESCmotor_state_set(MOTOR_STATE_TEST);

	ttprintf("Waiting for result");

	port_str * port = handle->port;
	while(MotorState == MOTOR_STATE_TEST){
		xSemaphoreGive(port->term_block);
		vTaskDelay(200);
		xQueueSemaphoreTake(port->term_block, portMAX_DELAY);
		ttprintf(".");
	}

	ttprintf("Deadtime register: %d\r\n", deadtime_comp);


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

char buff[16];
float hall[6] = {1,2,3,4,5,6};

void MESCinterface_init(void){
	static bool is_init=false;
	if(is_init) return;

//	cli_register_var_rw("idq_req" 	, input_vars.Idq_req_UART.q, NULL);
//    cli_register_var_rw( "id"     	, foc_vars.Idq_req.d, NULL);
//    cli_register_var_rw( "iq"     	, foc_vars.Idq_req.q, NULL);
//    cli_register_var_ro( "vbus"   	, measurement_buffers.ConvertedADC[0][1], NULL);
//    cli_register_var_rw( "ld"     	, motor.Lphase, calculateGains);
//    cli_register_var_rw( "lq"     	, motor.Lqphase, calculateGains);
//    cli_register_var_rw( "r"      	, motor.Rphase, calculateGains);
//    cli_register_var_rw( "flux"     , motor.motor_flux, calculateGains);
//    cli_register_var_rw( "pwm_freq" , foc_vars.pwm_frequency, calculateGains);
//    cli_register_var_rw( "hfi"      , foc_vars.hfi_enable, NULL);

	memcpy(buff, "Hello World", sizeof("Hello World"));



	TERM_addVar(motor.Rphase, 0.0f, 10.0f, "r_phase", "Phase resistance", 0, &TERM_varList);
	TERM_addVar(motor.Lphase, 0.0f, 10.0f, "ld_phase", "Phase inductance", 0, &TERM_varList);
	TERM_addVar(motor.Lqphase, 0.0f, 10.0f, "lq_phase", "Phase inductance", 0, &TERM_varList);
	//TERM_addVar(motor.motor_flux, 0.0f, 100.0f, "flux", "Flux linkage", 0, &TERM_varList);
	TERM_addVar(buff, 0, 0, "name", "ESC name", 0, &TERM_varList);
	TERM_addVar(motor1.FOC.hfi_enable, 0, 0, "hfi", "Enable HFI", 0, &TERM_varList);
	TERM_addVar(input_vars.Idq_req_UART.q, -100.0f, 100.0f, "iq_req", "IQ request", 0, &TERM_varList);
	TERM_addVar(hall, -10.0f, 10.0f, "hall", "Hall array", 0, &TERM_varList);


//	TERM_addVarFloat(&motor.Rphase, TERM_VARIABLE_FLOAT, sizeof(motor.Rphase),"r_phase", "Phase resistance", 0, &TERM_varList);
//	TERM_addVarFloat(&motor.Lphase, TERM_VARIABLE_FLOAT, sizeof(motor.Lphase),"lq_phase", "Phase inductance", 0, &TERM_varList);

	TERM_addCommand(CMD_measure, "measure", "Measure motor R+L", 0, &TERM_defaultList);
	TERM_addCommand(CMD_getkv, "getkv", "Measure motor kV", 0, &TERM_defaultList);
	TERM_addCommand(CMD_detect, "deadtime", "Detect deadtime compensation", 0, &TERM_defaultList);
//	TERM_addCommand(cli_read, "read", "Read variable", 0, &TERM_defaultList);
//	TERM_addCommand(cli_write, "write", "Write variable", 0, &TERM_defaultList);
//	TERM_addCommand(cli_list, "list", "List all variables", 0, &TERM_defaultList);
	TERM_addCommand(CMD_status, "status", "Realtime data", 0, &TERM_defaultList);
	TERM_addCommand(profile_cli_info, "profile", "Profile info", 0, &TERM_defaultList);
	TERM_addCommand(CMD_flash, "flash", "Flash write", 0, &TERM_defaultList);


	REGISTER_apps(&TERM_defaultList);
	is_init=true;
}
