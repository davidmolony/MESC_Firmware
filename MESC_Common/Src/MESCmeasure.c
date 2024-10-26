/*
 **
 ******************************************************************************
 * @file           : MESCmeasure.c
 * @brief          : Motor detection and measurement
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 David Molony.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
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
 ******************************************************************************
 */

#include "MESCmeasure.h"
#include "MESCpwm.h"
#include "MESCfluxobs.h"

 void MESCmeasure_RL(MESC_motor_typedef *_motor) {
	 switch(_motor->meas.state) {
	 	 case MEAS_STATE_IDLE:
	 		_motor->meas.state = MEAS_STATE_INIT;
	 		_motor->FOC.PLL_int = 0.0f;
	 		_motor->FOC.PLL_angle = 0;
	 		break;
	 	 case MEAS_STATE_INIT:
			_motor->meas.previous_HFI_type = _motor->HFI.Type;
			uint16_t half_ARR = _motor->mtimer->Instance->ARR / 2;
			_motor->mtimer->Instance->CCR1 = half_ARR;
			_motor->mtimer->Instance->CCR2 = half_ARR;
			_motor->mtimer->Instance->CCR3 = half_ARR;
			_motor->m.R = 0.001f;     // Initialise with a very low value 1mR
			_motor->m.L_D = 0.000001f;  // Initialise with a very low value 1uH
			_motor->m.L_Q = 0.000001f;
			calculateVoltageGain(_motor);    // Set initial gains to enable MESCFOC to run
			calculateGains(_motor);
			MESCpwm_phU_Enable(_motor);
			MESCpwm_phV_Enable(_motor);
			MESCpwm_phW_Enable(_motor);
			_motor->FOC.Idq_req.d = _motor->meas.measure_current;
			_motor->FOC.Idq_req.q = 0.0f;
			_motor->FOC.FOCAngle = 0;

			_motor->HFI.inject = 0;  // flag to not inject at SVPWM top

			MESCFOC(_motor);

			_motor->meas.top_V = 0;
			_motor->meas.bottom_V = 0;
			_motor->meas.top_I = 0;
			_motor->meas.bottom_I = 0;
			_motor->meas.top_I_L = 0;
			_motor->meas.bottom_I_L = 0;
			_motor->meas.top_I_Lq = 0;
			_motor->meas.bottom_I_Lq = 0;

			_motor->meas.count_top = 0.0f;
			_motor->meas.count_bottom = 0.0f;

			_motor->meas.PWM_cycles = 0;

			_motor->meas.state = MEAS_STATE_ALIGN; //Next Step
	 		break;
	 	 case MEAS_STATE_ALIGN:
	 	      _motor->FOC.Idq_req.d = _motor->meas.measure_current;
	 	      _motor->FOC.Idq_req.q = 0.0f;

	 	      _motor->HFI.inject = 0;
	 	      MESCFOC(_motor);
	 	      if(_motor->meas.PWM_cycles > _motor->FOC.pwm_frequency){ // 1second
	 	    	 _motor->meas.state = MEAS_STATE_LOWER_SETPOINT;
	 	    	 _motor->meas.PWM_cycles = 0;
	 	      }
	 		 break;
	 	 case MEAS_STATE_LOWER_SETPOINT:
			_motor->FOC.Idq_req.d = 0.20f*_motor->meas.measure_current;
			_motor->HFI.inject = 0;
			MESCFOC(_motor);

			_motor->meas.bottom_V = _motor->meas.bottom_V + _motor->FOC.Vdq.d;
			_motor->meas.bottom_I = _motor->meas.bottom_I + _motor->FOC.Idq.d;
			_motor->meas.count_bottom++;
			_motor->meas.Vd_temp = _motor->FOC.Vdq.d * 1.0f;  // Store the voltage required for the low setpoint, to
											 // use as an offset for the inductance
			if(_motor->meas.PWM_cycles > 5000){
				_motor->meas.state = MEAS_STATE_UPPER_SETPOINT;
				_motor->meas.PWM_cycles = 0;
			}
			break;
	 	 case MEAS_STATE_UPPER_SETPOINT_STABILISATION:
			_motor->FOC.Idq_req.d = _motor->meas.measure_current;
			_motor->HFI.inject = 0;
			MESCFOC(_motor);
	 		if(_motor->meas.PWM_cycles > 5000){
	 			_motor->meas.state = MEAS_STATE_UPPER_SETPOINT;
				_motor->meas.PWM_cycles = 0;
			}
	 		break;
	 	 case MEAS_STATE_UPPER_SETPOINT:
			_motor->FOC.Idq_req.d = _motor->meas.measure_current;
			_motor->HFI.inject = 0;
			MESCFOC(_motor);

			_motor->meas.top_V = _motor->meas.top_V + _motor->FOC.Vdq.d;
			_motor->meas.top_I = _motor->meas.top_I + _motor->FOC.Idq.d;
			_motor->meas.count_top++;
			_motor->meas.Vd_temp = _motor->FOC.Vdq.d * 0.75f;  // Store the voltage required for the low setpoint, to
											 // use as an offset for the inductance
			if(_motor->meas.PWM_cycles > 5000){
				MESCpwm_generateBreak(_motor);
				//Calculate R
				_motor->m.R = (_motor->meas.top_V - _motor->meas.bottom_V) / (_motor->meas.top_I - _motor->meas.bottom_I);

				_motor->meas.state = MEAS_STATE_INIT_LD;
				_motor->meas.PWM_cycles = 0;
			}
			break;
	 	 case MEAS_STATE_INIT_LD:
			//Initialise the variables for the next measurement
			//Vd_temp = _motor->FOC.Vdq.d * 1.0f;  // Store the voltage required for the high setpoint, to
											 // use as an offset for the inductance
			_motor->meas.Vq_temp = 0.0f;
			_motor->FOC.Vdq.q = 0.0f;//
			_motor->FOC.Idq_int_err.d = 0.0f;
			_motor->FOC.Idq_int_err.q = 0.0f;
			_motor->meas.count_top = 0.0f;
			_motor->meas.count_bottom = 0.0f;
			_motor->meas.top_I_L = 0.0f;
			_motor->meas.bottom_I_L = 0.0f;

			MESCpwm_generateEnable(_motor);
			_motor->meas.state = MEAS_STATE_COLLECT_LD;
			_motor->meas.PWM_cycles = 0;
	 		break;
	 	 case MEAS_STATE_COLLECT_LD:
			_motor->HFI.Type = HFI_TYPE_SPECIAL;
			_motor->HFI.inject = 1;  // flag to the SVPWM writer to inject at top
			_motor->HFI.special_injectionVd = _motor->meas.measure_voltage;
			_motor->HFI.special_injectionVq = 0.0f;

			_motor->FOC.Vdq.d = _motor->meas.Vd_temp;
			_motor->FOC.Vdq.q = 0.0f;


			if (_motor->HFI.inject_high_low_now == 1) {
			  _motor->meas.top_I_L = _motor->meas.top_I_L + _motor->FOC.Idq.d;
			  _motor->meas.count_top++;
			} else if (_motor->HFI.inject_high_low_now == 0) {
			  _motor->meas.bottom_I_L = _motor->meas.bottom_I_L + _motor->FOC.Idq.d;
			  _motor->meas.count_bottom++;
			}
			if(_motor->meas.PWM_cycles > _motor->FOC.pwm_frequency){ // 1second
				_motor->meas.state = MEAS_STATE_INIT_LQ;
				_motor->meas.PWM_cycles = 0;
			}
	 		break;
	 	 case MEAS_STATE_INIT_LQ:
	 		MESCpwm_generateBreak(_motor);
			_motor->m.L_D =
			  fabsf((_motor->HFI.special_injectionVd) /
			  ((_motor->meas.top_I_L - _motor->meas.bottom_I_L) / (_motor->meas.count_top * _motor->FOC.pwm_period)));
			_motor->meas.top_I_Lq = 0.0f;
			_motor->meas.bottom_I_Lq = 0.0f;
			_motor->meas.count_topq = 0.0f;
			_motor->meas.count_bottomq = 0.0f;
			if(_motor->meas.PWM_cycles > 2){ //Wait a bit
				MESCpwm_phU_Enable(_motor);
				MESCpwm_phV_Enable(_motor);
				MESCpwm_phW_Enable(_motor);
				_motor->meas.state = MEAS_STATE_COLLECT_LQ;
				_motor->meas.PWM_cycles = 0;
			}
	 		break;
	 	case MEAS_STATE_COLLECT_LQ:
			_motor->HFI.special_injectionVd = 0.0f;
			_motor->HFI.special_injectionVq = _motor->meas.measure_voltage;
			_motor->HFI.inject = 1;  // flag to the SVPWM writer to update at top
			_motor->FOC.Vdq.d = _motor->meas.Vd_temp;  // Vd_temp to keep it aligned with D axis
			_motor->FOC.Vdq.q = 0.0f;


			if (_motor->HFI.inject_high_low_now == 1) {
			  _motor->meas.top_I_Lq = _motor->meas.top_I_Lq + _motor->FOC.Idq.q;
			  _motor->meas.count_topq++;
			} else if (_motor->HFI.inject_high_low_now == 0) {
			  _motor->meas.bottom_I_Lq = _motor->meas.bottom_I_Lq + _motor->FOC.Idq.q;
			_motor->meas.count_bottomq++;
			}

			if(_motor->meas.PWM_cycles > _motor->FOC.pwm_frequency){ // 1second
				MESCpwm_generateBreak(_motor);
				_motor->m.L_Q =
				  fabsf((_motor->HFI.special_injectionVq) /
				  ((_motor->meas.top_I_Lq - _motor->meas.bottom_I_Lq) / (_motor->meas.count_top * _motor->FOC.pwm_period)));


			      _motor->HFI.Type = _motor->meas.previous_HFI_type;
			      _motor->MotorState = MOTOR_STATE_IDLE;

			      _motor->HFI.inject = 0;  // flag to the SVPWM writer stop injecting at top
			      _motor->HFI.special_injectionVd = 0.0f;
			      _motor->HFI.special_injectionVq = 0.0f;
			      _motor->HFI.Vd_injectionV = 0.0f;
			      _motor->HFI.Vq_injectionV = 0.0f;
			      calculateGains(_motor);
			      _motor->MotorState = MOTOR_STATE_TRACKING;
			      _motor->meas.PWM_cycles = 0;
			      MESCpwm_phU_Enable(_motor);
			      MESCpwm_phV_Enable(_motor);
			      MESCpwm_phW_Enable(_motor);

			      _motor->meas.state = MEAS_STATE_IDLE;
			}

	 		break;
	 	 default:
	 		_motor->meas.state = MEAS_STATE_IDLE;
	 		 break;


	 }
	 _motor->meas.PWM_cycles++;
 }


 void MESCmeasure_GetkV(MESC_motor_typedef *_motor) {
	_motor->meas.previous_HFI_type = _motor->HFI.Type;
	_motor->HFI.Type=HFI_TYPE_NONE;
 	_motor->HFI.inject = 0;

   static int cycles = 0;
   static HFI_type_e old_HFI_type;
   if (cycles < 2) {
   	_motor->m.flux_linkage_max = 0.1f;
   	_motor->m.flux_linkage_min = 0.00001f;//Set really wide limits
   	_motor->FOC.openloop_step = 0;
   	_motor->FOC.flux_observed = _motor->m.flux_linkage_min;
   	old_HFI_type = _motor->HFI.Type;
   	_motor->HFI.Type = HFI_TYPE_NONE;
       MESCpwm_phU_Enable(_motor);
       MESCpwm_phV_Enable(_motor);
       MESCpwm_phW_Enable(_motor);
   }

   MESCfluxobs_run(_motor);//We run the flux observer during this

   static int count = 0;
   static uint16_t temp_angle;
   if (cycles < 60002) {
       _motor->FOC.Idq_req.d = _motor->meas.measure_current*0.5f;  //
       _motor->FOC.Idq_req.q = 0.0f;
   	_motor->meas.angle_delta = temp_angle-_motor->FOC.FOCAngle;
   	_motor->FOC.openloop_step = (uint16_t)(ERPM_MEASURE*65536.0f/(_motor->FOC.pwm_frequency*60.0f)*(float)cycles/65000.0f);
   	_motor->FOC.FOCAngle = temp_angle;
       OLGenerateAngle(_motor);
       temp_angle = _motor->FOC.FOCAngle;
       if(cycles==60001){
       	_motor->meas.temp_flux = sqrtf(_motor->FOC.Vdq.d*_motor->FOC.Vdq.d+_motor->FOC.Vdq.q*_motor->FOC.Vdq.q)/(6.28f * (float)_motor->FOC.openloop_step * (float)_motor->FOC.pwm_frequency/65536.0f);
       	_motor->FOC.flux_observed  = _motor->meas.temp_flux;
       	_motor->FOC.flux_a = _motor->FOC.sincosangle.cos*_motor->FOC.flux_observed;
       	_motor->FOC.flux_b = _motor->FOC.sincosangle.sin*_motor->FOC.flux_observed;
       	_motor->m.flux_linkage_max = 1.7f*_motor->FOC.flux_observed;
       	_motor->m.flux_linkage_min = 0.5f*_motor->FOC.flux_observed;
       	_motor->meas.temp_FLA = _motor->FOC.flux_a;
       	_motor->meas.temp_FLB = _motor->FOC.flux_b;
       }
       MESCFOC(_motor);
   } else if (cycles < 128000) {
     count++;
     _motor->FOC.Idq_req.d = 0.0f;
     _motor->FOC.Idq_req.q = _motor->meas.measure_closedloop_current;
     MESCFOC(_motor);
   } else {
      MESCpwm_generateBreak(_motor);
      _motor->m.flux_linkage = _motor->FOC.flux_observed;
      calculateFlux(_motor);
     _motor->MotorState = MOTOR_STATE_TRACKING;
     _motor->HFI.Type = old_HFI_type;
     cycles = 0;
     _motor->HFI.Type = _motor->meas.previous_HFI_type;
     if (_motor->m.flux_linkage > 0.0001f && _motor->m.flux_linkage < 200.0f) {
   	_motor->MotorSensorMode = MOTOR_SENSOR_MODE_SENSORLESS;
     } else {
       _motor->MotorState = MOTOR_STATE_ERROR;
       MESCpwm_generateBreak(_motor);
     }
   }
//    writePWM(_motor);

   cycles++;

 }

float MESCmeasure_DetectHFI(MESC_motor_typedef *_motor){

 	static float dinductance, qinductance;
 	  ///Try out a new detection routine
 #if 1

 	_motor->meas.previous_HFI_type = _motor->HFI.Type;
 	_motor->HFI.Type = HFI_TYPE_D;
 	_motor->input_vars.UART_req = 0.25f;
 	int a = 0;
 	dinductance = 0;
 	qinductance = 0;
 	while(a<1000){
 		a++;
 		_motor->HFI.Type = HFI_TYPE_D;
 		dinductance = dinductance + _motor->FOC.didq.d;
 		HAL_Delay(0);
 		//input_vars.input_options = 0b
 	}
 	dinductance = dinductance/1000.0f;
 	//dinductance = motor1.FOC.pwm_period*motor1.HFI.Vd_injectionV/(motor1.Conv.Vbus*dinductance);
 	//Vdt/di = L
 	_motor->FOC.d_polarity = -1;
 	a=0;
 	while(a<1000){
 		a++;
 		_motor->HFI.Type = HFI_TYPE_D;
 		qinductance = qinductance + _motor->FOC.didq.d;
 		HAL_Delay(0);
 		//input_vars.input_options = 0b
 	}
 	qinductance = qinductance/1000.0f; //Note that this is not yet an inductance, but an inverse of inductance*voltage
 	_motor->HFI.mod_didq = sqrtf(qinductance*qinductance+dinductance*dinductance);
 	_motor->HFI.Gain = 5000.0f/_motor->HFI.mod_didq; //Magic numbers that seem to work
 	_motor->input_vars.UART_req = 0.0f;
 	_motor->FOC.d_polarity = 1;

 	_motor->HFI.Type = _motor->meas.previous_HFI_type;

 	return _motor->HFI.mod_didq;

 #endif
}

void MESCmeasure_GetDeadtime(MESC_motor_typedef *_motor){
	  static uint16_t test_on_time;
	  static uint16_t test_on_time_acc[3];
	  static uint16_t test_counts;

	  static int use_phase = 0;

	  if(test_on_time<1){test_on_time = 1;}

		if(use_phase==0){
			_motor->mtimer->Instance->CCR1 = test_on_time;
			_motor->mtimer->Instance->CCR2 = 0;
			_motor->mtimer->Instance->CCR3 = 0;
			if(_motor->Conv.Iu<1.0f){ test_on_time=test_on_time+1;}
			if(_motor->Conv.Iu>1.0f){ test_on_time=test_on_time-1;}
			MESCpwm_generateEnable(_motor);
			test_on_time_acc[0] = test_on_time_acc[0]+test_on_time;
			}
		if(use_phase==1){
			_motor->mtimer->Instance->CCR1 = 0;
			_motor->mtimer->Instance->CCR2 = test_on_time;
			_motor->mtimer->Instance->CCR3 = 0;
			if(_motor->Conv.Iv<1.0f){ test_on_time=test_on_time+1;}
			if(_motor->Conv.Iv>1.0f){ test_on_time=test_on_time-1;}
			MESCpwm_generateEnable(_motor);
			test_on_time_acc[1] = test_on_time_acc[1]+test_on_time;
		}
		if(use_phase==2){
			_motor->mtimer->Instance->CCR1 = 0;
			_motor->mtimer->Instance->CCR2 = 0;
			_motor->mtimer->Instance->CCR3 = test_on_time;
			if(_motor->Conv.Iw<1.0f){ test_on_time=test_on_time+1;}
			if(_motor->Conv.Iw>1.0f){ test_on_time=test_on_time-1;}
			MESCpwm_generateEnable(_motor);
			test_on_time_acc[2] = test_on_time_acc[2]+test_on_time;
		}
		if(use_phase>2){
			MESCpwm_generateBreak(_motor);
			_motor->MotorState = MOTOR_STATE_TRACKING;
			use_phase = 0;
			test_on_time_acc[0] = test_on_time_acc[0]>>10;
			test_on_time_acc[1] = test_on_time_acc[1]>>10;
			test_on_time_acc[2] = test_on_time_acc[2]>>10;
			_motor->FOC.deadtime_comp = test_on_time_acc[0];
		}
		test_counts++;

	if(test_counts>511){
		use_phase++;
	test_counts = 0;
	}
}

void MESCmeasure_GetHallTable(MESC_motor_typedef *_motor) {
  static int firstturn = 1;
  static int hallstate;
  hallstate = getHallState();
  static int lasthallstate = -1;
  static uint16_t pwm_count = 0;
  static int anglestep = 5;  // This defines how fast the motor spins
  static uint32_t hallangles[7][2];
  static int rollover;
  hallstate = _motor->hall.current_hall_state;
  if (firstturn) {
  	MESCpwm_generateEnable(_motor);
    lasthallstate = hallstate;
    (void)lasthallstate;
    firstturn = 0;
  }

  ////// Align the rotor////////////////////
  static uint16_t a = 65535;
  if (a)  // Align time
  {
    _motor->FOC.Idq_req.d = 10.0f;
    _motor->FOC.Idq_req.q = 0.0f;

    _motor->FOC.FOCAngle = 0.0f;
    a = a - 1;
  } else {
    _motor->FOC.Idq_req.d = 10.0f;
    _motor->FOC.Idq_req.q = 0.0f;
    static int dir = 1;
    if (pwm_count < 65534) {
      if (_motor->FOC.FOCAngle < (anglestep)) {
        rollover = hallstate;
      }
      if ((_motor->FOC.FOCAngle < (30000)) &&
          (_motor->FOC.FOCAngle > (29000 - anglestep))) {
        rollover = 0;
      }
      lasthallstate = hallstate;
      if (rollover == hallstate) {
        hallangles[hallstate][0] =
            hallangles[hallstate][0] +
            (uint32_t)65535;  // Accumulate the angles through the sweep
      }

      _motor->FOC.FOCAngle =
          _motor->FOC.FOCAngle + anglestep;  // Increment the angle
      hallangles[hallstate][0] =
          hallangles[hallstate][0] +
          _motor->FOC.FOCAngle;       // Accumulate the angles through the sweep
      hallangles[hallstate][1]++;  // Accumulate the number of PWM pulses for
                                   // this hall state
      pwm_count = pwm_count + 1;
    } else if (pwm_count < 65535) {
      if (dir == 1) {
        dir = 0;
        rollover = 0;
      }
      if ((_motor->FOC.FOCAngle < (12000)) && (hallstate != _motor->hall.last_hall_state)) {
        rollover = hallstate;
      }
      if ((_motor->FOC.FOCAngle < (65535)) &&
          (_motor->FOC.FOCAngle > (65535 - anglestep))) {
        rollover = 0;
      }
      lasthallstate = hallstate;
      if (rollover == hallstate) {
        hallangles[hallstate][0] =
            hallangles[hallstate][0] +
            (uint32_t)65535;  // Accumulate the angles through the sweep
      }

      _motor->FOC.FOCAngle =
          _motor->FOC.FOCAngle - anglestep;  // Increment the angle
      hallangles[hallstate][0] =
          hallangles[hallstate][0] +
          _motor->FOC.FOCAngle;       // Accumulate the angles through the sweep
      hallangles[hallstate][1]++;  // Accumulate the number of PWM pulses for
                                   // this hall state
      pwm_count = pwm_count + 1;
    }
  }
  if (pwm_count == 65535) {
    MESCpwm_generateBreak(_motor);  // Debugging
    for (int i = 1; i < 7; i++) {
      hallangles[i][0] = hallangles[i][0] / hallangles[i][1];
      if (hallangles[i][0] > 65535) {
        hallangles[i][0] = hallangles[i][0] - 65535;
      }
    }
    for (int i = 0; i < 6; i++) {
          _motor->m.hall_table[i][2] = hallangles[i + 1][0];//This is the center angle of the hall state
          _motor->m.hall_table[i][3] = hallangles[i + 1][1];//This is the width of the hall state
          _motor->m.hall_table[i][0] = _motor->m.hall_table[i][2]-_motor->m.hall_table[i][3]/2;//This is the start angle of the hall state
          _motor->m.hall_table[i][1] = _motor->m.hall_table[i][2]+_motor->m.hall_table[i][3]/2;//This is the end angle of the hall state
    }
    _motor->MotorState = MOTOR_STATE_TRACKING;
    _motor->FOC.Idq_req.d = 0;
    _motor->FOC.Idq_req.q = 0;
    MESCpwm_phU_Enable(_motor);
    MESCpwm_phV_Enable(_motor);
    MESCpwm_phW_Enable(_motor);
  }
}

static volatile int dp_periods = 6;
void MESCmeasure_DoublePulseTest(MESC_motor_typedef *_motor) {
	static int dp_counter;
	if  (dp_counter == 0) { //Let bootstrap charge
		__HAL_TIM_DISABLE_IT(_motor->mtimer,TIM_IT_UPDATE); //DISABLE INTERRUPT, DANGEROUS
		MESCpwm_phU_Enable(_motor);
		MESCpwm_phV_Enable(_motor);
		MESCpwm_phW_Enable(_motor);
		_motor->mtimer->Instance->CCR1 = 0;
		_motor->mtimer->Instance->CCR2 = 0;
		_motor->mtimer->Instance->CCR3 = 0;
		_motor->test_vals.dp_current_final[dp_counter] = _motor->Conv.Iv;
		dp_counter++;
	}else if(dp_counter <= (dp_periods-3)) { //W State ON
    	_motor->mtimer->Instance->CCR1 = 0;
    	_motor->mtimer->Instance->CCR2 = 0;
    	_motor->mtimer->Instance->CCR3 = _motor->mtimer->Instance->ARR;
    	MESCpwm_phU_Break(_motor);
    	MESCpwm_phV_Enable(_motor);
    	MESCpwm_phW_Enable(_motor);
    	_motor->test_vals.dp_current_final[dp_counter] = _motor->Conv.Iv;
    	dp_counter++;
	} else if (dp_counter == (dp_periods-2)) { //Freewheel
	  _motor->mtimer->Instance->CCR2 = 0;
	  _motor->mtimer->Instance->CCR3 = 0;
	  _motor->test_vals.dp_current_final[dp_counter] = _motor->Conv.Iv;
      dp_counter++;
	}else if (dp_counter == (dp_periods-1)) { //W short second pulse
	   _motor->mtimer->Instance->CCR2 = 0;
	   _motor->mtimer->Instance->CCR3 = 200;
	   _motor->test_vals.dp_current_final[dp_counter] = _motor->Conv.Iv;
	   dp_counter++;
	} else if (dp_counter == dp_periods) { //Freewheel a bit to see the current
	   _motor->mtimer->Instance->CCR2 = 0;
	   _motor->mtimer->Instance->CCR3 = 0;
	   _motor->test_vals.dp_current_final[dp_counter] = _motor->Conv.Iv;
        dp_counter++;
    }else { //Turn all off
    	_motor->mtimer->Instance->CCR1 = 0;
    	_motor->mtimer->Instance->CCR2 = 0;
    	_motor->mtimer->Instance->CCR3 = 0;
    	_motor->test_vals.dp_current_final[dp_counter] = _motor->Conv.Iv;
    	dp_counter = 0;
    	MESCpwm_generateBreak(_motor);
    	__HAL_TIM_ENABLE_IT(_motor->mtimer, TIM_IT_UPDATE);///RE-ENABLE INTERRUPT
    	_motor->MotorState = MOTOR_STATE_TRACKING;
    }
}
