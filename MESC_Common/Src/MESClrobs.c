/*
 * MESClrobs.c
 *
 *  Created on: Sep 24, 2024
 *      Author: jkerrinnes
 */

#include <math.h>
#include "MESClrobs.h"

void MESClrobs_Init(MESC_motor_typedef *_motor){
	_motor->lrobs.plusminus = 1;
}

  //LR observer. WIP function that works by injecting a
  //low frequency Id signal into the PID input and observing the change in Vd and Vq
  //Does not work too well, requires some care in use.
  //Original work to MESC project.

 void MESClrobs_Run(MESC_motor_typedef *_motor){
	 if((fabsf(_motor->FOC.eHz)>0.005f*_motor->FOC.pwm_frequency)&&(_motor->HFI.inject ==0)){

		  _motor->lrobs.R_observer = (_motor->lrobs.Vd_obs_high_filt-_motor->lrobs.Vd_obs_low_filt)/(2.0f*LR_OBS_CURRENT);
		  _motor->lrobs.L_observer = (_motor->lrobs.Vq_obs_high_filt-_motor->lrobs.Vq_obs_low_filt-6.28f*(_motor->FOC.eHz-_motor->lrobs.Last_eHz)*_motor->FOC.flux_observed)/(2.0f*LR_OBS_CURRENT*6.28f*_motor->FOC.eHz);

		  if(_motor->lrobs.plusminus==1){
				_motor->lrobs.plusminus = -1;
				_motor->lrobs.Vd_obs_low_filt = _motor->lrobs.Vd_obs_low/_motor->lrobs.LR_collect_count;
				_motor->lrobs.Vq_obs_low_filt = _motor->lrobs.Vq_obs_low/_motor->lrobs.LR_collect_count;
				_motor->FOC.Idq_req.d = _motor->FOC.Idq_req.d+1.0f*LR_OBS_CURRENT;
				_motor->lrobs.Vd_obs_low = 0;
				_motor->lrobs.Vq_obs_low = 0;
		  }else if(_motor->lrobs.plusminus == -1){
				_motor->lrobs.plusminus = 1;
				_motor->lrobs.Vd_obs_high_filt = _motor->lrobs.Vd_obs_high/_motor->lrobs.LR_collect_count;
				_motor->lrobs.Vq_obs_high_filt = _motor->lrobs.Vq_obs_high/_motor->lrobs.LR_collect_count;
				_motor->FOC.Idq_req.d = _motor->FOC.Idq_req.d-1.0f*LR_OBS_CURRENT;
				_motor->lrobs.Vd_obs_high = 0;
				_motor->lrobs.Vq_obs_high = 0;
		  }
		_motor->lrobs.Last_eHz = _motor->FOC.eHz;
		_motor->lrobs.LR_collect_count = 0; //Reset this after doing the calcs
	  }
#if 0
	  	float Rerror = R_observer-_motor->m.R;
	  	float Lerror = L_observer-_motor->m.L_D;
	  	//Apply the correction excluding large changes
	  	if(fabs(Rerror)<0.1f*_motor->m.R){
	  		_motor->m.R = _motor->m.R+0.1f*Rerror;
	  	}else if(fabs(Rerror)<0.5f*_motor->m.R){
	  		_motor->m.R = _motor->m.R+0.001f*Rerror;
	  	}
	  	if(fabs(Lerror)<0.1f*_motor->m.L_D){
	  		_motor->m.L_D = _motor->m.L_D+0.1f*Lerror;
	  		_motor->m.L_Q = _motor->m.L_Q +0.1f*Lerror;
	  	}else if(fabs(Lerror)<0.5f*_motor->m.L_D){
	  		_motor->m.L_D = _motor->m.L_D+0.001f*Lerror;
	  		_motor->m.L_Q = _motor->m.L_Q +0.001f*Lerror;
	  	}

#endif
}

void MESClrobs_Collect(MESC_motor_typedef *_motor){
	_motor->lrobs.LR_collect_count++;
	if((fabsf(_motor->FOC.eHz)>0.005f*_motor->FOC.pwm_frequency)&&(_motor->HFI.inject ==0)){
		if(_motor->lrobs.plusminus==1){
			_motor->lrobs.Vd_obs_low = _motor->lrobs.Vd_obs_low + _motor->FOC.Vdq.d;
			_motor->lrobs.Vq_obs_low = _motor->lrobs.Vq_obs_low + _motor->FOC.Vdq.q;
		}
		if(_motor->lrobs.plusminus == -1){
			_motor->lrobs.Vd_obs_high = _motor->lrobs.Vd_obs_high + _motor->FOC.Vdq.d;
			_motor->lrobs.Vq_obs_high = _motor->lrobs.Vq_obs_high + _motor->FOC.Vdq.q;
		}
	}
}
