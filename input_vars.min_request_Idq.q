./MESC_L431RC/Core/Src/MESChw_setup.c:68:  _motor->Raw.Iu = hadc1.Instance->JDR1;  // U Current
./MESC_L431RC/Core/Src/MESChw_setup.c:69:  _motor->Raw.Iv = hadc1.Instance->JDR2;  // V Current
./MESC_L431RC/Core/Src/MESChw_setup.c:70:  _motor->Raw.Iw = hadc1.Instance->JDR3;  // W Current
./MESC_L431RC/Core/Src/MESChw_setup.c:71:  _motor->Raw.Vbus = hadc1.Instance->JDR4;  // DC Link Voltage
./MESC_L431RC/Core/Src/MESChw_setup.c:74:  _motor->Raw.ADC_in_ext1 = ADC_buffer[4];  // Throttle external inputs on buffer 4=ADC_in10, 5=ADC_in11 and 7=ADC_in14 on Mxlemming FOCcontrol board, pins 21,22,25
./MESC_L431RC/Core/Src/MESChw_setup.c:75:  _motor->Raw.ADC_in_ext2 = ADC_buffer[5];  // Throttle
./MESC_L431RC/Core/Src/MESChw_setup.c:76:  _motor->Raw.Motor_T = ADC_buffer[6];		//Motor input pin 24, buffer 6=ADC_in13
./MESC_L431RC/Core/Src/MESChw_setup.c:99:	  _motor->Raw.Vu = ADC_buffer[0]; //PhaseU Voltage
./MESC_L431RC/Core/Src/MESChw_setup.c:100:	  _motor->Raw.Vv = ADC_buffer[1]; //PhaseV Voltage
./MESC_L431RC/Core/Src/MESChw_setup.c:101:	  _motor->Raw.Vw = ADC_buffer[2]; //PhaseW Voltage
./MESC_L431RC/Core/Src/MESChw_setup.c:217:	HAL_TIM_PWM_Start(_motor->mtimer, TIM_CHANNEL_4);
./MESC_L431RC/Core/Src/MESChw_setup.c:219:	HAL_TIM_PWM_Start(_motor->mtimer, TIM_CHANNEL_1);
./MESC_L431RC/Core/Src/MESChw_setup.c:220:	HAL_TIMEx_PWMN_Start(_motor->mtimer, TIM_CHANNEL_1);
./MESC_L431RC/Core/Src/MESChw_setup.c:222:	HAL_TIM_PWM_Start(_motor->mtimer, TIM_CHANNEL_2);
./MESC_L431RC/Core/Src/MESChw_setup.c:223:	HAL_TIMEx_PWMN_Start(_motor->mtimer, TIM_CHANNEL_2);
./MESC_L431RC/Core/Src/MESChw_setup.c:225:	HAL_TIM_PWM_Start(_motor->mtimer, TIM_CHANNEL_3);
./MESC_L431RC/Core/Src/MESChw_setup.c:226:	HAL_TIMEx_PWMN_Start(_motor->mtimer, TIM_CHANNEL_3);
./MESC_L431RC/Core/Src/MESChw_setup.c:232:	__HAL_TIM_ENABLE_IT(_motor->mtimer,TIM_IT_UPDATE);
./MESC_F405RG/Core/Src/MESChw_setup.c:73:  _motor->Raw.Iu = hadc1.Instance->JDR1;  // U Current
./MESC_F405RG/Core/Src/MESChw_setup.c:74:  _motor->Raw.Iv = hadc2.Instance->JDR1;  // V Current
./MESC_F405RG/Core/Src/MESChw_setup.c:75:  _motor->Raw.Iw = hadc3.Instance->JDR1;  // W Current
./MESC_F405RG/Core/Src/MESChw_setup.c:76:  _motor->Raw.Vbus = hadc3.Instance->JDR3;  // DC Link Voltage
./MESC_F405RG/Core/Src/MESChw_setup.c:78:  _motor->Raw.Iu = 2048;  // U Current
./MESC_F405RG/Core/Src/MESChw_setup.c:81:  _motor->Raw.Iv = 2048;  // V Current
./MESC_F405RG/Core/Src/MESChw_setup.c:84:  _motor->Raw.Iw = 2048;  // W Current
./MESC_F405RG/Core/Src/MESChw_setup.c:108:	  _motor->Raw.Vu = hadc1.Instance->JDR2; //PhaseU Voltage
./MESC_F405RG/Core/Src/MESChw_setup.c:109:	  _motor->Raw.Vv = hadc2.Instance->JDR2; //PhaseV Voltage
./MESC_F405RG/Core/Src/MESChw_setup.c:110:	  _motor->Raw.Vw = hadc3.Instance->JDR2; //PhaseW Voltage
./MESC_F405RG/Core/Src/MESChw_setup.c:144:    HAL_TIM_PWM_Start(_motor->mtimer, TIM_CHANNEL_4 );
./MESC_F405RG/Core/Src/MESChw_setup.c:147:    HAL_TIM_PWM_Start(    _motor->mtimer, TIM_CHANNEL_1 );
./MESC_F405RG/Core/Src/MESChw_setup.c:148:    HAL_TIMEx_PWMN_Start( _motor->mtimer, TIM_CHANNEL_1 );
./MESC_F405RG/Core/Src/MESChw_setup.c:150:    HAL_TIM_PWM_Start(    _motor->mtimer, TIM_CHANNEL_2 );
./MESC_F405RG/Core/Src/MESChw_setup.c:151:    HAL_TIMEx_PWMN_Start( _motor->mtimer, TIM_CHANNEL_2 );
./MESC_F405RG/Core/Src/MESChw_setup.c:153:    HAL_TIM_PWM_Start(    _motor->mtimer, TIM_CHANNEL_3 );
./MESC_F405RG/Core/Src/MESChw_setup.c:154:    HAL_TIMEx_PWMN_Start( _motor->mtimer, TIM_CHANNEL_3 );
./MESC_F405RG/Core/Src/MESChw_setup.c:178:    __HAL_TIM_ENABLE_IT(_motor->mtimer, TIM_IT_UPDATE);
./MESC_F303CB/Core/Src/MESChw_setup.c:133:	  _motor->Raw.Iu = hadc1.Instance->JDR1;  // U Current
./MESC_F303CB/Core/Src/MESChw_setup.c:134:	  _motor->Raw.Iv = hadc2.Instance->JDR1;  // V Current
./MESC_F303CB/Core/Src/MESChw_setup.c:135:	  _motor->Raw.Iw = hadc3.Instance->JDR1;  // W Current
./MESC_F303CB/Core/Src/MESChw_setup.c:136:	  _motor->Raw.Vbus = hadc1.Instance->JDR2;  // DC Link Voltage
./MESC_F303CB/Core/Src/MESChw_setup.c:140:	  _motor->Raw.MOSu_T = 0.99f*_motor->Raw.MOSu_T +0.01f*hadc4.Instance->JDR1; //Temperature on PB1
./MESC_F303CB/Core/Src/MESChw_setup.c:146:	  _motor->Raw.Vu = hadc1.Instance->JDR3; //PhaseU Voltage
./MESC_F303CB/Core/Src/MESChw_setup.c:147:	  _motor->Raw.Vv = hadc2.Instance->JDR2; //PhaseV Voltage
./MESC_F303CB/Core/Src/MESChw_setup.c:148:	  _motor->Raw.Vw = hadc2.Instance->JDR3; //PhaseW Voltage
./MESC_Common/Src/MESChfi.c:42:	if(((_motor->FOC.Vdq.q-_motor->FOC.Idq_smoothed.q*_motor->m.R) > _motor->HFI.toggle_voltage)
./MESC_Common/Src/MESChfi.c:43:			||((_motor->FOC.Vdq.q-_motor->FOC.Idq_smoothed.q*_motor->m.R) < -_motor->HFI.toggle_voltage)
./MESC_Common/Src/MESChfi.c:44:			||(_motor->MotorSensorMode==MOTOR_SENSOR_MODE_HALL)
./MESC_Common/Src/MESChfi.c:45:			||((fabsf(_motor->FOC.eHz)>_motor->HFI.toggle_eHz))){
./MESC_Common/Src/MESChfi.c:46:		_motor->HFI.inject = 0;
./MESC_Common/Src/MESChfi.c:47://		_motor->FOC.Current_bandwidth = CURRENT_BANDWIDTH;
./MESC_Common/Src/MESChfi.c:48:	} else if(((_motor->FOC.Vdq.q-_motor->FOC.Idq_smoothed.q*_motor->m.R) < (_motor->HFI.toggle_voltage-1.0f))//HFI hysteresis voltage hard coded as 1.0V
./MESC_Common/Src/MESChfi.c:49:			&&((_motor->FOC.Vdq.q-_motor->FOC.Idq_smoothed.q*_motor->m.R) > -(_motor->HFI.toggle_voltage-1.0f))
./MESC_Common/Src/MESChfi.c:50:			&&(_motor->HFI.Type !=HFI_TYPE_NONE)){
./MESC_Common/Src/MESChfi.c:51:		_motor->HFI.int_err = _motor->FOC.PLL_int;
./MESC_Common/Src/MESChfi.c:52:		_motor->HFI.inject = 1;
./MESC_Common/Src/MESChfi.c:53://		_motor->FOC.Current_bandwidth = CURRENT_BANDWIDTH*0.1f;
./MESC_Common/Src/MESChfi.c:58:  if ((_motor->HFI.inject)&&(_motor->MotorState != MOTOR_STATE_TRACKING)) {
./MESC_Common/Src/MESChfi.c:60:	if (_motor->HFI.inject_high_low_now == 0){//First we create the toggle
./MESC_Common/Src/MESChfi.c:61:		_motor->HFI.inject_high_low_now = 1;
./MESC_Common/Src/MESChfi.c:62:		  Idq[0].d = _motor->FOC.Idq.d;
./MESC_Common/Src/MESChfi.c:63:		  Idq[0].q = _motor->FOC.Idq.q;
./MESC_Common/Src/MESChfi.c:65:		_motor->HFI.inject_high_low_now = 0;
./MESC_Common/Src/MESChfi.c:66:		  Idq[1].d = _motor->FOC.Idq.d;
./MESC_Common/Src/MESChfi.c:67:		  Idq[1].q = _motor->FOC.Idq.q;
./MESC_Common/Src/MESChfi.c:69:	_motor->FOC.didq.d = (Idq[0].d - Idq[1].d); //Calculate the changing current levels
./MESC_Common/Src/MESChfi.c:70:	_motor->FOC.didq.q = (Idq[0].q - Idq[1].q);
./MESC_Common/Src/MESChfi.c:72:	switch(_motor->HFI.Type){
./MESC_Common/Src/MESChfi.c:78:			if(_motor->HFI.inject_high_low_now ==1){
./MESC_Common/Src/MESChfi.c:79:				_motor->HFI.Vd_injectionV = +_motor->meas.hfi_voltage;
./MESC_Common/Src/MESChfi.c:80:				if(_motor->FOC.Idq_req.q>0.0f){
./MESC_Common/Src/MESChfi.c:82:					_motor->HFI.Vq_injectionV = +_motor->meas.hfi_voltage;
./MESC_Common/Src/MESChfi.c:84:					_motor->HFI.Vq_injectionV = -_motor->meas.hfi_voltage;
./MESC_Common/Src/MESChfi.c:88:				_motor->HFI.Vd_injectionV = -_motor->meas.hfi_voltage;
./MESC_Common/Src/MESChfi.c:89:				if(_motor->FOC.Idq_req.q>0.0f){
./MESC_Common/Src/MESChfi.c:90:					_motor->HFI.Vq_injectionV = -_motor->meas.hfi_voltage;
./MESC_Common/Src/MESChfi.c:92:					_motor->HFI.Vq_injectionV = +_motor->meas.hfi_voltage;
./MESC_Common/Src/MESChfi.c:96:			magnitude45 = sqrtf(_motor->FOC.didq.d*_motor->FOC.didq.d+_motor->FOC.didq.q*_motor->FOC.didq.q);
./MESC_Common/Src/MESChfi.c:98:			if(_motor->FOC.was_last_tracking==0){
./MESC_Common/Src/MESChfi.c:102:				error = _motor->HFI.Gain*(magnitude45-_motor->HFI.mod_didq);
./MESC_Common/Src/MESChfi.c:105:				_motor->HFI.int_err = _motor->HFI.int_err +0.05f*error;
./MESC_Common/Src/MESChfi.c:106:				if(_motor->HFI.int_err>1000.0f){_motor->HFI.int_err = 1000.0f;}
./MESC_Common/Src/MESChfi.c:107:				if(_motor->HFI.int_err<-1000.0f){_motor->HFI.int_err = -1000.0f;}
./MESC_Common/Src/MESChfi.c:108:				_motor->FOC.FOCAngle = _motor->FOC.FOCAngle + (int)(error + _motor->HFI.int_err)*Idqreq_dir;
./MESC_Common/Src/MESChfi.c:111:				_motor->FOC.FOCAngle += _motor->HFI.test_increment;
./MESC_Common/Src/MESChfi.c:112:				_motor->HFI.accu += magnitude45;
./MESC_Common/Src/MESChfi.c:113:				_motor->HFI.count += 1;
./MESC_Common/Src/MESChfi.c:116:							_motor->FOC.FOCAngle = 62000;
./MESC_Common/Src/MESChfi.c:121:			if(_motor->HFI.inject_high_low_now ==1){
./MESC_Common/Src/MESChfi.c:122:			  _motor->HFI.Vd_injectionV = +_motor->meas.hfi_voltage;
./MESC_Common/Src/MESChfi.c:124:			  _motor->HFI.Vd_injectionV = -_motor->meas.hfi_voltage;
./MESC_Common/Src/MESChfi.c:126:			if(_motor->FOC.didq.q>1.0f){_motor->FOC.didq.q = 1.0f;}
./MESC_Common/Src/MESChfi.c:127:			if(_motor->FOC.didq.q<-1.0f){_motor->FOC.didq.q = -1.0f;}
./MESC_Common/Src/MESChfi.c:128:			intdidq.q = (intdidq.q + 0.1f*_motor->FOC.didq.q);
./MESC_Common/Src/MESChfi.c:131:			_motor->FOC.FOCAngle += (int)(250.0f*_motor->FOC.IIR[1] + 10.50f*intdidq.q)*_motor->FOC.d_polarity;
./MESC_Common/Src/MESChfi.c:135:			if(_motor->HFI.inject_high_low_now ==1){
./MESC_Common/Src/MESChfi.c:136:			  _motor->HFI.Vd_injectionV = _motor->HFI.special_injectionVd;
./MESC_Common/Src/MESChfi.c:137:			  _motor->HFI.Vq_injectionV = _motor->HFI.special_injectionVq;
./MESC_Common/Src/MESChfi.c:139:				_motor->HFI.Vd_injectionV = -_motor->HFI.special_injectionVd;
./MESC_Common/Src/MESChfi.c:140:				_motor->HFI.Vq_injectionV = -_motor->HFI.special_injectionVq;			}
./MESC_Common/Src/MESChfi.c:144:	  _motor->HFI.Vd_injectionV = 0.0f;
./MESC_Common/Src/MESChfi.c:145:	  _motor->HFI.Vq_injectionV = 0.0f;
./MESC_Common/Src/MESChfi.c:151:		switch(_motor->HFI.Type){
./MESC_Common/Src/MESChfi.c:154:				if(_motor->HFI.inject==1){
./MESC_Common/Src/MESChfi.c:156:					if(_motor->FOC.was_last_tracking==1){
./MESC_Common/Src/MESChfi.c:157:						if(_motor->HFI.countdown>1){
./MESC_Common/Src/MESChfi.c:158:							_motor->HFI.mod_didq = _motor->HFI.accu / _motor->HFI.count;
./MESC_Common/Src/MESChfi.c:159:							_motor->HFI.Gain = 5000.0f/_motor->HFI.mod_didq;
./MESC_Common/Src/MESChfi.c:160:							_motor->FOC.was_last_tracking = 0;
./MESC_Common/Src/MESChfi.c:162:							_motor->HFI.test_increment = 65536 * SLOW_LOOP_FREQUENCY / _motor->FOC.pwm_frequency;
./MESC_Common/Src/MESChfi.c:163:							_motor->HFI.countdown++;
./MESC_Common/Src/MESChfi.c:166:						_motor->HFI.countdown = 0;
./MESC_Common/Src/MESChfi.c:167:						_motor->HFI.count = 0;
./MESC_Common/Src/MESChfi.c:168:						_motor->HFI.accu = 0.0f;
./MESC_Common/Src/MESChfi.c:175:				if(_motor->HFI.inject==1){
./MESC_Common/Src/MESChfi.c:176:					if(_motor->HFI.countdown==3){
./MESC_Common/Src/MESChfi.c:177:						_motor->FOC.Idq_req.d = HFI_TEST_CURRENT;
./MESC_Common/Src/MESChfi.c:178:						_motor->FOC.Idq_req.q=0.0f;//Override the inputs to set Q current to zero
./MESC_Common/Src/MESChfi.c:179:					}else if(_motor->HFI.countdown==2){
./MESC_Common/Src/MESChfi.c:180:						_motor->FOC.Ldq_now_dboost[0] = _motor->FOC.IIR[0]; //Find the effect of d-axis current
./MESC_Common/Src/MESChfi.c:181:						_motor->FOC.Idq_req.d = 1.0f;
./MESC_Common/Src/MESChfi.c:182:						_motor->FOC.Idq_req.q=0.0f;
./MESC_Common/Src/MESChfi.c:183:					}else if(_motor->HFI.countdown == 1){
./MESC_Common/Src/MESChfi.c:184:						_motor->FOC.Idq_req.d = -HFI_TEST_CURRENT;
./MESC_Common/Src/MESChfi.c:185:						_motor->FOC.Idq_req.q=0.0f;
./MESC_Common/Src/MESChfi.c:186:					}else if(_motor->HFI.countdown == 0){
./MESC_Common/Src/MESChfi.c:187:						_motor->FOC.Ldq_now[0] = _motor->FOC.IIR[0];//_motor->HFI.Vd_injectionV;
./MESC_Common/Src/MESChfi.c:188:						_motor->FOC.Idq_req.d = 0.0f;
./MESC_Common/Src/MESChfi.c:189:					if(_motor->FOC.Ldq_now[0]>_motor->FOC.Ldq_now_dboost[0]){_motor->FOC.FOCAngle+=32768;}
./MESC_Common/Src/MESChfi.c:190:					_motor->HFI.countdown = 200;
./MESC_Common/Src/MESChfi.c:192:					_motor->HFI.countdown--;
./MESC_Common/Src/MESChfi.c:197:				_motor->HFI.inject = 0;
./MESC_Common/Src/MESChfi.c:198://				_motor->FOC.Current_bandwidth = CURRENT_BANDWIDTH;
./MESC_Common/Src/MESCApp.c:88:	  _motor->FOC.Idq_prereq.q = 	_motor->input_vars.UART_req +
./MESC_Common/Src/MESCApp.c:89:	  	  	  	  	  	  	  	  	_motor->input_vars.max_request_Idq.q * (_motor->input_vars.ADC1_req + _motor->input_vars.ADC2_req +
./MESC_Common/Src/MESCApp.c:90:	  	  	  	  	  	  	  	  	_motor->input_vars.RCPWM_req + _motor->input_vars.ADC12_diff_req +
./MESC_Common/Src/MESCApp.c:91:									_motor->input_vars.remote_ADC1_req + _motor->input_vars.remote_ADC2_req );
./MESC_Common/Src/MESCApp.c:94:	  _motor->FOC.Idq_prereq.q = clamp(_motor->FOC.Idq_prereq.q, _motor->input_vars.min_request_Idq.q, _motor->input_vars.max_request_Idq.q);
./MESC_Common/Src/MESCApp.c:100:	  _motor->FOC.Idq_prereq.q = 	_motor->input_vars.UART_req +
./MESC_Common/Src/MESCApp.c:101:	  	  	  	  	  	  	  	  	_motor->input_vars.max_request_Idq.q * (_motor->input_vars.ADC1_req + _motor->input_vars.ADC2_req +
./MESC_Common/Src/MESCApp.c:102:	  	  	  	  	  	  	  	  	_motor->input_vars.RCPWM_req + _motor->input_vars.ADC12_diff_req +
./MESC_Common/Src/MESCApp.c:103:									_motor->input_vars.remote_ADC1_req + _motor->input_vars.remote_ADC2_req );
./MESC_Common/Src/MESCApp.c:106:	  _motor->FOC.Idq_prereq.q = clamp(_motor->FOC.Idq_prereq.q, _motor->input_vars.min_request_Idq.q, _motor->input_vars.max_request_Idq.q);
./MESC_Common/Src/MESCApp.c:111:	  if((!vehicle.reverseState)&&(fabsf(_motor->FOC.Idq_prereq.q)<5.0f)){
./MESC_Common/Src/MESCApp.c:118:		  _motor->FOC.Idq_prereq.q *= -1.0f;
./MESC_Common/Src/MESCApp.c:123:		  _motor->FOC.Idq_prereq.q *= -1.0f;
./MESC_Common/Src/MESCApp.c:131:		_motor->key_bits |= APP_KEY; //Set the bit; lock the motor from current
./MESC_Common/Src/MESCApp.c:137:		_motor->key_bits |= APP_KEY; //Set the bit; lock the motor from current
./MESC_Common/Src/MESCApp.c:142:		_motor->key_bits &= ~APP_KEY; //Clear the bit; allow motor current
./MESC_Common/Src/MESCApp.c:152:		_motor->key_bits |= APP_KEY; //Set the bit; lock the motor from current
./MESC_Common/Src/MESCApp.c:155://	_motor->key_bits;
./MESC_Common/Src/MESCApp.c:157:// 	_motor->key_bits |= APP_KEY; Set the bit; lock the motor from current
./MESC_Common/Src/MESCApp.c:158://	_motor->key_bits &= ~APP_KEY; Clear the bit - allow current
./MESC_Common/Src/MESCpwm.c:58:	if (_motor->mtimer->Instance->CR1&0x16) {//Polling the DIR (direction) bit on the motor counter DIR = 1 = downcounting
./MESC_Common/Src/MESCpwm.c:61:	if (!(_motor->mtimer->Instance->CR1&0x16)) {//Polling the DIR (direction) bit on the motor counter DIR = 0 = upcounting
./MESC_Common/Src/MESCpwm.c:65:	_motor->FOC.cycles_pwmloop = CPU_CYCLES - cycles;
./MESC_Common/Src/MESCpwm.c:79:	Vd = _motor->FOC.Vdq.d + _motor->HFI.Vd_injectionV;
./MESC_Common/Src/MESCpwm.c:80:	Vq = _motor->FOC.Vdq.q + _motor->HFI.Vq_injectionV;
./MESC_Common/Src/MESCpwm.c:86:if((fabsf(_motor->FOC.eHz)>0.005f*_motor->FOC.pwm_frequency)&&(_motor->HFI.inject==0)){
./MESC_Common/Src/MESCpwm.c:91:	_motor->FOC.FOCAngle = _motor->FOC.FOCAngle + 0.5f*_motor->FOC.PLL_int;
./MESC_Common/Src/MESCpwm.c:94:	sin_cos_fast(_motor->FOC.FOCAngle, &_motor->FOC.sincosangle.sin, &_motor->FOC.sincosangle.cos);
./MESC_Common/Src/MESCpwm.c:97:    _motor->FOC.Vab.a = _motor->FOC.sincosangle.cos * Vd -
./MESC_Common/Src/MESCpwm.c:98:                      _motor->FOC.sincosangle.sin * Vq;
./MESC_Common/Src/MESCpwm.c:99:    _motor->FOC.Vab.b = _motor->FOC.sincosangle.sin * Vd +
./MESC_Common/Src/MESCpwm.c:100:                      _motor->FOC.sincosangle.cos * Vq;
./MESC_Common/Src/MESCpwm.c:103:    _motor->mtimer->Instance->CCR1 = (uint16_t)(1.0f * _motor->FOC.Vab_to_PWM * (_motor->FOC.Vab.a) + _motor->FOC.PWMmid);
./MESC_Common/Src/MESCpwm.c:104:    _motor->mtimer->Instance->CCR2 = (uint16_t)(-1.0f * _motor->FOC.Vab_to_PWM * (_motor->FOC.Vab.a) + _motor->FOC.PWMmid);
./MESC_Common/Src/MESCpwm.c:105:    _motor->mtimer->Instance->CCR3 = (uint16_t)(1.0f * _motor->FOC.Vab_to_PWM * (_motor->FOC.Vab.b) + _motor->FOC.PWMmid);
./MESC_Common/Src/MESCpwm.c:106:    _motor->mtimer->Instance->CCR4 = (uint16_t)(-1.0f * _motor->FOC.Vab_to_PWM * (_motor->FOC.Vab.b) + _motor->FOC.PWMmid);
./MESC_Common/Src/MESCpwm.c:111:	_motor->FOC.inverterVoltage[0] = _motor->FOC.Vab.a;
./MESC_Common/Src/MESCpwm.c:112:	_motor->FOC.inverterVoltage[1] = -0.5f*_motor->FOC.inverterVoltage[0];
./MESC_Common/Src/MESCpwm.c:113:	_motor->FOC.inverterVoltage[2] = _motor->FOC.inverterVoltage[1] - sqrt3_on_2 * _motor->FOC.Vab.b;
./MESC_Common/Src/MESCpwm.c:114:	_motor->FOC.inverterVoltage[1] = _motor->FOC.inverterVoltage[1] + sqrt3_on_2 * _motor->FOC.Vab.b;
./MESC_Common/Src/MESCpwm.c:120:    top_value = _motor->FOC.inverterVoltage[0];
./MESC_Common/Src/MESCpwm.c:122:    _motor->HighPhase = U;
./MESC_Common/Src/MESCpwm.c:124:    if (_motor->FOC.inverterVoltage[1] > top_value) {
./MESC_Common/Src/MESCpwm.c:125:      top_value = _motor->FOC.inverterVoltage[1];
./MESC_Common/Src/MESCpwm.c:126:      _motor->HighPhase = V;
./MESC_Common/Src/MESCpwm.c:128:    if (_motor->FOC.inverterVoltage[2] > top_value) {
./MESC_Common/Src/MESCpwm.c:129:      top_value = _motor->FOC.inverterVoltage[2];
./MESC_Common/Src/MESCpwm.c:130:      _motor->HighPhase = W;
./MESC_Common/Src/MESCpwm.c:132:    if (_motor->FOC.inverterVoltage[1] < bottom_value) {
./MESC_Common/Src/MESCpwm.c:133:      bottom_value = _motor->FOC.inverterVoltage[1];
./MESC_Common/Src/MESCpwm.c:135:    if (_motor->FOC.inverterVoltage[2] < bottom_value) {
./MESC_Common/Src/MESCpwm.c:136:      bottom_value = _motor->FOC.inverterVoltage[2];
./MESC_Common/Src/MESCpwm.c:138:    if(_motor->FOC.Voltage < _motor->FOC.V_3Q_mag_max){
./MESC_Common/Src/MESCpwm.c:139:        _motor->HighPhase = N; //Trigger the full clark transform
./MESC_Common/Src/MESCpwm.c:142:    switch(_motor->options.pwm_type){
./MESC_Common/Src/MESCpwm.c:144:    	   mid_value = _motor->FOC.PWMmid -
./MESC_Common/Src/MESCpwm.c:145:    	                0.5f * _motor->FOC.Vab_to_PWM * (top_value + bottom_value);
./MESC_Common/Src/MESCpwm.c:149:    	    _motor->mtimer->Instance->CCR1 =
./MESC_Common/Src/MESCpwm.c:150:    	    		(uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[0] + mid_value);
./MESC_Common/Src/MESCpwm.c:151:    	    _motor->mtimer->Instance->CCR2 =
./MESC_Common/Src/MESCpwm.c:152:    	    		(uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[1] + mid_value);
./MESC_Common/Src/MESCpwm.c:153:    	    _motor->mtimer->Instance->CCR3 =
./MESC_Common/Src/MESCpwm.c:154:    	    		(uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[2] + mid_value);
./MESC_Common/Src/MESCpwm.c:172:    	    if(_motor->Conv.Iu < -0.030f){_motor->mtimer->Instance->CCR1 = _motor->mtimer->Instance->CCR1-_motor->FOC.deadtime_comp;}
./MESC_Common/Src/MESCpwm.c:173:    	    if(_motor->Conv.Iv < -0.030f){_motor->mtimer->Instance->CCR2 = _motor->mtimer->Instance->CCR2-_motor->FOC.deadtime_comp;}
./MESC_Common/Src/MESCpwm.c:174:    	    if(_motor->Conv.Iw < -0.030f){_motor->mtimer->Instance->CCR3 = _motor->mtimer->Instance->CCR3-_motor->FOC.deadtime_comp;}
./MESC_Common/Src/MESCpwm.c:175:    	    if(_motor->Conv.Iu > -0.030f){_motor->mtimer->Instance->CCR1 = _motor->mtimer->Instance->CCR1+_motor->FOC.deadtime_comp;}
./MESC_Common/Src/MESCpwm.c:176:    	    if(_motor->Conv.Iv > -0.030f){_motor->mtimer->Instance->CCR2 = _motor->mtimer->Instance->CCR2+_motor->FOC.deadtime_comp;}
./MESC_Common/Src/MESCpwm.c:177:    	    if(_motor->Conv.Iw > -0.030f){_motor->mtimer->Instance->CCR3 = _motor->mtimer->Instance->CCR3+_motor->FOC.deadtime_comp;}
./MESC_Common/Src/MESCpwm.c:189:    	    if(_motor->FOC.Voltage < _motor->FOC.V_3Q_mag_max){//Sinusoidal
./MESC_Common/Src/MESCpwm.c:190:    	    	   mid_value = _motor->FOC.PWMmid -
./MESC_Common/Src/MESCpwm.c:191:    	    	                0.5f * _motor->FOC.Vab_to_PWM * (top_value + bottom_value);
./MESC_Common/Src/MESCpwm.c:195:    	    	    _motor->mtimer->Instance->CCR1 =
./MESC_Common/Src/MESCpwm.c:196:    	    	    		(uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[0] + mid_value);
./MESC_Common/Src/MESCpwm.c:197:    	    	    _motor->mtimer->Instance->CCR2 =
./MESC_Common/Src/MESCpwm.c:198:    	    	    		(uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[1] + mid_value);
./MESC_Common/Src/MESCpwm.c:199:    	    	    _motor->mtimer->Instance->CCR3 =
./MESC_Common/Src/MESCpwm.c:200:    	    	    		(uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[2] + mid_value);
./MESC_Common/Src/MESCpwm.c:202://    			_motor->FOC.inverterVoltage[0] = _motor->FOC.inverterVoltage[0]+ mid_value;//0.5*_motor->FOC.Vmag_max;
./MESC_Common/Src/MESCpwm.c:203://    			_motor->FOC.inverterVoltage[1] = _motor->FOC.inverterVoltage[1]+ mid_value;//0.5*_motor->FOC.Vmag_max;
./MESC_Common/Src/MESCpwm.c:204://    			_motor->FOC.inverterVoltage[2] = _motor->FOC.inverterVoltage[2]+ mid_value;//0.5*_motor->FOC.Vmag_max;
./MESC_Common/Src/MESCpwm.c:206:    			_motor->FOC.inverterVoltage[0] = _motor->FOC.inverterVoltage[0]-bottom_value;
./MESC_Common/Src/MESCpwm.c:207:    			_motor->FOC.inverterVoltage[1] = _motor->FOC.inverterVoltage[1]-bottom_value;
./MESC_Common/Src/MESCpwm.c:208:    			_motor->FOC.inverterVoltage[2] = _motor->FOC.inverterVoltage[2]-bottom_value;
./MESC_Common/Src/MESCpwm.c:211:				_motor->mtimer->Instance->CCR1 = (uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[0]);
./MESC_Common/Src/MESCpwm.c:212:				_motor->mtimer->Instance->CCR2 = (uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[1]);
./MESC_Common/Src/MESCpwm.c:213:				_motor->mtimer->Instance->CCR3 = (uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[2]);
./MESC_Common/Src/MESCpwm.c:221://    		_motor->mtimer->Instance->CCR1 = 	_motor->mtimer->Instance->CCR1 - carryU;
./MESC_Common/Src/MESCpwm.c:222://    		_motor->mtimer->Instance->CCR2 = 	_motor->mtimer->Instance->CCR2 - carryV;
./MESC_Common/Src/MESCpwm.c:223://    		_motor->mtimer->Instance->CCR3 = 	_motor->mtimer->Instance->CCR3 - carryW;
./MESC_Common/Src/MESCpwm.c:228://    		if(_motor->mtimer->Instance->CCR1>(_motor->mtimer->Instance->ARR-OVERMOD_DT_COMP_THRESHOLD)){
./MESC_Common/Src/MESCpwm.c:229://    			carryU = _motor->mtimer->Instance->ARR-_motor->mtimer->Instance->CCR1; //Save the amount we have overmodulated by
./MESC_Common/Src/MESCpwm.c:230://    			_motor->mtimer->Instance->CCR1 = _motor->mtimer->Instance->ARR;
./MESC_Common/Src/MESCpwm.c:232://    		if(_motor->mtimer->Instance->CCR2>(_motor->mtimer->Instance->ARR-OVERMOD_DT_COMP_THRESHOLD)){
./MESC_Common/Src/MESCpwm.c:233://    			carryV = _motor->mtimer->Instance->ARR-_motor->mtimer->Instance->CCR2; //Save the amount we have overmodulated by
./MESC_Common/Src/MESCpwm.c:234://    			_motor->mtimer->Instance->CCR2 = _motor->mtimer->Instance->ARR;
./MESC_Common/Src/MESCpwm.c:236://    		if(_motor->mtimer->Instance->CCR3>(_motor->mtimer->Instance->ARR-OVERMOD_DT_COMP_THRESHOLD)){
./MESC_Common/Src/MESCpwm.c:237://    			carryW = _motor->mtimer->Instance->ARR-_motor->mtimer->Instance->CCR3; //Save the amount we have overmodulated by
./MESC_Common/Src/MESCpwm.c:238://    			_motor->mtimer->Instance->CCR3 = _motor->mtimer->Instance->ARR;
./MESC_Common/Src/MESCpwm.c:294:   tmpccmrx = _motor->mtimer->Instance->CCMR1;
./MESC_Common/Src/MESCpwm.c:298:   _motor->mtimer->Instance->CCMR1 = tmpccmrx;
./MESC_Common/Src/MESCpwm.c:299:   _motor->mtimer->Instance->CCER &= ~TIM_CCER_CC1E;   // disable
./MESC_Common/Src/MESCpwm.c:300:   _motor->mtimer->Instance->CCER &= ~TIM_CCER_CC1NE;  // disable
./MESC_Common/Src/MESCpwm.c:304:   tmpccmrx = _motor->mtimer->Instance->CCMR1;
./MESC_Common/Src/MESCpwm.c:308:   _motor->mtimer->Instance->CCMR1 = tmpccmrx;
./MESC_Common/Src/MESCpwm.c:309:   _motor->mtimer->Instance->CCER |= TIM_CCER_CC1E;   // enable
./MESC_Common/Src/MESCpwm.c:310:   _motor->mtimer->Instance->CCER |= TIM_CCER_CC1NE;  // enable
./MESC_Common/Src/MESCpwm.c:314:   tmpccmrx = _motor->mtimer->Instance->CCMR1;
./MESC_Common/Src/MESCpwm.c:318:   _motor->mtimer->Instance->CCMR1 = tmpccmrx;
./MESC_Common/Src/MESCpwm.c:319:   _motor->mtimer->Instance->CCER &= ~TIM_CCER_CC2E;   // disable
./MESC_Common/Src/MESCpwm.c:320:   _motor->mtimer->Instance->CCER &= ~TIM_CCER_CC2NE;  // disable
./MESC_Common/Src/MESCpwm.c:324:   tmpccmrx = _motor->mtimer->Instance->CCMR1;
./MESC_Common/Src/MESCpwm.c:328:   _motor->mtimer->Instance->CCMR1 = tmpccmrx;
./MESC_Common/Src/MESCpwm.c:329:   _motor->mtimer->Instance->CCER |= TIM_CCER_CC2E;   // enable
./MESC_Common/Src/MESCpwm.c:330:   _motor->mtimer->Instance->CCER |= TIM_CCER_CC2NE;  // enable
./MESC_Common/Src/MESCpwm.c:334:   tmpccmrx = _motor->mtimer->Instance->CCMR2;
./MESC_Common/Src/MESCpwm.c:338:   _motor->mtimer->Instance->CCMR2 = tmpccmrx;
./MESC_Common/Src/MESCpwm.c:339:   _motor->mtimer->Instance->CCER &= ~TIM_CCER_CC3E;   // disable
./MESC_Common/Src/MESCpwm.c:340:   _motor->mtimer->Instance->CCER &= ~TIM_CCER_CC3NE;  // disable
./MESC_Common/Src/MESCpwm.c:344:   tmpccmrx = _motor->mtimer->Instance->CCMR2;
./MESC_Common/Src/MESCpwm.c:348:   _motor->mtimer->Instance->CCMR2 = tmpccmrx;
./MESC_Common/Src/MESCpwm.c:349:   _motor->mtimer->Instance->CCER |= TIM_CCER_CC3E;   // enable
./MESC_Common/Src/MESCpwm.c:350:   _motor->mtimer->Instance->CCER |= TIM_CCER_CC3NE;  // enable
./MESC_Common/Src/MESCfoc.c:137:	_motor->safe_start[0] = SAFE_START_DEFAULT;
./MESC_Common/Src/MESCfoc.c:140:	_motor->MotorState = MOTOR_STATE_IDLE;
./MESC_Common/Src/MESCfoc.c:146:	_motor->offset.Iu = ADC_OFFSET_DEFAULT;
./MESC_Common/Src/MESCfoc.c:147:	_motor->offset.Iv = ADC_OFFSET_DEFAULT;
./MESC_Common/Src/MESCfoc.c:148:	_motor->offset.Iw = ADC_OFFSET_DEFAULT;
./MESC_Common/Src/MESCfoc.c:150:	_motor->FOC.deadtime_comp = DEADTIME_COMP_V;
./MESC_Common/Src/MESCfoc.c:152:	_motor->MotorState = MOTOR_STATE_INITIALISING;
./MESC_Common/Src/MESCfoc.c:155:	_motor->MotorControlType = MOTOR_CONTROL_TYPE_FOC;
./MESC_Common/Src/MESCfoc.c:156:	_motor->ControlMode = DEFAULT_CONTROL_MODE;
./MESC_Common/Src/MESCfoc.c:158:	_motor->MotorSensorMode = DEFAULT_SENSOR_MODE;
./MESC_Common/Src/MESCfoc.c:159:	_motor->SLStartupSensor = DEFAULT_STARTUP_SENSOR;
./MESC_Common/Src/MESCfoc.c:160:	_motor->HFI.Type = DEFAULT_HFI_TYPE;
./MESC_Common/Src/MESCfoc.c:161:	if(_motor->SLStartupSensor != STARTUP_SENSOR_HFI){_motor->HFI.Type = HFI_TYPE_NONE;}
./MESC_Common/Src/MESCfoc.c:162:	_motor->meas.hfi_voltage = HFI_VOLTAGE;
./MESC_Common/Src/MESCfoc.c:164:	_motor->meas.measure_current = I_MEASURE;
./MESC_Common/Src/MESCfoc.c:165:	_motor->meas.measure_voltage = V_MEASURE;
./MESC_Common/Src/MESCfoc.c:166:	_motor->meas.measure_closedloop_current = I_MEASURE_CLOSEDLOOP;
./MESC_Common/Src/MESCfoc.c:167:	_motor->FOC.pwm_frequency =PWM_FREQUENCY;
./MESC_Common/Src/MESCfoc.c:171:	_motor->hall.dir = 1.0f;
./MESC_Common/Src/MESCfoc.c:172:	_motor->hall.ticks_since_last_observer_change = 65535.0f;
./MESC_Common/Src/MESCfoc.c:173:	_motor->hall.last_observer_period = 65536.0f;
./MESC_Common/Src/MESCfoc.c:174:	_motor->hall.one_on_last_observer_period = 1.0f;
./MESC_Common/Src/MESCfoc.c:175:	_motor->hall.angular_velocity = 0.0f;
./MESC_Common/Src/MESCfoc.c:176:	_motor->hall.angle_step = 0.0f;
./MESC_Common/Src/MESCfoc.c:181:	_motor->options.use_hall_start = true;
./MESC_Common/Src/MESCfoc.c:183:	_motor->options.use_hall_start = false;
./MESC_Common/Src/MESCfoc.c:185:    _motor->FOC.hall_IIR = HALL_IIR; //decay constant for the hall start preload
./MESC_Common/Src/MESCfoc.c:186:    _motor->FOC.hall_IIR = HALL_IIRN; //decay constant for the hall start preload
./MESC_Common/Src/MESCfoc.c:187:    _motor->FOC.hall_transition_V = HALL_VOLTAGE_THRESHOLD; //transition voltage above which the hall sensors are not doing any preloading
./MESC_Common/Src/MESCfoc.c:191:	_motor->options.use_lr_observer = true;
./MESC_Common/Src/MESCfoc.c:193:	_motor->options.use_lr_observer = false;
./MESC_Common/Src/MESCfoc.c:197:	_motor->options.MTPA_mode = MTPA_MAG;
./MESC_Common/Src/MESCfoc.c:199:	_motor->options.MTPA_mode = MTPA_NONE;
./MESC_Common/Src/MESCfoc.c:203:	_motor->options.use_phase_balancing = true;
./MESC_Common/Src/MESCfoc.c:205:	_motor->options.use_phase_balancing = false;
./MESC_Common/Src/MESCfoc.c:208:	_motor->options.field_weakening = FIELD_WEAKENING_OFF;
./MESC_Common/Src/MESCfoc.c:210:	_motor->options.field_weakening = FIELD_WEAKENING_V1;
./MESC_Common/Src/MESCfoc.c:214:	_motor->options.field_weakening = FIELD_WEAKENING_V2;
./MESC_Common/Src/MESCfoc.c:217:	_motor->options.observer_type = MXLEMMING_LAMBDA;
./MESC_Common/Src/MESCfoc.c:219:	_motor->options.field_weakening = ORTEGA_ORIGINAL;
./MESC_Common/Src/MESCfoc.c:222:	_motor->options.sqrt_circle_lim = SQRT_CIRCLE_LIM_OFF;
./MESC_Common/Src/MESCfoc.c:224:	_motor->options.sqrt_circle_lim = SQRT_CIRCLE_LIM_ON;
./MESC_Common/Src/MESCfoc.c:228:	_motor->options.sqrt_circle_lim = SQRT_CIRCLE_LIM_VD;
./MESC_Common/Src/MESCfoc.c:231:	_motor->options.pwm_type = PWM_SVPWM;//Default to combined bottom clamp sinusoidal combinationPWM
./MESC_Common/Src/MESCfoc.c:232:	_motor->FOC.Modulation_max = MAX_MODULATION;
./MESC_Common/Src/MESCfoc.c:234:	_motor->options.pwm_type = PWM_SIN_BOTTOM;
./MESC_Common/Src/MESCfoc.c:237:	_motor->options.app_type = APP_NONE;//Default to no app
./MESC_Common/Src/MESCfoc.c:239:	_motor->options.app_type = APP_VEHICLE;
./MESC_Common/Src/MESCfoc.c:244:	_motor->FOC.enc_offset = ENCODER_E_OFFSET;
./MESC_Common/Src/MESCfoc.c:245:	_motor->FOC.encoder_polarity_invert = DEFAULT_ENCODER_POLARITY;
./MESC_Common/Src/MESCfoc.c:246:	_motor->FOC.enc_period_count = 1; //Avoid /0s
./MESC_Common/Src/MESCfoc.c:249:	_motor->m.enc_counts = 4096;//Default to this, common for many motors. Avoid div0.
./MESC_Common/Src/MESCfoc.c:250:	_motor->FOC.enc_ratio = 65536/_motor->m.enc_counts;
./MESC_Common/Src/MESCfoc.c:253:	_motor->hall.hall_error = 0;
./MESC_Common/Src/MESCfoc.c:255:	_motor->BLDC.com_flux = _motor->m.flux_linkage*1.65f;//0.02f;
./MESC_Common/Src/MESCfoc.c:256:	_motor->BLDC.direction = -1;
./MESC_Common/Src/MESCfoc.c:259:	_motor->FOC.speed_kp = DEFAULT_SPEED_KP; //0.01 = 10A/1000eHz
./MESC_Common/Src/MESCfoc.c:260:	_motor->FOC.speed_ki = DEFAULT_SPEED_KI; //Trickier to set since we want this to be proportional to the ramp speed? Not intuitive? Try 0.1; ramp in 1/10 of a second @100Hz.
./MESC_Common/Src/MESCfoc.c:262:	_motor->FOC.Duty_scaler = 1.0f; //We want this to be 1.0f for everything except duty control mode.
./MESC_Common/Src/MESCfoc.c:264:	_motor->FOC.PLL_kp = PLL_KP;
./MESC_Common/Src/MESCfoc.c:265:	_motor->FOC.PLL_ki = PLL_KI;
./MESC_Common/Src/MESCfoc.c:267:	_motor->pos.Kp = POS_KP;
./MESC_Common/Src/MESCfoc.c:268:	_motor->pos.Ki = POS_KI;
./MESC_Common/Src/MESCfoc.c:269:	_motor->pos.Kd = POS_KD;
./MESC_Common/Src/MESCfoc.c:272:	_motor->FOC.BEMF_kp = -0.25;
./MESC_Common/Src/MESCfoc.c:273:	_motor->FOC.BEMF_ki = 0.001;
./MESC_Common/Src/MESCfoc.c:276:	_motor->Raw.MOS_temp.V                  = 3.3f;
./MESC_Common/Src/MESCfoc.c:277:	_motor->Raw.MOS_temp.R_F                = MESC_TEMP_MOS_R_F;
./MESC_Common/Src/MESCfoc.c:278:	_motor->Raw.MOS_temp.adc_range          = 4096;
./MESC_Common/Src/MESCfoc.c:279:	_motor->Raw.MOS_temp.method             = MESC_TEMP_MOS_METHOD;
./MESC_Common/Src/MESCfoc.c:280:	_motor->Raw.MOS_temp.schema             = MESC_TEMP_MOS_SCHEMA;
./MESC_Common/Src/MESCfoc.c:281:	_motor->Raw.MOS_temp.parameters.SH.Beta = MESC_TEMP_MOS_SH_BETA;
./MESC_Common/Src/MESCfoc.c:282:	_motor->Raw.MOS_temp.parameters.SH.r    = MESC_TEMP_MOS_SH_R;
./MESC_Common/Src/MESCfoc.c:283:	_motor->Raw.MOS_temp.parameters.SH.T0   = CVT_CELSIUS_TO_KELVIN_F( 25.0f );
./MESC_Common/Src/MESCfoc.c:284:	_motor->Raw.MOS_temp.parameters.SH.R0   = MESC_TEMP_MOS_SH_R0;
./MESC_Common/Src/MESCfoc.c:285:	_motor->Raw.MOS_temp.limit.Tmin         = CVT_CELSIUS_TO_KELVIN_F( -15.0f );
./MESC_Common/Src/MESCfoc.c:286:	_motor->Raw.MOS_temp.limit.Thot         = CVT_CELSIUS_TO_KELVIN_F(  80.0f );
./MESC_Common/Src/MESCfoc.c:287:	_motor->Raw.MOS_temp.limit.Tmax         = CVT_CELSIUS_TO_KELVIN_F( 100.0f );
./MESC_Common/Src/MESCfoc.c:289:	_motor->Raw.Motor_temp.V                  = 3.3f;
./MESC_Common/Src/MESCfoc.c:290:	_motor->Raw.Motor_temp.R_F                = MESC_TEMP_MOTOR_R_F;
./MESC_Common/Src/MESCfoc.c:291:	_motor->Raw.Motor_temp.adc_range          = 4096;
./MESC_Common/Src/MESCfoc.c:292:	_motor->Raw.Motor_temp.method             = MESC_TEMP_MOTOR_METHOD;
./MESC_Common/Src/MESCfoc.c:293:	_motor->Raw.Motor_temp.schema             = MESC_TEMP_MOTOR_SCHEMA;
./MESC_Common/Src/MESCfoc.c:294:	_motor->Raw.Motor_temp.parameters.SH.Beta = MESC_TEMP_MOTOR_SH_BETA;
./MESC_Common/Src/MESCfoc.c:295:	_motor->Raw.Motor_temp.parameters.SH.r    = MESC_TEMP_MOTOR_SH_R;
./MESC_Common/Src/MESCfoc.c:296:	_motor->Raw.Motor_temp.parameters.SH.T0   = CVT_CELSIUS_TO_KELVIN_F( 25.0f );
./MESC_Common/Src/MESCfoc.c:297:	_motor->Raw.Motor_temp.parameters.SH.R0   = MESC_TEMP_MOTOR_SH_R0;
./MESC_Common/Src/MESCfoc.c:298:	_motor->Raw.Motor_temp.limit.Tmin         = CVT_CELSIUS_TO_KELVIN_F( -15.0f );
./MESC_Common/Src/MESCfoc.c:299:	_motor->Raw.Motor_temp.limit.Thot         = CVT_CELSIUS_TO_KELVIN_F(  80.0f );
./MESC_Common/Src/MESCfoc.c:300:	_motor->Raw.Motor_temp.limit.Tmax         = CVT_CELSIUS_TO_KELVIN_F( 100.0f );
./MESC_Common/Src/MESCfoc.c:304:    _motor->FOC.FW_curr_max = FIELD_WEAKENING_CURRENT;  // test number, to be stored in user settings
./MESC_Common/Src/MESCfoc.c:307:    _motor->FOC.Current_bandwidth = CURRENT_BANDWIDTH;
./MESC_Common/Src/MESCfoc.c:309:    _motor->FOC.ortega_gain = 1000000.0f;
./MESC_Common/Src/MESCfoc.c:345:	_motor->key_bits = UNINITIALISED_KEY + KILLSWITCH_KEY + SAFESTART_KEY;
./MESC_Common/Src/MESCfoc.c:347:while(_motor->MotorState == MOTOR_STATE_INITIALISING){
./MESC_Common/Src/MESCfoc.c:355:  _motor->logging.lognow = 1;
./MESC_Common/Src/MESCfoc.c:359:  _motor->FOC.enc_offset = ENCODER_E_OFFSET;
./MESC_Common/Src/MESCfoc.c:361:	//  __HAL_TIM_ENABLE_IT(_motor->stimer, TIM_IT_UPDATE);
./MESC_Common/Src/MESCfoc.c:364:  	  HAL_TIM_Base_Start(_motor->stimer);
./MESC_Common/Src/MESCfoc.c:366:	  __HAL_TIM_SET_PRESCALER(_motor->stimer, ((HAL_RCC_GetHCLKFreq())/ 1000000 - 1));
./MESC_Common/Src/MESCfoc.c:367:	  __HAL_TIM_SET_AUTORELOAD(_motor->stimer,(1000000/SLOWTIM_SCALER) / SLOW_LOOP_FREQUENCY); //Run slowloop at 100Hz
./MESC_Common/Src/MESCfoc.c:368:	  __HAL_TIM_ENABLE_IT(_motor->stimer, TIM_IT_UPDATE);
./MESC_Common/Src/MESCfoc.c:378:  _motor->conf_is_valid = true;
./MESC_Common/Src/MESCfoc.c:381://	while(_motor->key_bits & UNINITIALISED_KEY){
./MESC_Common/Src/MESCfoc.c:382://		_motor->MotorState = MOTOR_STATE_INITIALISING;
./MESC_Common/Src/MESCfoc.c:390:      Iuoff += (float)_motor->Raw.Iu;
./MESC_Common/Src/MESCfoc.c:391:      Ivoff += (float)_motor->Raw.Iv;
./MESC_Common/Src/MESCfoc.c:392:      Iwoff += (float)_motor->Raw.Iw;
./MESC_Common/Src/MESCfoc.c:400:        _motor->FOC.flux_b = 0.001f;
./MESC_Common/Src/MESCfoc.c:401:        _motor->FOC.flux_a = 0.001f;
./MESC_Common/Src/MESCfoc.c:403:        _motor->offset.Iu =  Iuoff/initcycles;
./MESC_Common/Src/MESCfoc.c:404:        _motor->offset.Iv =  Ivoff/initcycles;
./MESC_Common/Src/MESCfoc.c:405:        _motor->offset.Iw =  Iwoff/initcycles;
./MESC_Common/Src/MESCfoc.c:410:		if((_motor->offset.Iu>1500) &&(_motor->offset.Iu<2600)&&(_motor->offset.Iv>1500) &&(_motor->offset.Iv<2600)&&(_motor->offset.Iw>1500) &&(_motor->offset.Iw<2600)){
./MESC_Common/Src/MESCfoc.c:412:					_motor->MotorState = MOTOR_STATE_TRACKING;
./MESC_Common/Src/MESCfoc.c:413:			        _motor->key_bits &= ~UNINITIALISED_KEY;
./MESC_Common/Src/MESCfoc.c:414:			        _motor->mtimer->Instance->BDTR |= TIM_BDTR_MOE;
./MESC_Common/Src/MESCfoc.c:441:	uint32_t period = cycles - _motor->jitter.last_entry_cyc;   // wrap-safe
./MESC_Common/Src/MESCfoc.c:442:	_motor->jitter.last_entry_cyc = cycles;
./MESC_Common/Src/MESCfoc.c:445:	    int32_t jitter_cyc = (int32_t)period - _motor->jitter.expected_cyc;
./MESC_Common/Src/MESCfoc.c:447:	    if (jitter_cyc < _motor->jitter.min_cyc) _motor->jitter.min_cyc = jitter_cyc;
./MESC_Common/Src/MESCfoc.c:448:	    if (jitter_cyc > _motor->jitter.max_cyc) _motor->jitter.max_cyc = jitter_cyc;
./MESC_Common/Src/MESCfoc.c:449:	    _motor->jitter.sum_cyc += jitter_cyc;
./MESC_Common/Src/MESCfoc.c:450:	    _motor->jitter.samples++;
./MESC_Common/Src/MESCfoc.c:454:	if (_motor->jitter.clear_req) {
./MESC_Common/Src/MESCfoc.c:455:	    _motor->jitter.min_cyc = INT32_MAX;
./MESC_Common/Src/MESCfoc.c:456:	    _motor->jitter.max_cyc = INT32_MIN;
./MESC_Common/Src/MESCfoc.c:457:	    _motor->jitter.sum_cyc = 0;
./MESC_Common/Src/MESCfoc.c:458:	    _motor->jitter.samples = 0;
./MESC_Common/Src/MESCfoc.c:459:	    _motor->jitter.clear_req = 0;
./MESC_Common/Src/MESCfoc.c:465:  _motor->hall.current_hall_state = getHallState(); //ToDo, this macro is not applicable to dual motors
./MESC_Common/Src/MESCfoc.c:470:  switch (_motor->MotorState) {
./MESC_Common/Src/MESCfoc.c:477:    	switch(_motor->MotorSensorMode){
./MESC_Common/Src/MESCfoc.c:479:				if(_motor->options.use_hall_start){
./MESC_Common/Src/MESCfoc.c:480:					if(_motor->FOC.hall_start_now){
./MESC_Common/Src/MESCfoc.c:481:						_motor->FOC.flux_a = (1.0f-_motor->FOC.hall_IIR)*_motor->FOC.flux_a + _motor->FOC.hall_IIR*_motor->m.hall_flux[_motor->hall.current_hall_state-1][0];
./MESC_Common/Src/MESCfoc.c:482:						_motor->FOC.flux_b = (1.0f-_motor->FOC.hall_IIR)*_motor->FOC.flux_b + _motor->FOC.hall_IIR*_motor->m.hall_flux[_motor->hall.current_hall_state-1][1];
./MESC_Common/Src/MESCfoc.c:483://						if(fabsf(_motor->FOC.Vdq.q-_motor->m.R*_motor->FOC.Idq_smoothed.q)>HALL_VOLTAGE_THRESHOLD){
./MESC_Common/Src/MESCfoc.c:487:							_motor->FOC.FOCAngle = (uint16_t)(32768.0f + 10430.0f * fast_atan2(_motor->FOC.flux_b, _motor->FOC.flux_a)) - 32768;
./MESC_Common/Src/MESCfoc.c:489:					}else if(_motor->FOC.enc_start_now){
./MESC_Common/Src/MESCfoc.c:490:						_motor->FOC.flux_a = 0.95f*_motor->FOC.flux_a + _motor->FOC.enccos * 0.05f * _motor->m.flux_linkage;
./MESC_Common/Src/MESCfoc.c:491:						_motor->FOC.flux_b = 0.95f*_motor->FOC.flux_b + _motor->FOC.encsin * 0.05f * _motor->m.flux_linkage;
./MESC_Common/Src/MESCfoc.c:502:				_motor->HFI.inject = 0;
./MESC_Common/Src/MESCfoc.c:513:				_motor->FOC.enc_period_count++;
./MESC_Common/Src/MESCfoc.c:514:				_motor->FOC.FOCAngle = _motor->FOC.enc_angle + (uint16_t)((float)(_motor->FOC.enc_period_count) * (float)_motor->FOC.enc_pwm_step);
./MESC_Common/Src/MESCfoc.c:519:				_motor->FOC.FOCAngle = _motor->FOC.enc_angle;
./MESC_Common/Src/MESCfoc.c:532:		  switch(_motor->MotorSensorMode){
./MESC_Common/Src/MESCfoc.c:539:				  if(_motor->options.use_hall_start){
./MESC_Common/Src/MESCfoc.c:544:		  		  _motor->FOC.FOCAngle = _motor->FOC.enc_angle;
./MESC_Common/Src/MESCfoc.c:548:		  		  _motor->FOC.FOCAngle = _motor->FOC.enc_angle;
./MESC_Common/Src/MESCfoc.c:559:    	_motor->FOC.openloop_step = 60;
./MESC_Common/Src/MESCfoc.c:579:      if ((_motor->hall.current_hall_state == 7)) { // no hall sensors detected, all GPIO pulled high
./MESC_Common/Src/MESCfoc.c:580:    	_motor->MotorSensorMode = MOTOR_SENSOR_MODE_SENSORLESS;
./MESC_Common/Src/MESCfoc.c:581:        _motor->MotorState = MOTOR_STATE_GET_KV;
./MESC_Common/Src/MESCfoc.c:582:      } else if (_motor->hall.current_hall_state == 0) {
./MESC_Common/Src/MESCfoc.c:583:        _motor->MotorState = MOTOR_STATE_ERROR;
./MESC_Common/Src/MESCfoc.c:587:    	  _motor->MotorSensorMode = MOTOR_SENSOR_MODE_HALL;
./MESC_Common/Src/MESCfoc.c:611:	  if(_motor->MotorSensorMode == MOTOR_SENSOR_MODE_INCREMENTAL_ENCODER){
./MESC_Common/Src/MESCfoc.c:612:		  _motor->FOC.FOCAngle = _motor->FOC.enc_angle;
./MESC_Common/Src/MESCfoc.c:652:      if((fabsf(_motor->Conv.Iu)>_motor->input_vars.max_request_Idq.q)||
./MESC_Common/Src/MESCfoc.c:653:		  (fabsf(_motor->Conv.Iv)>_motor->input_vars.max_request_Idq.q)||
./MESC_Common/Src/MESCfoc.c:654:		  (fabsf(_motor->Conv.Iw)>_motor->input_vars.max_request_Idq.q)){
./MESC_Common/Src/MESCfoc.c:663:    	  if(_motor->MotorSensorMode ==MOTOR_SENSOR_MODE_INCREMENTAL_ENCODER){
./MESC_Common/Src/MESCfoc.c:665:    		  _motor->FOC.FOCAngle = _motor->FOC.enc_angle;
./MESC_Common/Src/MESCfoc.c:666://     		  if((_motor->FOC.parkangle-_motor->FOC.FOCAngle)>16384){
./MESC_Common/Src/MESCfoc.c:667://     			  if((_motor->FOC.parkangle-_motor->FOC.FOCAngle)>32768){
./MESC_Common/Src/MESCfoc.c:668://    			  _motor->FOC.parkangle = _motor->FOC.FOCAngle+16384;
./MESC_Common/Src/MESCfoc.c:671://     		  if((_motor->FOC.FOCAngle-_motor->FOC.parkangle)>16384){
./MESC_Common/Src/MESCfoc.c:672://    			  if((_motor->FOC.FOCAngle-_motor->FOC.parkangle)<32767){
./MESC_Common/Src/MESCfoc.c:673://    				  _motor->FOC.parkangle = _motor->FOC.FOCAngle-16384;
./MESC_Common/Src/MESCfoc.c:676:    		  diff =(int)(_motor->FOC.FOCAngle-_motor->FOC.parkangle);
./MESC_Common/Src/MESCfoc.c:679:    				  _motor->FOC.parkangle = _motor->FOC.FOCAngle+16000;
./MESC_Common/Src/MESCfoc.c:682:    				  _motor->FOC.parkangle = _motor->FOC.FOCAngle-16000;
./MESC_Common/Src/MESCfoc.c:687:				  _motor->FOC.Vdq.q = 0.0f;
./MESC_Common/Src/MESCfoc.c:688:				  _motor->FOC.Vdq.d = 0.0f;
./MESC_Common/Src/MESCfoc.c:689:				  _motor->FOC.park_current_now = 0.0f;
./MESC_Common/Src/MESCfoc.c:691:    			  _motor->FOC.Idq_req.q = -_motor->FOC.park_current*(float)diff/(float)8192;//Fill with some PID logic
./MESC_Common/Src/MESCfoc.c:692:    			  _motor->FOC.Idq_req.d = 0.0f;//
./MESC_Common/Src/MESCfoc.c:694:    				  _motor->FOC.Idq_req.q = _motor->FOC.Idq_req.q + _motor->FOC.park_current;
./MESC_Common/Src/MESCfoc.c:696:    				  _motor->FOC.Idq_req.q = _motor->FOC.Idq_req.q - _motor->FOC.park_current;
./MESC_Common/Src/MESCfoc.c:699:    			  _motor->FOC.park_current_now = _motor->FOC.Idq_req.q;
./MESC_Common/Src/MESCfoc.c:702:			  _motor->FOC.Vdq.q = 0.0f;
./MESC_Common/Src/MESCfoc.c:703:			  _motor->FOC.Vdq.d = 0.0f;
./MESC_Common/Src/MESCfoc.c:704:			  _motor->FOC.park_current_now = 0.0f;
./MESC_Common/Src/MESCfoc.c:718:		_motor->MotorState = MOTOR_STATE_ERROR;
./MESC_Common/Src/MESCfoc.c:727:	if(_motor->options.use_lr_observer){
./MESC_Common/Src/MESCfoc.c:736:	_motor->FOC.PLL_angle = _motor->FOC.PLL_angle + (int16_t)_motor->FOC.PLL_int + (int16_t)_motor->FOC.PLL_error;
./MESC_Common/Src/MESCfoc.c:738:	_motor->FOC.PLL_error = _motor->FOC.PLL_kp * (int16_t)(_motor->FOC.FOCAngle - (_motor->FOC.PLL_angle & 0xFFFF));
./MESC_Common/Src/MESCfoc.c:739:	_motor->FOC.PLL_int = _motor->FOC.PLL_int + _motor->FOC.PLL_ki * _motor->FOC.PLL_error;
./MESC_Common/Src/MESCfoc.c:740:	_motor->FOC.eHz = _motor->FOC.PLL_int * _motor->FOC.pwm_frequency*0.00001526f;//1/65536
./MESC_Common/Src/MESCfoc.c:743:	if(_motor->logging.lognow){
./MESC_Common/Src/MESCfoc.c:745:		if(_motor->MotorState!=MOTOR_STATE_ERROR && _motor->logging.sample_now == false){
./MESC_Common/Src/MESCfoc.c:753:				_motor->logging.print_samples_now = 1;
./MESC_Common/Src/MESCfoc.c:754:				_motor->logging.sample_now = false;
./MESC_Common/Src/MESCfoc.c:768:   _motor->FOC.cycles_fastloop = CPU_CYCLES - cycles;
./MESC_Common/Src/MESCfoc.c:787:  if (_motor->Raw.Iu > g_hw_setup.RawCurrLim){
./MESC_Common/Src/MESCfoc.c:790:  if (_motor->Raw.Iv > g_hw_setup.RawCurrLim){
./MESC_Common/Src/MESCfoc.c:793:  if (_motor->Raw.Iw > g_hw_setup.RawCurrLim){
./MESC_Common/Src/MESCfoc.c:796:  if (_motor->Raw.Vbus > g_hw_setup.RawVoltLim){
./MESC_Common/Src/MESCfoc.c:803:	_motor->FOC.Idq_smoothed.d = (_motor->FOC.Idq_smoothed.d*99.0f + _motor->FOC.Idq.d)*0.01f;
./MESC_Common/Src/MESCfoc.c:804:	_motor->FOC.Idq_smoothed.q = (_motor->FOC.Idq_smoothed.q*99.0f + _motor->FOC.Idq.q)*0.01f;
./MESC_Common/Src/MESCfoc.c:811:	_motor->Conv.Iu = ((float)_motor->Raw.Iu - _motor->offset.Iu) * g_hw_setup.Igain;
./MESC_Common/Src/MESCfoc.c:812:	_motor->Conv.Iv = ((float)_motor->Raw.Iv - _motor->offset.Iv) * g_hw_setup.Igain;
./MESC_Common/Src/MESCfoc.c:813:	_motor->Conv.Iw = ((float)_motor->Raw.Iw - _motor->offset.Iw) * g_hw_setup.Igain;
./MESC_Common/Src/MESCfoc.c:814:	_motor->Conv.Vbus =	(float)_motor->Raw.Vbus * g_hw_setup.VBGain;  // Vbus
./MESC_Common/Src/MESCfoc.c:819:	if (_motor->Conv.Iu > g_hw_setup.Imax){
./MESC_Common/Src/MESCfoc.c:822:	if (_motor->Conv.Iv > g_hw_setup.Imax){
./MESC_Common/Src/MESCfoc.c:825:	if (_motor->Conv.Iw > g_hw_setup.Imax){
./MESC_Common/Src/MESCfoc.c:828:	if (_motor->Conv.Vbus > g_hw_setup.Vmax){
./MESC_Common/Src/MESCfoc.c:831:	if (_motor->Conv.Vbus < g_hw_setup.Vmin){
./MESC_Common/Src/MESCfoc.c:838:    _motor->Conv.Iu =
./MESC_Common/Src/MESCfoc.c:839:    		-_motor->Conv.Iv -_motor->Conv.Iw;
./MESC_Common/Src/MESCfoc.c:842:    _motor->Conv.Iv =
./MESC_Common/Src/MESCfoc.c:843:    		-_motor->Conv.Iu -_motor->Conv.Iw;
./MESC_Common/Src/MESCfoc.c:846:    _motor->Conv.Iw =
./MESC_Common/Src/MESCfoc.c:847:    		-_motor->Conv.Iu -_motor->Conv.Iv;
./MESC_Common/Src/MESCfoc.c:851:    _motor->FOC.Iab.a = _motor->Conv.Iu;
./MESC_Common/Src/MESCfoc.c:852:    _motor->FOC.Iab.b = _motor->Conv.Iv;
./MESC_Common/Src/MESCfoc.c:858:    switch(_motor->HighPhase){
./MESC_Common/Src/MESCfoc.c:861:			_motor->FOC.Iab.a = -_motor->Conv.Iv -
./MESC_Common/Src/MESCfoc.c:862:				  _motor->Conv.Iw;
./MESC_Common/Src/MESCfoc.c:863:			_motor->FOC.Iab.b =
./MESC_Common/Src/MESCfoc.c:864:				one_on_sqrt3 * _motor->Conv.Iv -
./MESC_Common/Src/MESCfoc.c:865:				one_on_sqrt3 * _motor->Conv.Iw;
./MESC_Common/Src/MESCfoc.c:869:			_motor->FOC.Iab.a = _motor->Conv.Iu;
./MESC_Common/Src/MESCfoc.c:870:			_motor->FOC.Iab.b =
./MESC_Common/Src/MESCfoc.c:871:				-one_on_sqrt3 * _motor->Conv.Iu -
./MESC_Common/Src/MESCfoc.c:872:				two_on_sqrt3 * _motor->Conv.Iw;
./MESC_Common/Src/MESCfoc.c:876:			_motor->FOC.Iab.a = _motor->Conv.Iu;
./MESC_Common/Src/MESCfoc.c:877:			_motor->FOC.Iab.b =
./MESC_Common/Src/MESCfoc.c:878:				two_on_sqrt3 * _motor->Conv.Iv +
./MESC_Common/Src/MESCfoc.c:879:				one_on_sqrt3 * _motor->Conv.Iu;
./MESC_Common/Src/MESCfoc.c:883:			if(_motor->options.use_phase_balancing){
./MESC_Common/Src/MESCfoc.c:884:				_motor->FOC.Iab.g = 0.33f * (_motor->Conv.Iu + _motor->Conv.Iv + _motor->Conv.Iw);
./MESC_Common/Src/MESCfoc.c:885:				_motor->Conv.Iu = _motor->Conv.Iu - _motor->FOC.Iab.g;
./MESC_Common/Src/MESCfoc.c:886:				_motor->Conv.Iv = _motor->Conv.Iv - _motor->FOC.Iab.g;
./MESC_Common/Src/MESCfoc.c:887:				_motor->Conv.Iw = _motor->Conv.Iw - _motor->FOC.Iab.g;
./MESC_Common/Src/MESCfoc.c:888:				if(fabs(_motor->FOC.Iab.g)>fabs(_motor->FOC.maxIgamma)){
./MESC_Common/Src/MESCfoc.c:889:					_motor->FOC.maxIgamma = _motor->FOC.Iab.g;
./MESC_Common/Src/MESCfoc.c:891:				if(_motor->FOC.Vdq.q<2.0f){ //Reset it to reject accumulated random noise and enable multiple goes
./MESC_Common/Src/MESCfoc.c:892:					_motor->FOC.maxIgamma = 0.0f;
./MESC_Common/Src/MESCfoc.c:897:	      _motor->FOC.Iab.a =
./MESC_Common/Src/MESCfoc.c:898:	          0.66666f * _motor->Conv.Iu -
./MESC_Common/Src/MESCfoc.c:899:	          0.33333f * _motor->Conv.Iv -
./MESC_Common/Src/MESCfoc.c:900:	          0.33333f * _motor->Conv.Iw;
./MESC_Common/Src/MESCfoc.c:901:	      _motor->FOC.Iab.b =
./MESC_Common/Src/MESCfoc.c:902:	          one_on_sqrt3 * _motor->Conv.Iv -
./MESC_Common/Src/MESCfoc.c:903:	          one_on_sqrt3 * _motor->Conv.Iw;
./MESC_Common/Src/MESCfoc.c:908:    _motor->FOC.Idq.d = _motor->FOC.sincosangle.cos * _motor->FOC.Iab.a +
./MESC_Common/Src/MESCfoc.c:909:                     _motor->FOC.sincosangle.sin * _motor->FOC.Iab.b;
./MESC_Common/Src/MESCfoc.c:910:    _motor->FOC.Idq.q = _motor->FOC.sincosangle.cos * _motor->FOC.Iab.b -
./MESC_Common/Src/MESCfoc.c:911:                     _motor->FOC.sincosangle.sin * _motor->FOC.Iab.a;
./MESC_Common/Src/MESCfoc.c:917:	_motor->Conv.Vu = (float)_motor->Raw.Vu * g_hw_setup.VBGain;
./MESC_Common/Src/MESCfoc.c:918:	_motor->Conv.Vv = (float)_motor->Raw.Vv * g_hw_setup.VBGain;
./MESC_Common/Src/MESCfoc.c:919:	_motor->Conv.Vw = (float)_motor->Raw.Vw * g_hw_setup.VBGain;
./MESC_Common/Src/MESCfoc.c:961:	if (_motor->hall.current_hall_state != _motor->hall.last_hall_state) {
./MESC_Common/Src/MESCfoc.c:962:		_motor->FOC.hall_update = 1;
./MESC_Common/Src/MESCfoc.c:963:		if (_motor->hall.current_hall_state == 0) {
./MESC_Common/Src/MESCfoc.c:964:			_motor->MotorState = MOTOR_STATE_ERROR;
./MESC_Common/Src/MESCfoc.c:966:		} else if (_motor->hall.current_hall_state == 7) {
./MESC_Common/Src/MESCfoc.c:967:			_motor->MotorState = MOTOR_STATE_ERROR;
./MESC_Common/Src/MESCfoc.c:972:		_motor->hall.current_hall_angle = _motor->m.hall_table[_motor->hall.current_hall_state - 1][2];
./MESC_Common/Src/MESCfoc.c:977:		if ((a = _motor->hall.current_hall_angle - _motor->hall.last_hall_angle) < 32000) {  // Forwards
./MESC_Common/Src/MESCfoc.c:978:			_motor->hall.hall_error =
./MESC_Common/Src/MESCfoc.c:979:				  _motor->FOC.FOCAngle - _motor->m.hall_table[_motor->hall.current_hall_state - 1][0];
./MESC_Common/Src/MESCfoc.c:980:			_motor->hall.dir = 1.0f;
./MESC_Common/Src/MESCfoc.c:981:		// _motor->FOC.HallAngle = _motor->FOC.HallAngle - 5460;
./MESC_Common/Src/MESCfoc.c:984:			_motor->hall.hall_error =
./MESC_Common/Src/MESCfoc.c:985:					_motor->FOC.FOCAngle - _motor->m.hall_table[_motor->hall.current_hall_state - 1][1];
./MESC_Common/Src/MESCfoc.c:986:			_motor->hall.dir = -1.0f;
./MESC_Common/Src/MESCfoc.c:987:			// _motor->FOC.HallAngle = _motor->FOC.HallAngle + 5460;
./MESC_Common/Src/MESCfoc.c:989:		if (_motor->hall.hall_error > 32000) {
./MESC_Common/Src/MESCfoc.c:990:			_motor->hall.hall_error = _motor->hall.hall_error - 65536;
./MESC_Common/Src/MESCfoc.c:992:		if (_motor->hall.hall_error < -32000) {
./MESC_Common/Src/MESCfoc.c:993:			_motor->hall.hall_error = _motor->hall.hall_error + 65536;
./MESC_Common/Src/MESCfoc.c:1001:    if (_motor->FOC.hall_update == 1) {
./MESC_Common/Src/MESCfoc.c:1002:      _motor->FOC.hall_update = 0;
./MESC_Common/Src/MESCfoc.c:1003:      _motor->hall.last_observer_period = _motor->hall.ticks_since_last_observer_change;
./MESC_Common/Src/MESCfoc.c:1004:      float one_on_ticks = (1.0f / _motor->hall.ticks_since_last_observer_change);
./MESC_Common/Src/MESCfoc.c:1005:      _motor->hall.one_on_last_observer_period =
./MESC_Common/Src/MESCfoc.c:1006:          (4.0f * _motor->hall.one_on_last_observer_period + (one_on_ticks)) * 0.2f;  // ;
./MESC_Common/Src/MESCfoc.c:1007:      _motor->hall.angle_step =
./MESC_Common/Src/MESCfoc.c:1008:          (4.0f * _motor->hall.angle_step +
./MESC_Common/Src/MESCfoc.c:1009:           (one_on_ticks)*_motor->m.hall_table[_motor->hall.last_hall_state - 1][3]) *
./MESC_Common/Src/MESCfoc.c:1013:      _motor->hall.last_hall_state = _motor->hall.current_hall_state;
./MESC_Common/Src/MESCfoc.c:1014:      _motor->hall.last_hall_angle = _motor->hall.current_hall_angle;
./MESC_Common/Src/MESCfoc.c:1015:      _motor->hall.ticks_since_last_observer_change = 0;
./MESC_Common/Src/MESCfoc.c:1019:    _motor->hall.ticks_since_last_observer_change = _motor->hall.ticks_since_last_observer_change + 1;
./MESC_Common/Src/MESCfoc.c:1021:    if (_motor->hall.ticks_since_last_observer_change <= 2.0f * _motor->hall.last_observer_period) {
./MESC_Common/Src/MESCfoc.c:1022:      /*      _motor->FOC.FOCAngle = _motor->FOC.FOCAngle + (uint16_t)(dir*angle_step
./MESC_Common/Src/MESCfoc.c:1027:      if (_motor->hall.dir > 0) {  // Apply a gain to the error as well as the feed forward
./MESC_Common/Src/MESCfoc.c:1030:        _motor->FOC.FOCAngle =
./MESC_Common/Src/MESCfoc.c:1031:            _motor->FOC.FOCAngle +
./MESC_Common/Src/MESCfoc.c:1032:            (uint16_t)(_motor->hall.angle_step - _motor->hall.one_on_last_observer_period * _motor->hall.hall_error);
./MESC_Common/Src/MESCfoc.c:1034:      } else if (_motor->hall.dir < 0.0f) {
./MESC_Common/Src/MESCfoc.c:1035:        _motor->FOC.FOCAngle =
./MESC_Common/Src/MESCfoc.c:1036:            _motor->FOC.FOCAngle +
./MESC_Common/Src/MESCfoc.c:1037:            (uint16_t)(-_motor->hall.angle_step +
./MESC_Common/Src/MESCfoc.c:1038:            		_motor->hall.one_on_last_observer_period * (-0.9f * _motor->hall.hall_error));
./MESC_Common/Src/MESCfoc.c:1041:        _motor->FOC.FOCAngle =
./MESC_Common/Src/MESCfoc.c:1042:            _motor->FOC.FOCAngle -
./MESC_Common/Src/MESCfoc.c:1043:            (uint16_t)(_motor->hall.angle_step +
./MESC_Common/Src/MESCfoc.c:1044:            		_motor->hall.one_on_last_observer_period * (0.2f * _motor->hall.hall_error));
./MESC_Common/Src/MESCfoc.c:1047:    if (_motor->hall.ticks_since_last_observer_change > 1500.0f) {
./MESC_Common/Src/MESCfoc.c:1048:    	_motor->hall.ticks_since_last_observer_change = 1500.0f;
./MESC_Common/Src/MESCfoc.c:1049:    	_motor->hall.last_observer_period = 1500.0f;  //(ticks_since_last_hall_change);
./MESC_Common/Src/MESCfoc.c:1050:    	_motor->hall.one_on_last_observer_period =
./MESC_Common/Src/MESCfoc.c:1051:          1.0f / _motor->hall.last_observer_period;  // / ticks_since_last_hall_change;
./MESC_Common/Src/MESCfoc.c:1052:      _motor->FOC.FOCAngle = _motor->hall.current_hall_angle;
./MESC_Common/Src/MESCfoc.c:1057:	//_motor->FOC.PLL_int = 0.5f*_motor->FOC.openloop_step;
./MESC_Common/Src/MESCfoc.c:1058:    _motor->FOC.FOCAngle = _motor->FOC.FOCAngle + _motor->FOC.openloop_step;
./MESC_Common/Src/MESCfoc.c:1074:    Idq_err.q = (_motor->FOC.Idq_req.q - 0.5f *(_motor->FOC.Idq.q + Idq_last.q)) * _motor->FOC.Iq_pgain;
./MESC_Common/Src/MESCfoc.c:1075:    Idq_last.q = _motor->FOC.Idq.q;
./MESC_Common/Src/MESCfoc.c:1076:    //    Idq_err.q = (_motor->FOC.Idq_req.q - _motor->FOC.Idq.q) * _motor->FOC.Iq_pgain;
./MESC_Common/Src/MESCfoc.c:1079:    if(_motor->options.field_weakening != FIELD_WEAKENING_OFF){
./MESC_Common/Src/MESCfoc.c:1080:		if((_motor->FOC.FW_current<_motor->FOC.Idq_req.d)&&(_motor->MotorState==MOTOR_STATE_RUN)){//Field weakenning is -ve, but there may already be d-axis from the MTPA
./MESC_Common/Src/MESCfoc.c:1081://			Idq_err.d = (_motor->FOC.FW_current - _motor->FOC.Idq.d) * _motor->FOC.Id_pgain;
./MESC_Common/Src/MESCfoc.c:1082:		    Idq_err.d = (_motor->FOC.FW_current - 0.5f *(_motor->FOC.Idq.d + Idq_last.d)) * _motor->FOC.Id_pgain;
./MESC_Common/Src/MESCfoc.c:1085://			Idq_err.d = (_motor->FOC.Idq_req.d - _motor->FOC.Idq.d) * _motor->FOC.Id_pgain;
./MESC_Common/Src/MESCfoc.c:1086:		    Idq_err.d = (_motor->FOC.Idq_req.d - 0.5f *(_motor->FOC.Idq.d + Idq_last.d)) * _motor->FOC.Id_pgain;
./MESC_Common/Src/MESCfoc.c:1089://    	Idq_err.d = (_motor->FOC.Idq_req.d - _motor->FOC.Idq.d) * _motor->FOC.Id_pgain;
./MESC_Common/Src/MESCfoc.c:1090:	    Idq_err.d = (_motor->FOC.Idq_req.d - 0.5f *(_motor->FOC.Idq.d + Idq_last.d)) * _motor->FOC.Id_pgain;
./MESC_Common/Src/MESCfoc.c:1094:    Idq_last.d = _motor->FOC.Idq.d;
./MESC_Common/Src/MESCfoc.c:1098:    _motor->FOC.Idq_int_err.d =
./MESC_Common/Src/MESCfoc.c:1099:    		_motor->FOC.Idq_int_err.d + _motor->FOC.Id_igain * Idq_err.d * _motor->FOC.pwm_period;
./MESC_Common/Src/MESCfoc.c:1100:    _motor->FOC.Idq_int_err.q =
./MESC_Common/Src/MESCfoc.c:1101:    		_motor->FOC.Idq_int_err.q + _motor->FOC.Iq_igain * Idq_err.q * _motor->FOC.pwm_period;
./MESC_Common/Src/MESCfoc.c:1105:      _motor->FOC.Vdq.d = Idq_err.d + _motor->FOC.Idq_int_err.d;
./MESC_Common/Src/MESCfoc.c:1106:      _motor->FOC.Vdq.q = Idq_err.q + _motor->FOC.Idq_int_err.q;
./MESC_Common/Src/MESCfoc.c:1110:switch(_motor->options.sqrt_circle_lim){
./MESC_Common/Src/MESCfoc.c:1122:	  _motor->FOC.Idq_int_err.d = clamp(_motor->FOC.Idq_int_err.d, -_motor->FOC.Vdint_max, _motor->FOC.Vdint_max);
./MESC_Common/Src/MESCfoc.c:1123:	  _motor->FOC.Idq_int_err.q = clamp(_motor->FOC.Idq_int_err.q, -_motor->FOC.Vqint_max, _motor->FOC.Vqint_max);
./MESC_Common/Src/MESCfoc.c:1126:    _motor->FOC.Vdq.d = clamp(_motor->FOC.Vdq.d, -_motor->FOC.Vd_max, _motor->FOC.Vd_max);
./MESC_Common/Src/MESCfoc.c:1127:    _motor->FOC.Vdq.q = clamp(_motor->FOC.Vdq.q, -_motor->FOC.Vq_max, _motor->FOC.Vq_max);
./MESC_Common/Src/MESCfoc.c:1130:    Vmagnow2 = _motor->FOC.Vdq.d*_motor->FOC.Vdq.d+_motor->FOC.Vdq.q*_motor->FOC.Vdq.q;
./MESC_Common/Src/MESCfoc.c:1132:    _motor->FOC.Voltage = sqrtf(Vmagnow2);
./MESC_Common/Src/MESCfoc.c:1133:    if(_motor->FOC.Voltage > _motor->FOC.Vmag_max){
./MESC_Common/Src/MESCfoc.c:1135:		  float one_on_Vmagnow = 1.0f/_motor->FOC.Voltage;
./MESC_Common/Src/MESCfoc.c:1136:		  float one_on_VmagnowxVmagmax = _motor->FOC.Vmag_max*one_on_Vmagnow;
./MESC_Common/Src/MESCfoc.c:1137:		  _motor->FOC.Vdq.d = _motor->FOC.Vdq.d*one_on_VmagnowxVmagmax;
./MESC_Common/Src/MESCfoc.c:1138:		  _motor->FOC.Vdq.q = _motor->FOC.Vdq.q*one_on_VmagnowxVmagmax;
./MESC_Common/Src/MESCfoc.c:1139:		  _motor->FOC.Idq_int_err.d = _motor->FOC.Idq_int_err.d*one_on_VmagnowxVmagmax;
./MESC_Common/Src/MESCfoc.c:1140:		  _motor->FOC.Idq_int_err.q = _motor->FOC.Idq_int_err.q*one_on_VmagnowxVmagmax;
./MESC_Common/Src/MESCfoc.c:1142:		  if(_motor->options.field_weakening == FIELD_WEAKENING_V2){
./MESC_Common/Src/MESCfoc.c:1148:			  _motor->FOC.FW_current = 0.99f*_motor->FOC.FW_current -0.01f*_motor->FOC.FW_curr_max;
./MESC_Common/Src/MESCfoc.c:1153:  	  if(_motor->options.field_weakening == FIELD_WEAKENING_V2){
./MESC_Common/Src/MESCfoc.c:1154:  		  _motor->FOC.FW_current = 1.01f*_motor->FOC.FW_current + 0.0101f*_motor->FOC.FW_curr_max;
./MESC_Common/Src/MESCfoc.c:1158:    if(_motor->options.field_weakening == FIELD_WEAKENING_V2){
./MESC_Common/Src/MESCfoc.c:1160:  	  if(_motor->FOC.FW_current>_motor->FOC.Idq_req.d){_motor->FOC.FW_current = _motor->FOC.Idq_req.d;}
./MESC_Common/Src/MESCfoc.c:1161:  	  if(_motor->FOC.FW_current<-_motor->FOC.FW_curr_max){_motor->FOC.FW_current = -_motor->FOC.FW_curr_max;}
./MESC_Common/Src/MESCfoc.c:1175:	  if(_motor->FOC.Vdq.d<-0.866f*_motor->FOC.Vmag_max){ //Negative values of Vd - Normally Vd is -ve since it is driving field advance
./MESC_Common/Src/MESCfoc.c:1176:		  _motor->FOC.Vdq.d = -0.866f*_motor->FOC.Vmag_max; //Hard clamp the Vd
./MESC_Common/Src/MESCfoc.c:1177:		  if(_motor->FOC.Idq_int_err.d<_motor->FOC.Vdq.d){
./MESC_Common/Src/MESCfoc.c:1178:			  _motor->FOC.Idq_int_err.d = _motor->FOC.Vdq.d; //Also clamp the integral to stop windup
./MESC_Common/Src/MESCfoc.c:1180:	  } else if(_motor->FOC.Vdq.d>0.866f*_motor->FOC.Vmag_max){ //Positive values of Vd
./MESC_Common/Src/MESCfoc.c:1181:		  _motor->FOC.Vdq.d = 0.866f*_motor->FOC.Vmag_max; //Hard clamp the Vd
./MESC_Common/Src/MESCfoc.c:1182:		  if(_motor->FOC.Idq_int_err.d>_motor->FOC.Vdq.d){
./MESC_Common/Src/MESCfoc.c:1183:			  _motor->FOC.Idq_int_err.d = _motor->FOC.Vdq.d; //Also clamp the integral to stop windup
./MESC_Common/Src/MESCfoc.c:1188:	  Vmagnow2 = _motor->FOC.Vdq.d*_motor->FOC.Vdq.d+_motor->FOC.Vdq.q*_motor->FOC.Vdq.q;
./MESC_Common/Src/MESCfoc.c:1189:	  _motor->FOC.Voltage = sqrtf(Vmagnow2);
./MESC_Common/Src/MESCfoc.c:1190:	  if(_motor->FOC.Voltage > _motor->FOC.Vmag_max){
./MESC_Common/Src/MESCfoc.c:1191:		  _motor->FOC.Voltage = _motor->FOC.Vmag_max;
./MESC_Common/Src/MESCfoc.c:1192:		  if(_motor->FOC.Vdq.q>0.0f){ //Positive Vq
./MESC_Common/Src/MESCfoc.c:1193:			  _motor->FOC.Vdq.q = sqrtf(_motor->FOC.Vmag_max2-_motor->FOC.Vdq.d*_motor->FOC.Vdq.d);
./MESC_Common/Src/MESCfoc.c:1194:			  if(_motor->FOC.Idq_int_err.q>_motor->FOC.Vdq.q){
./MESC_Common/Src/MESCfoc.c:1195:				  _motor->FOC.Idq_int_err.q = _motor->FOC.Vdq.q;
./MESC_Common/Src/MESCfoc.c:1199:			  _motor->FOC.Vdq.q = -sqrtf(_motor->FOC.Vmag_max2-_motor->FOC.Vdq.d*_motor->FOC.Vdq.d);
./MESC_Common/Src/MESCfoc.c:1200:			  if(_motor->FOC.Idq_int_err.q<_motor->FOC.Vdq.q){
./MESC_Common/Src/MESCfoc.c:1201:				  _motor->FOC.Idq_int_err.q = _motor->FOC.Vdq.q;
./MESC_Common/Src/MESCfoc.c:1206:	  if(_motor->options.field_weakening == FIELD_WEAKENING_V2){
./MESC_Common/Src/MESCfoc.c:1207:	      if(_motor->FOC.Voltage > 0.95f*_motor->FOC.Vmag_max){
./MESC_Common/Src/MESCfoc.c:1211:	    		  _motor->FOC.FW_current = 0.99f*_motor->FOC.FW_current -0.01f*_motor->FOC.FW_curr_max;
./MESC_Common/Src/MESCfoc.c:1214:			  _motor->FOC.FW_current = 1.01f*_motor->FOC.FW_current + 0.0101f*_motor->FOC.FW_curr_max;
./MESC_Common/Src/MESCfoc.c:1216:	      if(_motor->FOC.FW_current>_motor->FOC.Idq_req.d){_motor->FOC.FW_current = _motor->FOC.Idq_req.d;}
./MESC_Common/Src/MESCfoc.c:1217:	      if(_motor->FOC.FW_current<-_motor->FOC.FW_curr_max){_motor->FOC.FW_current = -_motor->FOC.FW_curr_max;}
./MESC_Common/Src/MESCfoc.c:1223:	if(_motor->options.field_weakening == FIELD_WEAKENING_V1){
./MESC_Common/Src/MESCfoc.c:1225:		  float Vmagnow2 = _motor->FOC.Vdq.d*_motor->FOC.Vdq.d+_motor->FOC.Vdq.q*_motor->FOC.Vdq.q; //Need to recalculate this since limitation has maybe been applied
./MESC_Common/Src/MESCfoc.c:1228:		  if(Vmagnow2>(_motor->FOC.FW_threshold*_motor->FOC.FW_threshold)){
./MESC_Common/Src/MESCfoc.c:1229:			  _motor->FOC.FW_current = 0.95f*_motor->FOC.FW_current +
./MESC_Common/Src/MESCfoc.c:1230:							0.05f*_motor->FOC.FW_curr_max *_motor->FOC.FW_multiplier*
./MESC_Common/Src/MESCfoc.c:1231:							(_motor->FOC.FW_threshold - sqrtf(Vmagnow2));
./MESC_Common/Src/MESCfoc.c:1233:			  _motor->FOC.FW_current*=0.95f;//Ramp down a bit slowly
./MESC_Common/Src/MESCfoc.c:1234:			  if(_motor->FOC.FW_current>0.1f){//We do not allow positive field weakening current, and we want it to actually go to zero eventually
./MESC_Common/Src/MESCfoc.c:1235:				  _motor->FOC.FW_current = 0.0f;
./MESC_Common/Src/MESCfoc.c:1248:	  _motor->m.flux_linkage_max = 1.7f*_motor->m.flux_linkage;
./MESC_Common/Src/MESCfoc.c:1249:	  _motor->m.flux_linkage_min = 0.5f*_motor->m.flux_linkage;
./MESC_Common/Src/MESCfoc.c:1250:	  _motor->m.flux_linkage_gain = 10.0f * sqrtf(_motor->m.flux_linkage);
./MESC_Common/Src/MESCfoc.c:1251:	  _motor->m.non_linear_centering_gain = NON_LINEAR_CENTERING_GAIN;
./MESC_Common/Src/MESCfoc.c:1255:    _motor->FOC.pwm_period = 1.0f/_motor->FOC.pwm_frequency;
./MESC_Common/Src/MESCfoc.c:1256:    _motor->mtimer->Instance->ARR = HAL_RCC_GetHCLKFreq()/(((float)_motor->mtimer->Instance->PSC + 1.0f) * 2*_motor->FOC.pwm_frequency);
./MESC_Common/Src/MESCfoc.c:1257:    _motor->mtimer->Instance->CCR4 = _motor->mtimer->Instance->ARR-5; //Just short of dead center (dead center will not actually trigger the conversion)
./MESC_Common/Src/MESCfoc.c:1259:    _motor->mtimer->Instance->CCR4 = _motor->mtimer->Instance->ARR-80; //If we only have one ADC, we need to convert early otherwise the data will not be ready in time
./MESC_Common/Src/MESCfoc.c:1261:    _motor->FOC.PWMmid = _motor->mtimer->Instance->ARR * 0.5f;
./MESC_Common/Src/MESCfoc.c:1263:    _motor->FOC.ADC_duty_threshold = _motor->mtimer->Instance->ARR * 0.90f;
./MESC_Common/Src/MESCfoc.c:1264:    _motor->m.pole_angle = 65536/_motor->m.pole_pairs;
./MESC_Common/Src/MESCfoc.c:1268:    _motor->FOC.Id_pgain = _motor->FOC.Current_bandwidth * _motor->m.L_D;
./MESC_Common/Src/MESCfoc.c:1269:    _motor->FOC.Id_igain = _motor->m.R / _motor->m.L_D;
./MESC_Common/Src/MESCfoc.c:1271:    _motor->FOC.Iq_pgain = _motor->FOC.Id_pgain;
./MESC_Common/Src/MESCfoc.c:1272:    _motor->FOC.Iq_igain = _motor->FOC.Id_igain;
./MESC_Common/Src/MESCfoc.c:1274:	  if(_motor->FOC.FW_curr_max > 0.9f * _motor->input_vars.max_request_Idq.q){
./MESC_Common/Src/MESCfoc.c:1275:		  _motor->FOC.FW_curr_max = 0.9f * _motor->input_vars.max_request_Idq.q; //Limit the field weakenning to 90% of the max current to avoid math errors
./MESC_Common/Src/MESCfoc.c:1277:	_motor->m.L_QD = _motor->m.L_Q-_motor->m.L_D;
./MESC_Common/Src/MESCfoc.c:1278:	_motor->FOC.d_polarity = 1;
./MESC_Common/Src/MESCfoc.c:1282:	if (_motor->FOC.pwm_frequency <= 0.0f) {
./MESC_Common/Src/MESCfoc.c:1283:        _motor->FOC.pwm_frequency = PWM_FREQUENCY;  // fallback to default 20kHz
./MESC_Common/Src/MESCfoc.c:1285:    _motor->jitter.expected_cyc = (int32_t)((float)SystemCoreClock / _motor->FOC.pwm_frequency);
./MESC_Common/Src/MESCfoc.c:1292:    _motor->FOC.Vab_to_PWM =
./MESC_Common/Src/MESCfoc.c:1293:        _motor->mtimer->Instance->ARR / _motor->Conv.Vbus;
./MESC_Common/Src/MESCfoc.c:1297:    if(_motor->ControlMode != MOTOR_CONTROL_MODE_DUTY){_motor->FOC.Duty_scaler = 1.0f;}
./MESC_Common/Src/MESCfoc.c:1298:    _motor->FOC.Vmag_max = 0.5f * _motor->Conv.Vbus *
./MESC_Common/Src/MESCfoc.c:1299:    		_motor->FOC.Modulation_max * SVPWM_MULTIPLIER * _motor->FOC.Duty_scaler;
./MESC_Common/Src/MESCfoc.c:1300:    _motor->FOC.V_3Q_mag_max =  _motor->FOC.Vmag_max * 0.75f;
./MESC_Common/Src/MESCfoc.c:1302:    _motor->FOC.Vmag_max2 = _motor->FOC.Vmag_max*_motor->FOC.Vmag_max;
./MESC_Common/Src/MESCfoc.c:1303:    _motor->FOC.Vd_max = 0.5f * _motor->Conv.Vbus *
./MESC_Common/Src/MESCfoc.c:1304:    		_motor->FOC.Modulation_max * SVPWM_MULTIPLIER * Vd_MAX_PROPORTION;
./MESC_Common/Src/MESCfoc.c:1305:    _motor->FOC.Vq_max = 0.5f * _motor->Conv.Vbus *
./MESC_Common/Src/MESCfoc.c:1306:    		_motor->FOC.Modulation_max * SVPWM_MULTIPLIER * Vq_MAX_PROPORTION;
./MESC_Common/Src/MESCfoc.c:1308:    _motor->FOC.Vdint_max = _motor->FOC.Vd_max * 0.9f; //Logic in this is to always ensure headroom for the P term
./MESC_Common/Src/MESCfoc.c:1309:    _motor->FOC.Vqint_max = _motor->FOC.Vq_max * 0.9f;
./MESC_Common/Src/MESCfoc.c:1311:    _motor->FOC.FW_threshold = _motor->FOC.Vmag_max * FIELD_WEAKENING_THRESHOLD;
./MESC_Common/Src/MESCfoc.c:1312:    _motor->FOC.FW_multiplier = 1.0f/(_motor->FOC.Vmag_max*(1.0f-FIELD_WEAKENING_THRESHOLD));
./MESC_Common/Src/MESCfoc.c:1314:    switch(_motor->HFI.Type){//When running HFI we want the bandwidth low, so we calculate it with each slow loop depending on whether we are HFIing or not
./MESC_Common/Src/MESCfoc.c:1323:		_motor->FOC.Id_pgain = _motor->FOC.Current_bandwidth * _motor->m.L_D;
./MESC_Common/Src/MESCfoc.c:1324:		_motor->FOC.Id_igain = _motor->m.R / _motor->m.L_D;
./MESC_Common/Src/MESCfoc.c:1326:		_motor->FOC.Iq_pgain = _motor->FOC.Id_pgain;
./MESC_Common/Src/MESCfoc.c:1327:		_motor->FOC.Iq_igain = _motor->FOC.Id_igain;
./MESC_Common/Src/MESCfoc.c:1329:		//_motor->FOC.HFI_Threshold = ((HFI_VOLTAGE*sqrt2*2.0f)*_motor->FOC.pwm_period)/((_motor->m.L_D+_motor->m.L_Q)*0.5f);
./MESC_Common/Src/MESCfoc.c:1331:		_motor->HFI.toggle_voltage = mtr->Conv.Vbus*0.05f;
./MESC_Common/Src/MESCfoc.c:1332:			if(_motor->HFI.toggle_voltage<1.5f){_motor->HFI.toggle_voltage = 1.5f;} //Must be greater than HFI hysteresis
./MESC_Common/Src/MESCfoc.c:1334:		_motor->HFI.toggle_voltage = HFI_THRESHOLD;
./MESC_Common/Src/MESCfoc.c:1341:	g_hw_setup.Imax = _motor->input_vars.max_request_Idq.q * 1.5f;
./MESC_Common/Src/MESCfoc.c:1343:		g_hw_setup.Imax = _motor->input_vars.max_request_Idq.q + 0.1f*ABS_MAX_PHASE_CURRENT;
./MESC_Common/Src/MESCfoc.c:1350:	if(fabsf(_motor->FOC.Idq_req.q)<1.0f){
./MESC_Common/Src/MESCfoc.c:1351:	g_hw_setup.Vmax = 	0.995f * g_hw_setup.Vmax + 0.005f * (_motor->Conv.Vbus + 0.15f * ABS_MAX_BUS_VOLTAGE);
./MESC_Common/Src/MESCfoc.c:1379:		switch(_motor->options.app_type){
./MESC_Common/Src/MESCfoc.c:1381:				_motor->key_bits &= ~APP_KEY;
./MESC_Common/Src/MESCfoc.c:1395:	  switch(_motor->ControlMode){
./MESC_Common/Src/MESCfoc.c:1407:			  _motor->FOC.Idq_prereq = _motor->input_vars.max_request_Idq;
./MESC_Common/Src/MESCfoc.c:1409:			  float total_in = 	_motor->input_vars.ADC1_req + _motor->input_vars.ADC2_req +
./MESC_Common/Src/MESCfoc.c:1410:					  	  	    _motor->input_vars.RCPWM_req + _motor->input_vars.UART_req + _motor->input_vars.ADC12_diff_req +
./MESC_Common/Src/MESCfoc.c:1411:								_motor->input_vars.remote_ADC1_req + _motor->input_vars.remote_ADC2_req;
./MESC_Common/Src/MESCfoc.c:1414:				  _motor->FOC.Duty_scaler = fabsf(total_in); //Assign the duty here
./MESC_Common/Src/MESCfoc.c:1417:				  _motor->FOC.Duty_scaler = fabsf(total_in);
./MESC_Common/Src/MESCfoc.c:1421:			_motor->MotorSensorMode = MOTOR_SENSOR_MODE_OPENLOOP;
./MESC_Common/Src/MESCfoc.c:1422:			_motor->HFI.Type = HFI_TYPE_NONE;
./MESC_Common/Src/MESCfoc.c:1423:			_motor->FOC.Id_pgain = 0.0f;
./MESC_Common/Src/MESCfoc.c:1424:			_motor->FOC.Iq_pgain = 0.0f;
./MESC_Common/Src/MESCfoc.c:1425:			_motor->FOC.Id_igain = 0.0f;
./MESC_Common/Src/MESCfoc.c:1426:			_motor->FOC.Iq_igain = 0.0f;
./MESC_Common/Src/MESCfoc.c:1427:			_motor->FOC.openloop_step = (uint16_t)(600.0f*65536/_motor->FOC.pwm_frequency);//300Hz tone
./MESC_Common/Src/MESCfoc.c:1428:			_motor->FOC.Idq_int_err.d = 10.0f;//1V
./MESC_Common/Src/MESCfoc.c:1429:			_motor->FOC.Idq_int_err.q = 0.0f;//1V
./MESC_Common/Src/MESCfoc.c:1430:			_motor->FOC.Current_bandwidth = 0.0f;
./MESC_Common/Src/MESCfoc.c:1431:			_motor->FOC.PLL_int = 0.0f;
./MESC_Common/Src/MESCfoc.c:1432:			_motor->FOC.PLL_ki = 0.0f;
./MESC_Common/Src/MESCfoc.c:1433:			_motor->FOC.PLL_ki = 0.0f;
./MESC_Common/Src/MESCfoc.c:1434:			_motor->FOC.PLL_error = 0.0f;
./MESC_Common/Src/MESCfoc.c:1436:			_motor->m.R =10.0f*_motor->FOC.Idq_smoothed.d /(_motor->FOC.Idq_smoothed.d*_motor->FOC.Idq_smoothed.d +
./MESC_Common/Src/MESCfoc.c:1437:					_motor->FOC.Idq_smoothed.q*_motor->FOC.Idq_smoothed.q);
./MESC_Common/Src/MESCfoc.c:1438:			_motor->m.L_D = -10.0f*_motor->FOC.Idq_smoothed.q/(2.0f*3.1415f*600.0f*(_motor->FOC.Idq_smoothed.d*_motor->FOC.Idq_smoothed.d +
./MESC_Common/Src/MESCfoc.c:1439:					_motor->FOC.Idq_smoothed.q*_motor->FOC.Idq_smoothed.q));
./MESC_Common/Src/MESCfoc.c:1440:			if(_motor->MotorState !=MOTOR_STATE_ERROR){
./MESC_Common/Src/MESCfoc.c:1441:				_motor->MotorState = MOTOR_STATE_RUN;
./MESC_Common/Src/MESCfoc.c:1445:			  if((_motor->MotorState==MOTOR_STATE_RUN)||(_motor->MotorState==MOTOR_STATE_TRACKING)){
./MESC_Common/Src/MESCfoc.c:1446:				  if((fabsf(_motor->FOC.Vdq.q)<0.1f*_motor->Conv.Vbus)){//Check it is not error or spinning fast!
./MESC_Common/Src/MESCfoc.c:1447:					  _motor->MotorState = MOTOR_STATE_SLAMBRAKE;
./MESC_Common/Src/MESCfoc.c:1449:					  _motor->MotorState = MOTOR_STATE_TRACKING;
./MESC_Common/Src/MESCfoc.c:1453:			  float req_now = (_motor->input_vars.UART_req + _motor->input_vars.max_request_Idq.q * (_motor->input_vars.ADC1_req + _motor->input_vars.ADC2_req + _motor->input_vars.RCPWM_req));
./MESC_Common/Src/MESCfoc.c:1455:			  _motor->FOC.Idq_prereq.q = req_now;
./MESC_Common/Src/MESCfoc.c:1456:			  if((req_now>(0.05f*_motor->input_vars.max_request_Idq.q))&&(req_now>_motor->FOC.park_current_now)&&(_motor->MotorState == MOTOR_STATE_SLAMBRAKE)){
./MESC_Common/Src/MESCfoc.c:1457:				  _motor->MotorState = MOTOR_STATE_TRACKING;
./MESC_Common/Src/MESCfoc.c:1458:				  _motor->ControlMode = MOTOR_CONTROL_MODE_TORQUE;
./MESC_Common/Src/MESCfoc.c:1468:	  if((_motor->key_bits)){
./MESC_Common/Src/MESCfoc.c:1469:		  _motor->FOC.Idq_prereq.q = 0.0f;
./MESC_Common/Src/MESCfoc.c:1470:		  _motor->FOC.Idq_prereq.d = 0.0f;
./MESC_Common/Src/MESCfoc.c:1473:	switch(_motor->MotorState){
./MESC_Common/Src/MESCfoc.c:1476:			_motor->FOC.was_last_tracking = 1;
./MESC_Common/Src/MESCfoc.c:1479:			if(_motor->ControlMode == MOTOR_CONTROL_MODE_TORQUE){
./MESC_Common/Src/MESCfoc.c:1480:				if(MESCinput_isHandbrake()){_motor->ControlMode = MOTOR_CONTROL_MODE_HANDBRAKE;}
./MESC_Common/Src/MESCfoc.c:1481:				if(fabsf(_motor->FOC.Idq_prereq.q)>0.2f){
./MESC_Common/Src/MESCfoc.c:1483:					if(_motor->MotorControlType == MOTOR_CONTROL_TYPE_FOC){
./MESC_Common/Src/MESCfoc.c:1484:						_motor->MotorState = MOTOR_STATE_RUN;
./MESC_Common/Src/MESCfoc.c:1485:					}else if(_motor->MotorControlType == MOTOR_CONTROL_TYPE_BLDC){
./MESC_Common/Src/MESCfoc.c:1486:						_motor->MotorState = MOTOR_STATE_RUN_BLDC;
./MESC_Common/Src/MESCfoc.c:1490:					_motor->MotorState = MOTOR_STATE_RECOVERING;
./MESC_Common/Src/MESCfoc.c:1498:			} else if(_motor->ControlMode == MOTOR_CONTROL_MODE_POSITION){
./MESC_Common/Src/MESCfoc.c:1499:				if(_motor->MotorState!=MOTOR_STATE_ERROR){
./MESC_Common/Src/MESCfoc.c:1500:					_motor->MotorState = MOTOR_STATE_RUN;
./MESC_Common/Src/MESCfoc.c:1502:			}else if(_motor->ControlMode == MOTOR_CONTROL_MODE_SPEED){
./MESC_Common/Src/MESCfoc.c:1503:					if(_motor->FOC.speed_req > 10.0f){
./MESC_Common/Src/MESCfoc.c:1504:						_motor->MotorState = MOTOR_STATE_RUN;
./MESC_Common/Src/MESCfoc.c:1509:			}else if(_motor->ControlMode == MOTOR_CONTROL_MODE_DUTY){
./MESC_Common/Src/MESCfoc.c:1510:				if(_motor->FOC.Duty_scaler > 0.01f){
./MESC_Common/Src/MESCfoc.c:1511:					_motor->MotorState = MOTOR_STATE_RUN;
./MESC_Common/Src/MESCfoc.c:1522:			if(_motor->options.MTPA_mode){
./MESC_Common/Src/MESCfoc.c:1529:			if(_motor->options.use_lr_observer){
./MESC_Common/Src/MESCfoc.c:1534:			_motor->FOC.Idq_req.q = _motor->FOC.Idq_prereq.q;
./MESC_Common/Src/MESCfoc.c:1535:			_motor->FOC.Idq_req.d = _motor->FOC.Idq_prereq.d;
./MESC_Common/Src/MESCfoc.c:1536:			if(_motor->input_vars.UART_dreq){_motor->FOC.Idq_req.d = _motor->input_vars.UART_dreq;}//Override the calcs if a specific d is requested
./MESC_Common/Src/MESCfoc.c:1538:			switch(_motor->ControlMode){
./MESC_Common/Src/MESCfoc.c:1540:					if(((fabsf(_motor->FOC.Idq_prereq.q)<0.1f))){//Request current small, FW not active
./MESC_Common/Src/MESCfoc.c:1541:						if((_motor->FOC.FW_current>-0.5f)){
./MESC_Common/Src/MESCfoc.c:1542:						_motor->MotorState = MOTOR_STATE_TRACKING;
./MESC_Common/Src/MESCfoc.c:1548:					if(MESCinput_isHandbrake()){_motor->ControlMode = MOTOR_CONTROL_MODE_HANDBRAKE;}
./MESC_Common/Src/MESCfoc.c:1552:					if(fabsf(_motor->FOC.speed_req) < 10.0f){
./MESC_Common/Src/MESCfoc.c:1553:						_motor->MotorState = MOTOR_STATE_TRACKING;
./MESC_Common/Src/MESCfoc.c:1558:					if(_motor->FOC.Duty_scaler > 0.01f){
./MESC_Common/Src/MESCfoc.c:1561:						_motor->MotorState = MOTOR_STATE_TRACKING;
./MESC_Common/Src/MESCfoc.c:1575:			_motor->BLDC.I_set = _motor->FOC.Idq_prereq.q;
./MESC_Common/Src/MESCfoc.c:1580:			switch(_motor->ControlMode){
./MESC_Common/Src/MESCfoc.c:1584:				if(fabsf(_motor->FOC.Idq_prereq.q)<0.1f){
./MESC_Common/Src/MESCfoc.c:1585:					_motor->MotorState = MOTOR_STATE_TRACKING;
./MESC_Common/Src/MESCfoc.c:1591:					if(fabsf(_motor->FOC.speed_req)<0.1f){
./MESC_Common/Src/MESCfoc.c:1592:						_motor->MotorState = MOTOR_STATE_TRACKING;
./MESC_Common/Src/MESCfoc.c:1597:					if(fabsf(_motor->FOC.Duty_scaler)<0.01f){
./MESC_Common/Src/MESCfoc.c:1598:						_motor->MotorState = MOTOR_STATE_TRACKING;
./MESC_Common/Src/MESCfoc.c:1627:	_motor->offset.Iu = 0.9999f*_motor->offset.Iu +0.0001f*(float)_motor->Raw.Iu;
./MESC_Common/Src/MESCfoc.c:1628:	_motor->offset.Iv = 0.9999f*_motor->offset.Iv +0.0001f*(float)_motor->Raw.Iv;
./MESC_Common/Src/MESCfoc.c:1629:	_motor->offset.Iw = 0.9999f*_motor->offset.Iw +0.0001f*(float)_motor->Raw.Iw;
./MESC_Common/Src/MESCfoc.c:1633:	_motor->FOC.Vab.a =
./MESC_Common/Src/MESCfoc.c:1634:		0.666f * (_motor->Conv.Vu -
./MESC_Common/Src/MESCfoc.c:1635:				  0.5f * ((_motor->Conv.Vv) +
./MESC_Common/Src/MESCfoc.c:1636:						  (_motor->Conv.Vw)));
./MESC_Common/Src/MESCfoc.c:1637:	_motor->FOC.Vab.b =
./MESC_Common/Src/MESCfoc.c:1639:		(sqrt3_on_2 * ((_motor->Conv.Vv) -
./MESC_Common/Src/MESCfoc.c:1640:					   (_motor->Conv.Vw)));
./MESC_Common/Src/MESCfoc.c:1642:	sin_cos_fast(_motor->FOC.FOCAngle, &_motor->FOC.sincosangle.sin, &_motor->FOC.sincosangle.cos);
./MESC_Common/Src/MESCfoc.c:1646:	_motor->FOC.Vdq.d = _motor->FOC.sincosangle.cos * _motor->FOC.Vab.a +
./MESC_Common/Src/MESCfoc.c:1647:					  _motor->FOC.sincosangle.sin * _motor->FOC.Vab.b;
./MESC_Common/Src/MESCfoc.c:1648:	_motor->FOC.Vdq.q = _motor->FOC.sincosangle.cos * _motor->FOC.Vab.b -
./MESC_Common/Src/MESCfoc.c:1649:					  _motor->FOC.sincosangle.sin * _motor->FOC.Vab.a;
./MESC_Common/Src/MESCfoc.c:1650:	_motor->FOC.Idq_int_err.q = _motor->FOC.Vdq.q;
./MESC_Common/Src/MESCfoc.c:1651:	_motor->FOC.Idq_int_err.d = _motor->FOC.Vdq.d;
./MESC_Common/Src/MESCfoc.c:1680:	  		if(countdown == 1||(((_motor->FOC.Iab.a*_motor->FOC.Iab.a+_motor->FOC.Iab.b*_motor->FOC.Iab.b)>DEADSHORT_CURRENT*DEADSHORT_CURRENT)&&countdown<9))
./MESC_Common/Src/MESCfoc.c:1685:	  					IacalcDS = _motor->FOC.Iab.a;
./MESC_Common/Src/MESCfoc.c:1686:	  					IbcalcDS = _motor->FOC.Iab.b;
./MESC_Common/Src/MESCfoc.c:1687:	  					VacalcDS = -_motor->m.L_D*_motor->FOC.Iab.a/((9.0f-(float)countdown)*_motor->FOC.pwm_period);
./MESC_Common/Src/MESCfoc.c:1688:	  					VbcalcDS = -_motor->m.L_D*_motor->FOC.Iab.b/((9.0f-(float)countdown)*_motor->FOC.pwm_period);
./MESC_Common/Src/MESCfoc.c:1696:	  					_motor->FOC.FOCAngle = angleDS;//
./MESC_Common/Src/MESCfoc.c:1697:	  					sin_cos_fast(_motor->FOC.FOCAngle, &_motor->FOC.sincosangle.sin, &_motor->FOC.sincosangle.cos);
./MESC_Common/Src/MESCfoc.c:1700:	  					VdcalcDS = _motor->FOC.sincosangle.cos * VacalcDS +
./MESC_Common/Src/MESCfoc.c:1701:	  				                      _motor->FOC.sincosangle.sin * VbcalcDS;
./MESC_Common/Src/MESCfoc.c:1702:	  					VqcalcDS = _motor->FOC.sincosangle.cos * VbcalcDS -
./MESC_Common/Src/MESCfoc.c:1703:	  				                      _motor->FOC.sincosangle.sin * VacalcDS;
./MESC_Common/Src/MESCfoc.c:1705:	  					FLaDS = _motor->FOC.flux_observed*_motor->FOC.sincosangle.cos;
./MESC_Common/Src/MESCfoc.c:1706:	  					FLbDS = _motor->FOC.flux_observed*_motor->FOC.sincosangle.sin;
./MESC_Common/Src/MESCfoc.c:1708:	  					angleErrorDSENC = angleDS-_motor->FOC.enc_angle;
./MESC_Common/Src/MESCfoc.c:1710:	  					_motor->FOC.flux_a = FLaDS;
./MESC_Common/Src/MESCfoc.c:1711:	  					_motor->FOC.flux_b = FLbDS;
./MESC_Common/Src/MESCfoc.c:1712:	  					_motor->FOC.Ia_last = 0.0f;
./MESC_Common/Src/MESCfoc.c:1713:	  					_motor->FOC.Ib_last = 0.0f;
./MESC_Common/Src/MESCfoc.c:1714:	  					_motor->FOC.Idq_int_err.d = VdcalcDS;
./MESC_Common/Src/MESCfoc.c:1715:	  					_motor->FOC.Idq_int_err.q = VqcalcDS;
./MESC_Common/Src/MESCfoc.c:1723:	  			_motor->mtimer->Instance->CCR1 = 50;
./MESC_Common/Src/MESCfoc.c:1724:	  			_motor->mtimer->Instance->CCR2 = 50;
./MESC_Common/Src/MESCfoc.c:1725:	  			_motor->mtimer->Instance->CCR3 = 50;
./MESC_Common/Src/MESCfoc.c:1729:	  			_motor->mtimer->Instance->CCR1 = 50;
./MESC_Common/Src/MESCfoc.c:1730:	  			_motor->mtimer->Instance->CCR2 = 50;
./MESC_Common/Src/MESCfoc.c:1731:	  			_motor->mtimer->Instance->CCR3 = 50;
./MESC_Common/Src/MESCfoc.c:1737:  					_motor->MotorState = MOTOR_STATE_RUN;
./MESC_Common/Src/MESCfoc.c:1825:      	  _motor->FOC.enc_angle = -_motor->m.pole_pairs*((pkt.angle *2)%_motor->m.pole_angle)-_motor->FOC.enc_offset;
./MESC_Common/Src/MESCfoc.c:1827:      _motor->FOC.enc_angle = _motor->m.pole_pairs*((pkt.angle *2)%_motor->m.pole_angle)-_motor->FOC.enc_offset;
./MESC_Common/Src/MESCfoc.c:1838:	if(fabsf(_motor->FOC.Vdq.q)>MIN_HALL_FLUX_VOLTS){ //Are we actually spinning at a reasonable pace?
./MESC_Common/Src/MESCfoc.c:1839:		if((_motor->hall.current_hall_state>0)&&(_motor->hall.current_hall_state<7)){
./MESC_Common/Src/MESCfoc.c:1840:			_motor->m.hall_flux[_motor->hall.current_hall_state - 1][0] = 	0.999f*_motor->m.hall_flux[_motor->hall.current_hall_state - 1][0] +
./MESC_Common/Src/MESCfoc.c:1841:																			0.001f*_motor->FOC.flux_a;
./MESC_Common/Src/MESCfoc.c:1845:			_motor->m.hall_flux[_motor->hall.current_hall_state - 1][1] = 	0.999f*_motor->m.hall_flux[_motor->hall.current_hall_state - 1][1] +
./MESC_Common/Src/MESCfoc.c:1846:																			0.001f*_motor->FOC.flux_b;
./MESC_Common/Src/MESCfoc.c:1848:		_motor->FOC.hall_initialised = 1;
./MESC_Common/Src/MESCfoc.c:1852:	if(_motor->FOC.encoder_polarity_invert){
./MESC_Common/Src/MESCfoc.c:1853:		_motor->FOC.enc_angle = _motor->m.pole_pairs*(65536-(_motor->FOC.enc_ratio*(uint16_t)_motor->enctimer->Instance->CNT-_motor->FOC.enc_ratio*(uint16_t)_motor->enctimer->Instance->CCR3)) + _motor->FOC.enc_offset;
./MESC_Common/Src/MESCfoc.c:1855:		_motor->FOC.enc_angle = _motor->m.pole_pairs*((_motor->FOC.enc_ratio*(uint16_t)_motor->enctimer->Instance->CNT-_motor->FOC.enc_ratio*(uint16_t)_motor->enctimer->Instance->CCR3)) + _motor->FOC.enc_offset;
./MESC_Common/Src/MESCfoc.c:1859:	_motor->FOC.abs_position = ( (uint16_t)(_motor->enctimer->Instance->CNT) - (uint16_t)(_motor->enctimer->Instance->CCR3) ) & 0x0FFF; // for 12-bit encoder
./MESC_Common/Src/MESCfoc.c:1865:	_motor->logging.Vbus[_motor->logging.current_sample] = _motor->Conv.Vbus;
./MESC_Common/Src/MESCfoc.c:1866:	_motor->logging.Iu[_motor->logging.current_sample] = _motor->Conv.Iu;
./MESC_Common/Src/MESCfoc.c:1867:	_motor->logging.Iv[_motor->logging.current_sample] = _motor->Conv.Iv;
./MESC_Common/Src/MESCfoc.c:1868:	_motor->logging.Iw[_motor->logging.current_sample] = _motor->Conv.Iw;
./MESC_Common/Src/MESCfoc.c:1869:	_motor->logging.Vd[_motor->logging.current_sample] = _motor->FOC.Vdq.d;
./MESC_Common/Src/MESCfoc.c:1870:	_motor->logging.Vq[_motor->logging.current_sample] = _motor->FOC.Vdq.q;
./MESC_Common/Src/MESCfoc.c:1871:	_motor->logging.angle[_motor->logging.current_sample] = _motor->FOC.FOCAngle;
./MESC_Common/Src/MESCfoc.c:1872:	_motor->logging.hallstate[_motor->logging.current_sample] = (uint16_t)_motor->hall.current_hall_state;
./MESC_Common/Src/MESCfoc.c:1873:	if(_motor->MotorSensorMode == MOTOR_SENSOR_MODE_INCREMENTAL_ENCODER){
./MESC_Common/Src/MESCfoc.c:1874:		_motor->logging.hallstate[_motor->logging.current_sample] = (uint16_t)_motor->enctimer->Instance->CCR3;
./MESC_Common/Src/MESCfoc.c:1876:	_motor->logging.current_sample++;
./MESC_Common/Src/MESCfoc.c:1877:	if(_motor->logging.current_sample>=LOGLENGTH){
./MESC_Common/Src/MESCfoc.c:1878:		_motor->logging.current_sample = 0;
./MESC_Common/Src/MESCfoc.c:1888:	switch(_motor->SLStartupSensor){
./MESC_Common/Src/MESCfoc.c:1890:		if((fabsf(_motor->FOC.Vdq.q-_motor->m.R*_motor->FOC.Idq_smoothed.q)<_motor->FOC.hall_transition_V)&&(_motor->FOC.hall_initialised)&&(_motor->hall.current_hall_state>0)&&(_motor->hall.current_hall_state<7)){
./MESC_Common/Src/MESCfoc.c:1891:				_motor->FOC.hall_start_now = 1;
./MESC_Common/Src/MESCfoc.c:1892:		}else if((fabsf(_motor->FOC.Vdq.q-_motor->m.R*_motor->FOC.Idq_smoothed.q)>_motor->FOC.hall_transition_V+2.0f)||(_motor->hall.current_hall_state<1)||(_motor->hall.current_hall_state>6)){
./MESC_Common/Src/MESCfoc.c:1893:			_motor->FOC.hall_start_now = 0;
./MESC_Common/Src/MESCfoc.c:1897:		if((fabsf(_motor->FOC.Vdq.q-_motor->m.R*_motor->FOC.Idq_smoothed.q)<_motor->FOC.hall_transition_V)&&(_motor->FOC.encoder_OK)){
./MESC_Common/Src/MESCfoc.c:1898:				_motor->FOC.enc_start_now = 1;
./MESC_Common/Src/MESCfoc.c:1899:		}else if((fabsf(_motor->FOC.Vdq.q-_motor->m.R*_motor->FOC.Idq_smoothed.q)>_motor->FOC.hall_transition_V+2.0f)||!(_motor->FOC.encoder_OK)){
./MESC_Common/Src/MESCfoc.c:1900:			_motor->FOC.enc_start_now = 0;
./MESC_Common/Src/MESCfoc.c:1907:		_motor->FOC.hall_start_now = 0;
./MESC_Common/Src/MESCfoc.c:1908:		_motor->FOC.enc_start_now = 0;
./MESC_Common/Src/MESCfoc.c:1909:		_motor->HFI.inject = 0;
./MESC_Common/Src/MESCfoc.c:1918:	if(_motor->m.L_QD>0.0f){
./MESC_Common/Src/MESCfoc.c:1919:		switch(_motor->options.MTPA_mode){
./MESC_Common/Src/MESCfoc.c:1925:			i_mag = _motor->FOC.Idq_prereq.q;
./MESC_Common/Src/MESCfoc.c:1926://			_motor->FOC.id_mtpa = _motor->m.flux_linkage/(4.0f*_motor->m.L_QD) - sqrtf((_motor->m.flux_linkage*_motor->m.flux_linkage/(16.0f*_motor->m.L_QD*_motor->m.L_QD))+_motor->FOC.Idq_prereq.q*_motor->FOC.Idq_prereq.q*0.5f);
./MESC_Common/Src/MESCfoc.c:1930:			i_mag = sqrtf(_motor->FOC.Idq_smoothed.q * _motor->FOC.Idq_smoothed.q+_motor->FOC.Idq_smoothed.d * _motor->FOC.Idq_smoothed.d);
./MESC_Common/Src/MESCfoc.c:1934:			i_mag = _motor->FOC.Idq_smoothed.q;
./MESC_Common/Src/MESCfoc.c:1939:		_motor->FOC.id_mtpa = _motor->m.flux_linkage/(4.0f*_motor->m.L_QD) - sqrtf((_motor->m.flux_linkage*_motor->m.flux_linkage/(16.0f*_motor->m.L_QD*_motor->m.L_QD)) + (i_mag * i_mag) * 0.5f);
./MESC_Common/Src/MESCfoc.c:1941:		if(fabsf(_motor->FOC.Idq_prereq.q)>fabsf(_motor->FOC.id_mtpa)){
./MESC_Common/Src/MESCfoc.c:1942:			_motor->FOC.iq_mtpa = sqrtf(_motor->FOC.Idq_prereq.q * _motor->FOC.Idq_prereq.q - _motor->FOC.id_mtpa * _motor->FOC.id_mtpa);
./MESC_Common/Src/MESCfoc.c:1945:			_motor->FOC.iq_mtpa = 0.0f;
./MESC_Common/Src/MESCfoc.c:1948:		_motor->FOC.Idq_prereq.d = _motor->FOC.id_mtpa;
./MESC_Common/Src/MESCfoc.c:1950:		if(_motor->FOC.Idq_prereq.q>0.0f){
./MESC_Common/Src/MESCfoc.c:1951:			_motor->FOC.Idq_prereq.q = _motor->FOC.iq_mtpa;
./MESC_Common/Src/MESCfoc.c:1954:			_motor->FOC.Idq_prereq.q = -_motor->FOC.iq_mtpa;
./MESC_Common/Src/MESCfoc.c:1961:		_motor->FOC.currentPower.d = 1.5f*(_motor->FOC.Vdq.d*_motor->FOC.Idq_smoothed.d);
./MESC_Common/Src/MESCfoc.c:1962:		_motor->FOC.currentPower.q = 1.5f*(_motor->FOC.Vdq.q*_motor->FOC.Idq_smoothed.q);
./MESC_Common/Src/MESCfoc.c:1963:		_motor->FOC.Ibus = (_motor->FOC.currentPower.d + _motor->FOC.currentPower.q) /_motor->Conv.Vbus;
./MESC_Common/Src/MESCfoc.c:1969:    float mag = (Square(_motor->FOC.Idq_prereq.q) + Square(_motor->FOC.FW_current));
./MESC_Common/Src/MESCfoc.c:1970:    if(mag>Square(_motor->input_vars.max_request_Idq.q)){
./MESC_Common/Src/MESCfoc.c:1971:    	float Iqmax2 = Square(_motor->input_vars.max_request_Idq.q)-Square(_motor->FOC.FW_current);
./MESC_Common/Src/MESCfoc.c:1973:			if(_motor->FOC.Idq_prereq.q>0){
./MESC_Common/Src/MESCfoc.c:1974:				_motor->FOC.Idq_prereq.q = sqrtf(Iqmax2);
./MESC_Common/Src/MESCfoc.c:1976:				_motor->FOC.Idq_prereq.q = -sqrtf(Iqmax2);
./MESC_Common/Src/MESCfoc.c:1979:    		_motor->MotorState = MOTOR_STATE_ERROR;
./MESC_Common/Src/MESCfoc.c:1981:    		_motor->FOC.FW_current = 0.0f;
./MESC_Common/Src/MESCfoc.c:1990:    _motor->FOC.reqPower = 1.5f*fabsf(_motor->FOC.Vdq.q * _motor->FOC.Idq_prereq.q);
./MESC_Common/Src/MESCfoc.c:1991:    float batt_power_max = _motor->m.IBatmax*_motor->Conv.Vbus; //Calculate the max battery power allowed at current voltage
./MESC_Common/Src/MESCfoc.c:1992:    if(batt_power_max > _motor->m.Pmax){
./MESC_Common/Src/MESCfoc.c:1993:    	batt_power_max = _motor->m.Pmax;		//Replace batt_power with the lower power limit
./MESC_Common/Src/MESCfoc.c:1995:    if (_motor->FOC.reqPower > batt_power_max) {
./MESC_Common/Src/MESCfoc.c:1996:    	if(_motor->FOC.Idq_prereq.q > 0.0f){
./MESC_Common/Src/MESCfoc.c:1997:    		_motor->FOC.Idq_prereq.q = batt_power_max / (fabsf(_motor->FOC.Vdq.q)*1.5f);
./MESC_Common/Src/MESCfoc.c:1999:    		_motor->FOC.Idq_prereq.q = -batt_power_max / (fabsf(_motor->FOC.Vdq.q)*1.5f);
./MESC_Common/Src/MESCfoc.c:2009:	if ((_motor->FOC.flux_a * _motor->FOC.flux_a + _motor->FOC.flux_b * _motor->FOC.flux_b) <
./MESC_Common/Src/MESCfoc.c:2010:		0.25f * _motor->FOC.flux_observed * _motor->FOC.flux_observed) {
./MESC_Common/Src/MESCfoc.c:2011:		_motor->FOC.flux_a = 2.5f * _motor->FOC.flux_a;//_motor->FOC.flux_observed;
./MESC_Common/Src/MESCfoc.c:2012:		_motor->FOC.flux_b = 2.5f * _motor->FOC.flux_b;//_motor->FOC.flux_observed;
./MESC_Common/Src/MESCfoc.c:2018:	if(abs(_motor->FOC.PLL_int)>10000.0f){
./MESC_Common/Src/MESCfoc.c:2021:		_motor->FOC.PLL_int = 0;
./MESC_Common/Src/MESCfoc.c:2024:	if(_motor->m.pole_pairs>0){//avoid divide by zero
./MESC_Common/Src/MESCfoc.c:2025:	_motor->FOC.mechRPM = _motor->FOC.eHz*60.0f/(float)(_motor->m.pole_pairs);
./MESC_Common/Src/MESCfoc.c:2045:	if(_motor->FOC.Vdq.q <0.0f){
./MESC_Common/Src/MESCfoc.c:2046:		_motor->FOC.Idq_req.q = 0.2f; //Apply a brake current
./MESC_Common/Src/MESCfoc.c:2048:	if(_motor->FOC.Vdq.q >0.0f){
./MESC_Common/Src/MESCfoc.c:2049:		_motor->FOC.Idq_req.q = -0.2f; //Apply a brake current
./MESC_Common/Src/MESCfoc.c:2056:	TEMPState const temp_state = temp_check( &_motor->Raw.Motor_temp, T, &dT );
./MESC_Common/Src/MESCfoc.c:2070:	_motor->Conv.MOSu_T  = 0.99f *_motor->Conv.MOSu_T  + 0.01f * temp_read( &_motor->Raw.MOS_temp  , _motor->Raw.MOSu_T  );
./MESC_Common/Src/MESCfoc.c:2071:	_motor->Conv.MOSv_T  = 0.99f *_motor->Conv.MOSv_T  + 0.01f * temp_read( &_motor->Raw.MOS_temp  , _motor->Raw.MOSv_T  );
./MESC_Common/Src/MESCfoc.c:2072:	_motor->Conv.MOSw_T  = 0.99f *_motor->Conv.MOSw_T  + 0.01f * temp_read( &_motor->Raw.MOS_temp  , _motor->Raw.MOSw_T  );
./MESC_Common/Src/MESCfoc.c:2073:	_motor->Conv.Motor_T = 0.99f *_motor->Conv.Motor_T + 0.01f * temp_read( &_motor->Raw.Motor_temp, _motor->Raw.Motor_T );
./MESC_Common/Src/MESCfoc.c:2075:	handleThrottleTemperature( _motor, _motor->Conv.MOSu_T , &dTmax, ERROR_OVERTEMPU );
./MESC_Common/Src/MESCfoc.c:2076:	handleThrottleTemperature( _motor, _motor->Conv.MOSv_T , &dTmax, ERROR_OVERTEMPV );
./MESC_Common/Src/MESCfoc.c:2077:	handleThrottleTemperature( _motor, _motor->Conv.MOSw_T , &dTmax, ERROR_OVERTEMPW );
./MESC_Common/Src/MESCfoc.c:2078:	if(_motor->options.has_motor_temp_sensor){
./MESC_Common/Src/MESCfoc.c:2079:		handleThrottleTemperature( _motor, _motor->Conv.Motor_T, &dTmax, ERROR_OVERTEMP_MOTOR );
./MESC_Common/Src/MESCfoc.c:2082:	_motor->FOC.T_rollback = (1.0f-dTmax/(_motor->Raw.MOS_temp.limit.Tmax-_motor->Raw.MOS_temp.limit.Thot));
./MESC_Common/Src/MESCfoc.c:2083:	if(_motor->FOC.T_rollback<=0.0f){
./MESC_Common/Src/MESCfoc.c:2084:		_motor->FOC.T_rollback = 0.0f;
./MESC_Common/Src/MESCfoc.c:2086:	if(_motor->FOC.T_rollback>1.0f){
./MESC_Common/Src/MESCfoc.c:2087:		_motor->FOC.T_rollback = 1.0f;
./MESC_Common/Src/MESCfoc.c:2089:	if(_motor->FOC.Idq_prereq.q>(_motor->FOC.T_rollback * _motor->input_vars.max_request_Idq.q)){_motor->FOC.Idq_prereq.q = _motor->FOC.T_rollback * _motor->input_vars.max_request_Idq.q;}
./MESC_Common/Src/MESCfoc.c:2090:	if(_motor->FOC.Idq_prereq.q<(_motor->FOC.T_rollback * _motor->input_vars.min_request_Idq.q)){_motor->FOC.Idq_prereq.q = _motor->FOC.T_rollback * _motor->input_vars.min_request_Idq.q;}
./MESC_Common/Src/MESCfoc.c:2094:	if((_motor->FOC.Idq_req.q == 0.0f)&&(_motor->FOC.Idq_prereq.q == 0.0f)){
./MESC_Common/Src/MESCfoc.c:2095:		_motor->safe_start[1]++;
./MESC_Common/Src/MESCfoc.c:2096:	}else if(_motor->safe_start[1]<_motor->safe_start[0] ){
./MESC_Common/Src/MESCfoc.c:2097:		_motor->safe_start[1]=0;
./MESC_Common/Src/MESCfoc.c:2099:	if(_motor->safe_start[1] >=_motor->safe_start[0]){
./MESC_Common/Src/MESCfoc.c:2100:		_motor->safe_start[1] = _motor->safe_start[0];
./MESC_Common/Src/MESCfoc.c:2101:		_motor->key_bits &= ~SAFESTART_KEY;
./MESC_Common/Src/MESCfoc.c:2110:	  if(_motor->MotorState == MOTOR_STATE_RUN){
./MESC_Common/Src/MESCfoc.c:2112:		speed_error = _motor->FOC.speed_kp*(_motor->FOC.speed_req - _motor->FOC.eHz);
./MESC_Common/Src/MESCfoc.c:2115:		speed_error = clamp(speed_error, -_motor->input_vars.max_request_Idq.q, _motor->input_vars.max_request_Idq.q);
./MESC_Common/Src/MESCfoc.c:2117:		_motor->FOC.speed_error_int = _motor->FOC.speed_error_int + speed_error * _motor->FOC.speed_ki;
./MESC_Common/Src/MESCfoc.c:2120:		_motor->FOC.speed_error_int = clamp(_motor->FOC.speed_error_int, -_motor->input_vars.max_request_Idq.q, _motor->input_vars.max_request_Idq.q);
./MESC_Common/Src/MESCfoc.c:2123:		_motor->FOC.Idq_prereq.q = _motor->FOC.speed_error_int + speed_error;
./MESC_Common/Src/MESCfoc.c:2126:		_motor->FOC.Idq_prereq.q = clamp(_motor->FOC.Idq_prereq.q, _motor->input_vars.min_request_Idq.q, _motor->input_vars.max_request_Idq.q);
./MESC_Common/Src/MESCfoc.c:2130:		  _motor->FOC.Idq_prereq.q = 0.0f;
./MESC_Common/Src/MESCfoc.c:2131:		  _motor->FOC.speed_error_int = 0.0f;
./MESC_Common/Src/MESCfoc.c:2167:		_motor->input_vars.pulse_recieved = 1;
./MESC_Common/Src/MESCfoc.c:2168:		_motor->input_vars.IC_duration = CCR1;
./MESC_Common/Src/MESCfoc.c:2169:		_motor->input_vars.IC_pulse = CCR2;
./MESC_Common/Src/MESCfoc.c:2172:		_motor->input_vars.pulse_recieved = 0;
./MESC_Common/Src/MESCfoc.c:2179:		_motor->FOC.encoder_duration = CCR1;
./MESC_Common/Src/MESCfoc.c:2180:		_motor->FOC.encoder_pulse = CCR2;
./MESC_Common/Src/MESCfoc.c:2181:		_motor->FOC.encoder_OK = 1;
./MESC_Common/Src/MESCfoc.c:2190:		temp_enc_ang = _motor->FOC.enc_offset +
./MESC_Common/Src/MESCfoc.c:2191:						(uint16_t)(((65536*(CCR2-16))/(CCR1-24)*(uint32_t)_motor->m.pole_pairs)%65536);
./MESC_Common/Src/MESCfoc.c:2193:		if(_motor->FOC.encoder_polarity_invert){
./MESC_Common/Src/MESCfoc.c:2194:			_motor->FOC.last_enc_period = _motor->FOC.enc_period_count;
./MESC_Common/Src/MESCfoc.c:2195:			_motor->FOC.enc_period_count = 0;
./MESC_Common/Src/MESCfoc.c:2196:			_motor->FOC.enc_angle = 65536 - temp_enc_ang;
./MESC_Common/Src/MESCfoc.c:2198:			_motor->FOC.last_enc_period = _motor->FOC.enc_period_count;
./MESC_Common/Src/MESCfoc.c:2199:			_motor->FOC.enc_angle = temp_enc_ang;
./MESC_Common/Src/MESCfoc.c:2200:			_motor->FOC.enc_period_count = 0;
./MESC_Common/Src/MESCfoc.c:2204:		_motor->FOC.enc_pwm_step = 0.8f*_motor->FOC.enc_pwm_step +
./MESC_Common/Src/MESCfoc.c:2205:				0.2f*(((int16_t)(_motor->FOC.enc_angle - _motor->FOC.last_enc_angle))/(_motor->FOC.last_enc_period + 0.1f));
./MESC_Common/Src/MESCfoc.c:2206:		_motor->FOC.last_enc_angle = _motor->FOC.enc_angle;
./MESC_Common/Src/MESCfoc.c:2210:	sin_cos_fast((_motor->FOC.enc_angle), &_motor->FOC.encsin, &_motor->FOC.enccos);
./MESC_Common/Src/MESCfoc.c:2212:	if(SR & 0x1||_motor->FOC.encoder_pulse<14||_motor->FOC.encoder_pulse>(_motor->FOC.encoder_duration-7)){
./MESC_Common/Src/MESCfoc.c:2214:		_motor->FOC.encoder_OK = 0;
./MESC_Common/Src/MESCBLDC.c:237:	switch (_motor->BLDC.sector){
./MESC_Common/Src/MESCBLDC.c:239:		_motor->BLDC.I_meas = _motor->Conv.Iu;
./MESC_Common/Src/MESCBLDC.c:240://		_motor->BLDC.V_meas = _motor->Conv.Vv;
./MESC_Common/Src/MESCBLDC.c:241://		_motor->BLDC.rising_int = _motor->BLDC.rising_int + _motor->BLDC.V_meas*_motor->BLDC.PWM_period;
./MESC_Common/Src/MESCBLDC.c:245:			_motor->BLDC.I_meas = _motor->Conv.Iu;
./MESC_Common/Src/MESCBLDC.c:246://			_motor->BLDC.V_meas = _motor->Conv.Vw;
./MESC_Common/Src/MESCBLDC.c:247://			_motor->BLDC.falling_int = _motor->BLDC.rising_int + _motor->BLDC.V_meas*_motor->BLDC.PWM_period;
./MESC_Common/Src/MESCBLDC.c:251:			_motor->BLDC.I_meas = _motor->Conv.Iw;
./MESC_Common/Src/MESCBLDC.c:252://			_motor->BLDC.V_meas = _motor->Conv.Vu;
./MESC_Common/Src/MESCBLDC.c:253://			_motor->BLDC.rising_int = _motor->BLDC.rising_int + _motor->BLDC.V_meas*_motor->BLDC.PWM_period;
./MESC_Common/Src/MESCBLDC.c:257:			_motor->BLDC.I_meas = _motor->Conv.Iw;
./MESC_Common/Src/MESCBLDC.c:258://			_motor->BLDC.V_meas = _motor->Conv.Vv;
./MESC_Common/Src/MESCBLDC.c:259://			_motor->BLDC.falling_int = _motor->BLDC.rising_int + _motor->BLDC.V_meas*_motor->BLDC.PWM_period;
./MESC_Common/Src/MESCBLDC.c:263:			_motor->BLDC.I_meas = _motor->Conv.Iv;
./MESC_Common/Src/MESCBLDC.c:264://			_motor->BLDC.V_meas = _motor->Conv.Vw;
./MESC_Common/Src/MESCBLDC.c:265://			_motor->BLDC.rising_int = _motor->BLDC.rising_int + _motor->BLDC.V_meas*_motor->BLDC.PWM_period;
./MESC_Common/Src/MESCBLDC.c:269:			_motor->BLDC.I_meas = _motor->Conv.Iv;
./MESC_Common/Src/MESCBLDC.c:270://			_motor->BLDC.V_meas = _motor->Conv.Vu;
./MESC_Common/Src/MESCBLDC.c:271://			_motor->BLDC.falling_int = _motor->BLDC.rising_int + _motor->BLDC.V_meas*_motor->BLDC.PWM_period;
./MESC_Common/Src/MESCBLDC.c:275://	_motor->BLDC.rising_int = _motor->BLDC.rising_int * 0.999f;
./MESC_Common/Src/MESCBLDC.c:276://	_motor->BLDC.falling_int = _motor->BLDC.rising_int * 0.999f;
./MESC_Common/Src/MESCBLDC.c:279:	_motor->BLDC.I_meas = -_motor->BLDC.I_meas;
./MESC_Common/Src/MESCBLDC.c:282:	_motor->BLDC.I_pgain = _motor->FOC.Iq_pgain;//Borrow from FOC for now
./MESC_Common/Src/MESCBLDC.c:283:	_motor->BLDC.I_igain = _motor->FOC.Iq_igain;//Borrow from FOC for now
./MESC_Common/Src/MESCBLDC.c:284:	_motor->BLDC.PWM_period = _motor->FOC.pwm_period;//Borrow from FOC for now
./MESC_Common/Src/MESCBLDC.c:287:	_motor->BLDC.I_error = (_motor->BLDC.I_set-_motor->BLDC.I_meas)*_motor->BLDC.I_pgain;
./MESC_Common/Src/MESCBLDC.c:288:	_motor->BLDC.int_I_error = //Calculate the integral error
./MESC_Common/Src/MESCBLDC.c:289:			_motor->BLDC.int_I_error + _motor->BLDC.I_error * _motor->BLDC.I_igain * _motor->BLDC.PWM_period;
./MESC_Common/Src/MESCBLDC.c:291:	_motor->BLDC.V_bldc = _motor->BLDC.int_I_error + _motor->BLDC.I_error;
./MESC_Common/Src/MESCBLDC.c:293:	if(_motor->BLDC.V_bldc > 0.95f * _motor->Conv.Vbus){
./MESC_Common/Src/MESCBLDC.c:294:		_motor->BLDC.V_bldc = 0.95f * _motor->Conv.Vbus;
./MESC_Common/Src/MESCBLDC.c:295:		if(_motor->BLDC.int_I_error > _motor->Conv.Vbus){
./MESC_Common/Src/MESCBLDC.c:296:			_motor->BLDC.int_I_error = _motor->Conv.Vbus;
./MESC_Common/Src/MESCBLDC.c:297:			_motor->BLDC.I_error = 0.05f*_motor->BLDC.int_I_error;
./MESC_Common/Src/MESCBLDC.c:301:	_motor->BLDC.V_bldc_to_PWM = _motor->mtimer->Instance->ARR/_motor->Conv.Vbus;
./MESC_Common/Src/MESCBLDC.c:303:	_motor->BLDC.BLDC_PWM = _motor->BLDC.V_bldc*_motor->BLDC.V_bldc_to_PWM;
./MESC_Common/Src/MESCBLDC.c:307:	_motor->BLDC.flux_integral = _motor->BLDC.flux_integral + (_motor->BLDC.V_bldc - _motor->BLDC.I_meas * 2.0f*_motor->m.R)* _motor->BLDC.PWM_period; //Volt seconds
./MESC_Common/Src/MESCBLDC.c:310:	_motor->BLDC.closed_loop = 1;
./MESC_Common/Src/MESCBLDC.c:311:	if(_motor->BLDC.closed_loop){
./MESC_Common/Src/MESCBLDC.c:313:		if(_motor->BLDC.flux_integral<0.0f){_motor->BLDC.flux_integral = 0.0f;}
./MESC_Common/Src/MESCBLDC.c:314:		if(_motor->BLDC.flux_integral>_motor->BLDC.com_flux){
./MESC_Common/Src/MESCBLDC.c:315:			_motor->BLDC.V_meas_sect[_motor->BLDC.sector] = _motor->BLDC.V_meas;
./MESC_Common/Src/MESCBLDC.c:317:			_motor->BLDC.sector = _motor->BLDC.sector + _motor->BLDC.direction;
./MESC_Common/Src/MESCBLDC.c:318:			_motor->BLDC.last_flux_integral = _motor->BLDC.flux_integral;
./MESC_Common/Src/MESCBLDC.c:319:			_motor->BLDC.flux_integral = 0.0f; //Reset the integrator
./MESC_Common/Src/MESCBLDC.c:320:			_motor->BLDC.last_p_error = _motor->BLDC.I_error;
./MESC_Common/Src/MESCBLDC.c:322:			if((_motor->BLDC.int_I_error>_motor->Conv.Vbus*0.4f) && (_motor->BLDC.int_I_error<_motor->Conv.Vbus*0.9f)){
./MESC_Common/Src/MESCBLDC.c:323:				if(_motor->BLDC.I_error>0.05f*_motor->BLDC.int_I_error){
./MESC_Common/Src/MESCBLDC.c:324:					_motor->BLDC.com_flux = _motor->BLDC.com_flux*1.005f;
./MESC_Common/Src/MESCBLDC.c:326:				if(_motor->BLDC.I_error<0.05f*_motor->BLDC.int_I_error){
./MESC_Common/Src/MESCBLDC.c:327:					_motor->BLDC.com_flux = _motor->BLDC.com_flux*0.99f;
./MESC_Common/Src/MESCBLDC.c:331://			_motor->BLDC.rising_int_st =_motor->BLDC.rising_int;
./MESC_Common/Src/MESCBLDC.c:332://			_motor->BLDC.rising_int = 0.0f;
./MESC_Common/Src/MESCBLDC.c:333://			_motor->BLDC.falling_int_st = _motor->BLDC.falling_int;
./MESC_Common/Src/MESCBLDC.c:334://			_motor->BLDC.falling_int = 0.0f;
./MESC_Common/Src/MESCBLDC.c:335://			if(_motor->BLDC.falling_int_st > _motor->BLDC.falling_int_st){
./MESC_Common/Src/MESCBLDC.c:336://				_motor->BLDC.com_flux = _motor->BLDC.com_flux * 1.01f;
./MESC_Common/Src/MESCBLDC.c:338://				_motor->BLDC.com_flux = _motor->BLDC.com_flux * 0.99f;
./MESC_Common/Src/MESCBLDC.c:341://			if(_motor->BLDC.com_flux<0.018f){_motor->BLDC.com_flux=0.018f;}
./MESC_Common/Src/MESCBLDC.c:342://			if(_motor->BLDC.com_flux>0.022f){_motor->BLDC.com_flux=0.022f;}
./MESC_Common/Src/MESCBLDC.c:346:		_motor->BLDC.OL_countdown--;
./MESC_Common/Src/MESCBLDC.c:347:		if(_motor->BLDC.OL_countdown == 0){
./MESC_Common/Src/MESCBLDC.c:348:			_motor->BLDC.OL_countdown =_motor->BLDC.OL_periods;
./MESC_Common/Src/MESCBLDC.c:349:			_motor->BLDC.sector++;
./MESC_Common/Src/MESCBLDC.c:350:			_motor->BLDC.last_flux_integral = _motor->BLDC.flux_integral;
./MESC_Common/Src/MESCBLDC.c:351:			_motor->BLDC.flux_integral = 0.0f; //Reset the integrator
./MESC_Common/Src/MESCBLDC.c:356:	if(_motor->BLDC.sector>5){
./MESC_Common/Src/MESCBLDC.c:357:		_motor->BLDC.sector = 0;
./MESC_Common/Src/MESCBLDC.c:358:	}else if(_motor->BLDC.sector<0){
./MESC_Common/Src/MESCBLDC.c:359:		_motor->BLDC.sector = 5;
./MESC_Common/Src/MESCBLDC.c:363:	switch (_motor->BLDC.sector){
./MESC_Common/Src/MESCBLDC.c:368:			_motor->mtimer->Instance->CCR1 = 0;
./MESC_Common/Src/MESCBLDC.c:369:			_motor->mtimer->Instance->CCR3 = _motor->BLDC.BLDC_PWM;
./MESC_Common/Src/MESCBLDC.c:375:			_motor->mtimer->Instance->CCR1 = 0;
./MESC_Common/Src/MESCBLDC.c:376:			_motor->mtimer->Instance->CCR2 = _motor->BLDC.BLDC_PWM;
./MESC_Common/Src/MESCBLDC.c:382:			_motor->mtimer->Instance->CCR3 = 0;
./MESC_Common/Src/MESCBLDC.c:383:			_motor->mtimer->Instance->CCR2 = _motor->BLDC.BLDC_PWM;
./MESC_Common/Src/MESCBLDC.c:389:			_motor->mtimer->Instance->CCR3 = 0;
./MESC_Common/Src/MESCBLDC.c:390:			_motor->mtimer->Instance->CCR1 = _motor->BLDC.BLDC_PWM;
./MESC_Common/Src/MESCBLDC.c:396:			_motor->mtimer->Instance->CCR2 = 0;
./MESC_Common/Src/MESCBLDC.c:397:			_motor->mtimer->Instance->CCR1 = _motor->BLDC.BLDC_PWM;
./MESC_Common/Src/MESCBLDC.c:403:			_motor->mtimer->Instance->CCR2 = 0;
./MESC_Common/Src/MESCBLDC.c:404:			_motor->mtimer->Instance->CCR3 = _motor->BLDC.BLDC_PWM;
./MESC_Common/Src/MESCBLDC.c:408:			_motor->BLDC.sector = 0;
./MESC_Common/Src/MESCposition.c:14:	_motor->pos.set_position += 1;
./MESC_Common/Src/MESCposition.c:15://	if(_motor->pos.set_position>200000){_motor->pos.set_position = 100000;}
./MESC_Common/Src/MESCposition.c:16://	if(_motor->pos.set_position<200000){_motor->pos.set_position = _motor->pos.set_position+1500;}
./MESC_Common/Src/MESCposition.c:17:_motor->FOC.Idq_prereq.d = 2.0f;
./MESC_Common/Src/MESCposition.c:19:	_motor->pos.error = (float)(int)(_motor->pos.set_position - _motor->FOC.PLL_angle);
./MESC_Common/Src/MESCposition.c:20:	_motor->pos.p_error = _motor->pos.Kp * _motor->pos.error;
./MESC_Common/Src/MESCposition.c:21:	_motor->pos.int_error = _motor->pos.int_error + _motor->pos.Ki * _motor->pos.p_error;
./MESC_Common/Src/MESCposition.c:23:	_motor->pos.d_pos = (float)(int)(_motor->FOC.PLL_angle - _motor->pos.last_pll_pos);
./MESC_Common/Src/MESCposition.c:24:	_motor->pos.last_pll_pos = _motor->FOC.PLL_angle;
./MESC_Common/Src/MESCposition.c:25:	_motor->pos.d_error = -_motor->pos.Kd * _motor->pos.d_pos;
./MESC_Common/Src/MESCposition.c:27:	if(_motor->pos.int_error>_motor->input_vars.max_request_Idq.q){_motor->pos.int_error = _motor->input_vars.max_request_Idq.q;}
./MESC_Common/Src/MESCposition.c:28:	if(_motor->pos.int_error<_motor->input_vars.min_request_Idq.q){_motor->pos.int_error = _motor->input_vars.min_request_Idq.q;}
./MESC_Common/Src/MESCposition.c:30://	if(_motor->pos.error>0.0f){
./MESC_Common/Src/MESCposition.c:31://		if(_motor->pos.int_error<0.0f){
./MESC_Common/Src/MESCposition.c:32://			_motor->pos.int_error=0.0f;
./MESC_Common/Src/MESCposition.c:35://	if(_motor->pos.error<0.0f){
./MESC_Common/Src/MESCposition.c:36://		if(_motor->pos.int_error>0.0f){
./MESC_Common/Src/MESCposition.c:37://			_motor->pos.int_error=0.0f;
./MESC_Common/Src/MESCposition.c:41:	if(fabsf(_motor->pos.error)>_motor->pos.deadzone)
./MESC_Common/Src/MESCposition.c:43:		_motor->FOC.Idq_prereq.q = _motor->pos.p_error + _motor->pos.int_error + _motor->pos.d_error;
./MESC_Common/Src/MESCposition.c:44:	}else{_motor->FOC.Idq_prereq.q = 0.0f;}
./MESC_Common/Src/MESCposition.c:46:	if(_motor->FOC.Idq_prereq.q>_motor->input_vars.max_request_Idq.q){_motor->FOC.Idq_prereq.q = _motor->input_vars.max_request_Idq.q;}
./MESC_Common/Src/MESCposition.c:47:	if(_motor->FOC.Idq_prereq.q<_motor->input_vars.min_request_Idq.q){_motor->FOC.Idq_prereq.q = _motor->input_vars.min_request_Idq.q;}
./MESC_Common/Src/MESClrobs.c:12:	_motor->lrobs.plusminus = 1;
./MESC_Common/Src/MESClrobs.c:21:	 if((fabsf(_motor->FOC.eHz)>0.005f*_motor->FOC.pwm_frequency)&&(_motor->HFI.inject ==0)){
./MESC_Common/Src/MESClrobs.c:23:		  _motor->lrobs.R_observer = (_motor->lrobs.Vd_obs_high_filt-_motor->lrobs.Vd_obs_low_filt)/(2.0f*LR_OBS_CURRENT);
./MESC_Common/Src/MESClrobs.c:24:		  _motor->lrobs.L_observer = (_motor->lrobs.Vq_obs_high_filt-_motor->lrobs.Vq_obs_low_filt-6.28f*(_motor->FOC.eHz-_motor->lrobs.Last_eHz)*_motor->FOC.flux_observed)/(2.0f*LR_OBS_CURRENT*6.28f*_motor->FOC.eHz);
./MESC_Common/Src/MESClrobs.c:26:		  if(_motor->lrobs.plusminus==1){
./MESC_Common/Src/MESClrobs.c:27:				_motor->lrobs.plusminus = -1;
./MESC_Common/Src/MESClrobs.c:28:				_motor->lrobs.Vd_obs_low_filt = _motor->lrobs.Vd_obs_low/_motor->lrobs.LR_collect_count;
./MESC_Common/Src/MESClrobs.c:29:				_motor->lrobs.Vq_obs_low_filt = _motor->lrobs.Vq_obs_low/_motor->lrobs.LR_collect_count;
./MESC_Common/Src/MESClrobs.c:30:				_motor->FOC.Idq_req.d = _motor->FOC.Idq_req.d+1.0f*LR_OBS_CURRENT;
./MESC_Common/Src/MESClrobs.c:31:				_motor->lrobs.Vd_obs_low = 0;
./MESC_Common/Src/MESClrobs.c:32:				_motor->lrobs.Vq_obs_low = 0;
./MESC_Common/Src/MESClrobs.c:33:		  }else if(_motor->lrobs.plusminus == -1){
./MESC_Common/Src/MESClrobs.c:34:				_motor->lrobs.plusminus = 1;
./MESC_Common/Src/MESClrobs.c:35:				_motor->lrobs.Vd_obs_high_filt = _motor->lrobs.Vd_obs_high/_motor->lrobs.LR_collect_count;
./MESC_Common/Src/MESClrobs.c:36:				_motor->lrobs.Vq_obs_high_filt = _motor->lrobs.Vq_obs_high/_motor->lrobs.LR_collect_count;
./MESC_Common/Src/MESClrobs.c:37:				_motor->FOC.Idq_req.d = _motor->FOC.Idq_req.d-1.0f*LR_OBS_CURRENT;
./MESC_Common/Src/MESClrobs.c:38:				_motor->lrobs.Vd_obs_high = 0;
./MESC_Common/Src/MESClrobs.c:39:				_motor->lrobs.Vq_obs_high = 0;
./MESC_Common/Src/MESClrobs.c:41:		_motor->lrobs.Last_eHz = _motor->FOC.eHz;
./MESC_Common/Src/MESClrobs.c:42:		_motor->lrobs.LR_collect_count = 0; //Reset this after doing the calcs
./MESC_Common/Src/MESClrobs.c:45:	  	float Rerror = R_observer-_motor->m.R;
./MESC_Common/Src/MESClrobs.c:46:	  	float Lerror = L_observer-_motor->m.L_D;
./MESC_Common/Src/MESClrobs.c:48:	  	if(fabs(Rerror)<0.1f*_motor->m.R){
./MESC_Common/Src/MESClrobs.c:49:	  		_motor->m.R = _motor->m.R+0.1f*Rerror;
./MESC_Common/Src/MESClrobs.c:50:	  	}else if(fabs(Rerror)<0.5f*_motor->m.R){
./MESC_Common/Src/MESClrobs.c:51:	  		_motor->m.R = _motor->m.R+0.001f*Rerror;
./MESC_Common/Src/MESClrobs.c:53:	  	if(fabs(Lerror)<0.1f*_motor->m.L_D){
./MESC_Common/Src/MESClrobs.c:54:	  		_motor->m.L_D = _motor->m.L_D+0.1f*Lerror;
./MESC_Common/Src/MESClrobs.c:55:	  		_motor->m.L_Q = _motor->m.L_Q +0.1f*Lerror;
./MESC_Common/Src/MESClrobs.c:56:	  	}else if(fabs(Lerror)<0.5f*_motor->m.L_D){
./MESC_Common/Src/MESClrobs.c:57:	  		_motor->m.L_D = _motor->m.L_D+0.001f*Lerror;
./MESC_Common/Src/MESClrobs.c:58:	  		_motor->m.L_Q = _motor->m.L_Q +0.001f*Lerror;
./MESC_Common/Src/MESClrobs.c:65:	_motor->lrobs.LR_collect_count++;
./MESC_Common/Src/MESClrobs.c:66:	if((fabsf(_motor->FOC.eHz)>0.005f*_motor->FOC.pwm_frequency)&&(_motor->HFI.inject ==0)){
./MESC_Common/Src/MESClrobs.c:67:		if(_motor->lrobs.plusminus==1){
./MESC_Common/Src/MESClrobs.c:68:			_motor->lrobs.Vd_obs_low = _motor->lrobs.Vd_obs_low + _motor->FOC.Vdq.d;
./MESC_Common/Src/MESClrobs.c:69:			_motor->lrobs.Vq_obs_low = _motor->lrobs.Vq_obs_low + _motor->FOC.Vdq.q;
./MESC_Common/Src/MESClrobs.c:71:		if(_motor->lrobs.plusminus == -1){
./MESC_Common/Src/MESClrobs.c:72:			_motor->lrobs.Vd_obs_high = _motor->lrobs.Vd_obs_high + _motor->FOC.Vdq.d;
./MESC_Common/Src/MESClrobs.c:73:			_motor->lrobs.Vq_obs_high = _motor->lrobs.Vq_obs_high + _motor->FOC.Vdq.q;
./MESC_Common/Src/MESCfluxobs.c:54:fluxa = fluxa + (_motor->FOC.Vab.a - _motor->FOC.Iab.a * _motor->m.R)*_motor->FOC.pwm_period;
./MESC_Common/Src/MESCfluxobs.c:55:fluxb = fluxb + (_motor->FOC.Vab.b - _motor->FOC.Iab.b * _motor->m.R)*_motor->FOC.pwm_period;
./MESC_Common/Src/MESCfluxobs.c:59:fluxd = _motor->FOC.sincosangle.cos * fluxa +
./MESC_Common/Src/MESCfluxobs.c:60:	    _motor->FOC.sincosangle.sin * fluxb;
./MESC_Common/Src/MESCfluxobs.c:61:fluxq = _motor->FOC.sincosangle.cos * fluxb -
./MESC_Common/Src/MESCfluxobs.c:62:		_motor->FOC.sincosangle.sin * fluxa;
./MESC_Common/Src/MESCfluxobs.c:65:if (fluxa > _motor->FOC.flux_observed) {
./MESC_Common/Src/MESCfluxobs.c:66:	fluxa = _motor->FOC.flux_observed;}
./MESC_Common/Src/MESCfluxobs.c:67:if (fluxa < -_motor->FOC.flux_observed) {
./MESC_Common/Src/MESCfluxobs.c:68:	fluxa = -_motor->FOC.flux_observed;}
./MESC_Common/Src/MESCfluxobs.c:69:if (fluxb > _motor->FOC.flux_observed) {
./MESC_Common/Src/MESCfluxobs.c:70:	fluxb = _motor->FOC.flux_observed;}
./MESC_Common/Src/MESCfluxobs.c:71:if (fluxb < -_motor->FOC.flux_observed) {
./MESC_Common/Src/MESCfluxobs.c:72:	fluxb = -_motor->FOC.flux_observed;}
./MESC_Common/Src/MESCfluxobs.c:91:switch(_motor->options.observer_type){
./MESC_Common/Src/MESCfluxobs.c:101:		flux_linked_norm = _motor->FOC.flux_a*_motor->FOC.flux_a+_motor->FOC.flux_b*_motor->FOC.flux_b;
./MESC_Common/Src/MESCfluxobs.c:102:		flux_err = flux_linked_norm-_motor->FOC.flux_observed*_motor->FOC.flux_observed;
./MESC_Common/Src/MESCfluxobs.c:103:		_motor->FOC.flux_observed = _motor->FOC.flux_observed+ _motor->m.flux_linkage_gain*flux_err;
./MESC_Common/Src/MESCfluxobs.c:104:		if(_motor->FOC.flux_observed>_motor->m.flux_linkage_max){_motor->FOC.flux_observed = _motor->m.flux_linkage_max;}
./MESC_Common/Src/MESCfluxobs.c:105:		if(_motor->FOC.flux_observed<_motor->m.flux_linkage_min){_motor->FOC.flux_observed = _motor->m.flux_linkage_min;}
./MESC_Common/Src/MESCfluxobs.c:113:	  getLabFast(_motor->FOC.FOCAngle, _motor->m.L_D, _motor->m.L_QD, &La, &Lb);
./MESC_Common/Src/MESCfluxobs.c:115:	  _motor->FOC.flux_a = _motor->FOC.flux_a +
./MESC_Common/Src/MESCfluxobs.c:116:			  (_motor->FOC.Vab.a - _motor->m.R * _motor->FOC.Iab.a)*_motor->FOC.pwm_period -
./MESC_Common/Src/MESCfluxobs.c:117:        La * (_motor->FOC.Iab.a - _motor->FOC.Ia_last) - //Salient inductance NOW
./MESC_Common/Src/MESCfluxobs.c:118:		_motor->FOC.Iab.a * (La - La_last); //Differential of phi = Li -> Ldi/dt+idL/dt
./MESC_Common/Src/MESCfluxobs.c:119:	  _motor->FOC.flux_b = _motor->FOC.flux_b +
./MESC_Common/Src/MESCfluxobs.c:120:			  (_motor->FOC.Vab.b - _motor->m.R * _motor->FOC.Iab.b)*_motor->FOC.pwm_period -
./MESC_Common/Src/MESCfluxobs.c:121:        Lb * (_motor->FOC.Iab.b - _motor->FOC.Ib_last) -
./MESC_Common/Src/MESCfluxobs.c:122:		_motor->FOC.Iab.b * (Lb-Lb_last);
./MESC_Common/Src/MESCfluxobs.c:127:	  _motor->FOC.flux_a =
./MESC_Common/Src/MESCfluxobs.c:128:			  _motor->FOC.flux_a + (_motor->FOC.Vab.a - _motor->m.R * _motor->FOC.Iab.a)*_motor->FOC.pwm_period-
./MESC_Common/Src/MESCfluxobs.c:129:        _motor->m.L_D * (_motor->FOC.Iab.a - _motor->FOC.Ia_last);
./MESC_Common/Src/MESCfluxobs.c:130:	  _motor->FOC.flux_b =
./MESC_Common/Src/MESCfluxobs.c:131:			  _motor->FOC.flux_b + (_motor->FOC.Vab.b - _motor->m.R * _motor->FOC.Iab.b)*_motor->FOC.pwm_period -
./MESC_Common/Src/MESCfluxobs.c:132:        _motor->m.L_D * (_motor->FOC.Iab.b - _motor->FOC.Ib_last);
./MESC_Common/Src/MESCfluxobs.c:135:    _motor->FOC.Ia_last = _motor->FOC.Iab.a;
./MESC_Common/Src/MESCfluxobs.c:136:    _motor->FOC.Ib_last = _motor->FOC.Iab.b;
./MESC_Common/Src/MESCfluxobs.c:140:    float err = _motor->FOC.flux_observed*_motor->FOC.flux_observed-_motor->FOC.flux_a*_motor->FOC.flux_a-_motor->FOC.flux_b*_motor->FOC.flux_b;
./MESC_Common/Src/MESCfluxobs.c:141:    _motor->FOC.flux_b = _motor->FOC.flux_b+err*_motor->FOC.flux_b*_motor->m.non_linear_centering_gain;
./MESC_Common/Src/MESCfluxobs.c:142:    _motor->FOC.flux_a = _motor->FOC.flux_a+err*_motor->FOC.flux_a*_motor->m.non_linear_centering_gain;
./MESC_Common/Src/MESCfluxobs.c:145:    if (_motor->FOC.flux_a > _motor->FOC.flux_observed) {
./MESC_Common/Src/MESCfluxobs.c:146:    	_motor->FOC.flux_a = _motor->FOC.flux_observed;}
./MESC_Common/Src/MESCfluxobs.c:147:    if (_motor->FOC.flux_a < -_motor->FOC.flux_observed) {
./MESC_Common/Src/MESCfluxobs.c:148:    	_motor->FOC.flux_a = -_motor->FOC.flux_observed;}
./MESC_Common/Src/MESCfluxobs.c:149:    if (_motor->FOC.flux_b > _motor->FOC.flux_observed) {
./MESC_Common/Src/MESCfluxobs.c:150:    	_motor->FOC.flux_b = _motor->FOC.flux_observed;}
./MESC_Common/Src/MESCfluxobs.c:151:    if (_motor->FOC.flux_b < -_motor->FOC.flux_observed) {
./MESC_Common/Src/MESCfluxobs.c:152:    	_motor->FOC.flux_b = -_motor->FOC.flux_observed;}
./MESC_Common/Src/MESCfluxobs.c:155:    if(_motor->HFI.inject==0){
./MESC_Common/Src/MESCfluxobs.c:156:    	_motor->FOC.FOCAngle = (uint16_t)(32768.0f + 10430.0f * fast_atan2(_motor->FOC.flux_b, _motor->FOC.flux_a)) - 32768;
./MESC_Common/Src/MESCfluxobs.c:167:	float Lnow = _motor->m.L_D + 0.5*(_motor->m.L_Q - _motor->m.L_D) * (_motor->FOC.Idq.q * _motor->FOC.Idq.q) / (_motor->FOC.Idq.q * _motor->FOC.Idq.q + _motor->FOC.Idq.d * _motor->FOC.Idq.d);
./MESC_Common/Src/MESCfluxobs.c:169:	float L_ia = Lnow * _motor->FOC.Iab.a;
./MESC_Common/Src/MESCfluxobs.c:170:	float L_ib = Lnow * _motor->FOC.Iab.b;
./MESC_Common/Src/MESCfluxobs.c:174:			  (_motor->FOC.Vab.a
./MESC_Common/Src/MESCfluxobs.c:175:			  - _motor->m.R * _motor->FOC.Iab.a);
./MESC_Common/Src/MESCfluxobs.c:176://				+ _motor->FOC.ortega_gain * (_motor->FOC.flux_a - L_ia) * err);
./MESC_Common/Src/MESCfluxobs.c:178:			  (_motor->FOC.Vab.b
./MESC_Common/Src/MESCfluxobs.c:179:			  - _motor->m.R * _motor->FOC.Iab.b);
./MESC_Common/Src/MESCfluxobs.c:180://				+ _motor->FOC.ortega_gain * (_motor->FOC.flux_b - L_ib) * err);
./MESC_Common/Src/MESCfluxobs.c:183:		_motor->FOC.flux_a = _motor->FOC.flux_a
./MESC_Common/Src/MESCfluxobs.c:184:				+ flux_a_dot * _motor->FOC.pwm_period;
./MESC_Common/Src/MESCfluxobs.c:185:		_motor->FOC.flux_b = _motor->FOC.flux_b
./MESC_Common/Src/MESCfluxobs.c:186:				+ flux_b_dot * _motor->FOC.pwm_period;
./MESC_Common/Src/MESCfluxobs.c:190:		float err = _motor->m.flux_linkage * _motor->m.flux_linkage
./MESC_Common/Src/MESCfluxobs.c:191:				- (_motor->FOC.flux_a * _motor->FOC.flux_a + _motor->FOC.flux_b*_motor->FOC.flux_b);
./MESC_Common/Src/MESCfluxobs.c:197:		_motor->FOC.flux_a = _motor->FOC.flux_a
./MESC_Common/Src/MESCfluxobs.c:198:				+ _motor->FOC.ortega_gain * (_motor->FOC.flux_a - L_ia) * err * _motor->FOC.pwm_period;
./MESC_Common/Src/MESCfluxobs.c:199:		_motor->FOC.flux_b = _motor->FOC.flux_b
./MESC_Common/Src/MESCfluxobs.c:200:				+ _motor->FOC.ortega_gain * (_motor->FOC.flux_b - L_ib) * err * _motor->FOC.pwm_period;
./MESC_Common/Src/MESCfluxobs.c:202:	    if(_motor->HFI.inject==0){
./MESC_Common/Src/MESCfluxobs.c:203:	    	//_motor->FOC.FOCAngle = (uint16_t)(32768.0f + 10430.0f * fast_atan2((_motor->FOC.flux_b - L_ib), _motor->FOC.flux_a - L_ia)) - 32768;
./MESC_Common/Src/MESCfluxobs.c:204:	    	_motor->FOC.FOCAngle = (_motor->FOC.FOC_advance * _motor->FOC.PLL_int) + (uint16_t)(32768.0f + 10430.0f * fast_atan2((_motor->FOC.flux_b - L_ib), _motor->FOC.flux_a - L_ia)) - 32768;
./MESC_Common/Src/MESCfluxobs.c:216:	_motor->FOC.BEMFd = _motor->FOC.Vdq.d - _motor->FOC.Idq.d * _motor->m.R - _motor->FOC.eHz * 6.28f *_motor->m.L_Q * _motor->FOC.Idq.q;
./MESC_Common/Src/MESCfluxobs.c:217:	_motor->FOC.BEMFq = _motor->FOC.Vdq.q - _motor->FOC.Idq.q * _motor->m.R - _motor->FOC.eHz * 6.28f * (_motor->m.L_D * _motor->FOC.Idq.d);// + _motor->m.flux_linkage);
./MESC_Common/Src/MESCfluxobs.c:219:	_motor->FOC.BEMFdq_angle = fast_atan2(_motor->FOC.BEMFd, _motor->FOC.BEMFq);
./MESC_Common/Src/MESCfluxobs.c:220://_motor->FOC.FOCAngle = _motor->FOC.FOCAngle + 10430.0f * _motor->FOC.BEMFdq_angle;
./MESC_Common/Src/MESCfluxobs.c:227:	_motor->FOC.BEMF_error = _motor->FOC.BEMF_kp * _motor->FOC.BEMFd/(_motor->FOC.eHz * 6.28f*_motor->m.flux_linkage);
./MESC_Common/Src/MESCfluxobs.c:228:	_motor->FOC.BEMF_integral = 0.9995*_motor->FOC.BEMF_integral + _motor->FOC.BEMF_ki * _motor->FOC.BEMF_error;
./MESC_Common/Src/MESCfluxobs.c:229:    if(_motor->HFI.inject==0){
./MESC_Common/Src/MESCfluxobs.c:230:    	_motor->FOC.FOCAngle = _motor->FOC.FOCAngle + (int)(10430.69f * (_motor->FOC.BEMF_error + _motor->FOC.BEMF_integral));
./MESC_Common/Src/MESCfluxobs.c:241:    _motor->FOC.enc_obs_angle = _motor->FOC.FOCAngle - _motor->FOC.enc_angle;
./MESC_Common/Src/MESCerror.c:116:	if(_motor->MotorState == MOTOR_STATE_INITIALISING){
./MESC_Common/Src/MESCerror.c:119:	_motor->MotorState = MOTOR_STATE_ERROR;
./MESC_Common/Src/MESCerror.c:123:	error_log.current_A = _motor->Conv.Iu;
./MESC_Common/Src/MESCerror.c:124:	error_log.current_B = _motor->Conv.Iv;
./MESC_Common/Src/MESCerror.c:125:	error_log.current_C = _motor->Conv.Iw;
./MESC_Common/Src/MESCerror.c:126:	error_log.voltage = _motor->Conv.Vbus;
./MESC_Common/Src/MESCerror.c:127:	error_log.motor_flux = _motor->m.flux_linkage;
./MESC_Common/Src/MESCerror.c:128:	error_log.flux_a = _motor->FOC.flux_a;
./MESC_Common/Src/MESCerror.c:129:	error_log.flux_b = _motor->FOC.flux_b;
./MESC_Common/Src/MESCerror.c:148:		_motor->MotorState = MOTOR_STATE_TRACKING;
./MESC_Common/Src/MESCmeasure.c:39:	 switch(_motor->meas.state) {
./MESC_Common/Src/MESCmeasure.c:41:	 		_motor->meas.state = MEAS_STATE_INIT;
./MESC_Common/Src/MESCmeasure.c:42:	 		_motor->FOC.PLL_int = 0.0f;
./MESC_Common/Src/MESCmeasure.c:43:	 		_motor->FOC.PLL_angle = 0;
./MESC_Common/Src/MESCmeasure.c:46:			_motor->meas.previous_HFI_type = _motor->HFI.Type;
./MESC_Common/Src/MESCmeasure.c:47:			uint16_t half_ARR = _motor->mtimer->Instance->ARR / 2;
./MESC_Common/Src/MESCmeasure.c:48:			_motor->mtimer->Instance->CCR1 = half_ARR;
./MESC_Common/Src/MESCmeasure.c:49:			_motor->mtimer->Instance->CCR2 = half_ARR;
./MESC_Common/Src/MESCmeasure.c:50:			_motor->mtimer->Instance->CCR3 = half_ARR;
./MESC_Common/Src/MESCmeasure.c:51:			_motor->m.R = 0.001f;     // Initialise with a very low value 1mR
./MESC_Common/Src/MESCmeasure.c:52:			_motor->m.L_D = 0.000001f;  // Initialise with a very low value 1uH
./MESC_Common/Src/MESCmeasure.c:53:			_motor->m.L_Q = 0.000001f;
./MESC_Common/Src/MESCmeasure.c:59:			_motor->FOC.Idq_req.d = _motor->meas.measure_current;
./MESC_Common/Src/MESCmeasure.c:60:			_motor->FOC.Idq_req.q = 0.0f;
./MESC_Common/Src/MESCmeasure.c:61:			_motor->FOC.FOCAngle = 0;
./MESC_Common/Src/MESCmeasure.c:63:			_motor->HFI.inject = 0;  // flag to not inject at SVPWM top
./MESC_Common/Src/MESCmeasure.c:67:			_motor->meas.top_V = 0;
./MESC_Common/Src/MESCmeasure.c:68:			_motor->meas.bottom_V = 0;
./MESC_Common/Src/MESCmeasure.c:69:			_motor->meas.top_I = 0;
./MESC_Common/Src/MESCmeasure.c:70:			_motor->meas.bottom_I = 0;
./MESC_Common/Src/MESCmeasure.c:71:			_motor->meas.top_I_L = 0;
./MESC_Common/Src/MESCmeasure.c:72:			_motor->meas.bottom_I_L = 0;
./MESC_Common/Src/MESCmeasure.c:73:			_motor->meas.top_I_Lq = 0;
./MESC_Common/Src/MESCmeasure.c:74:			_motor->meas.bottom_I_Lq = 0;
./MESC_Common/Src/MESCmeasure.c:76:			_motor->meas.count_top = 0.0f;
./MESC_Common/Src/MESCmeasure.c:77:			_motor->meas.count_bottom = 0.0f;
./MESC_Common/Src/MESCmeasure.c:79:			_motor->meas.PWM_cycles = 0;
./MESC_Common/Src/MESCmeasure.c:81:			_motor->meas.state = MEAS_STATE_ALIGN; //Next Step
./MESC_Common/Src/MESCmeasure.c:84:	 	      _motor->FOC.Idq_req.d = _motor->meas.measure_current;
./MESC_Common/Src/MESCmeasure.c:85:	 	      _motor->FOC.Idq_req.q = 0.0f;
./MESC_Common/Src/MESCmeasure.c:87:	 	      _motor->HFI.inject = 0;
./MESC_Common/Src/MESCmeasure.c:89:	 	      if(_motor->meas.PWM_cycles > _motor->FOC.pwm_frequency){ // 1second
./MESC_Common/Src/MESCmeasure.c:90:	 	    	 _motor->meas.state = MEAS_STATE_LOWER_SETPOINT;
./MESC_Common/Src/MESCmeasure.c:91:	 	    	 _motor->meas.PWM_cycles = 0;
./MESC_Common/Src/MESCmeasure.c:95:			_motor->FOC.Idq_req.d = 0.20f*_motor->meas.measure_current;
./MESC_Common/Src/MESCmeasure.c:96:			_motor->HFI.inject = 0;
./MESC_Common/Src/MESCmeasure.c:99:			_motor->meas.bottom_V = _motor->meas.bottom_V + _motor->FOC.Vdq.d;
./MESC_Common/Src/MESCmeasure.c:100:			_motor->meas.bottom_I = _motor->meas.bottom_I + _motor->FOC.Idq.d;
./MESC_Common/Src/MESCmeasure.c:101:			_motor->meas.count_bottom++;
./MESC_Common/Src/MESCmeasure.c:102:			_motor->meas.Vd_temp = _motor->FOC.Vdq.d * 1.0f;  // Store the voltage required for the low setpoint, to
./MESC_Common/Src/MESCmeasure.c:104:			if(_motor->meas.PWM_cycles > 5000){
./MESC_Common/Src/MESCmeasure.c:105:				_motor->meas.state = MEAS_STATE_UPPER_SETPOINT;
./MESC_Common/Src/MESCmeasure.c:106:				_motor->meas.PWM_cycles = 0;
./MESC_Common/Src/MESCmeasure.c:110:			_motor->FOC.Idq_req.d = _motor->meas.measure_current;
./MESC_Common/Src/MESCmeasure.c:111:			_motor->HFI.inject = 0;
./MESC_Common/Src/MESCmeasure.c:113:	 		if(_motor->meas.PWM_cycles > 5000){
./MESC_Common/Src/MESCmeasure.c:114:	 			_motor->meas.state = MEAS_STATE_UPPER_SETPOINT;
./MESC_Common/Src/MESCmeasure.c:115:				_motor->meas.PWM_cycles = 0;
./MESC_Common/Src/MESCmeasure.c:119:			_motor->FOC.Idq_req.d = _motor->meas.measure_current;
./MESC_Common/Src/MESCmeasure.c:120:			_motor->HFI.inject = 0;
./MESC_Common/Src/MESCmeasure.c:123:			_motor->meas.top_V = _motor->meas.top_V + _motor->FOC.Vdq.d;
./MESC_Common/Src/MESCmeasure.c:124:			_motor->meas.top_I = _motor->meas.top_I + _motor->FOC.Idq.d;
./MESC_Common/Src/MESCmeasure.c:125:			_motor->meas.count_top++;
./MESC_Common/Src/MESCmeasure.c:126:			_motor->meas.Vd_temp = _motor->FOC.Vdq.d * 0.75f;  // Store the voltage required for the low setpoint, to
./MESC_Common/Src/MESCmeasure.c:128:			if(_motor->meas.PWM_cycles > 5000){
./MESC_Common/Src/MESCmeasure.c:131:				_motor->m.R = (_motor->meas.top_V - _motor->meas.bottom_V) / (_motor->meas.top_I - _motor->meas.bottom_I);
./MESC_Common/Src/MESCmeasure.c:133:				_motor->meas.state = MEAS_STATE_INIT_LD;
./MESC_Common/Src/MESCmeasure.c:134:				_motor->meas.PWM_cycles = 0;
./MESC_Common/Src/MESCmeasure.c:139:			//Vd_temp = _motor->FOC.Vdq.d * 1.0f;  // Store the voltage required for the high setpoint, to
./MESC_Common/Src/MESCmeasure.c:141:			_motor->meas.Vq_temp = 0.0f;
./MESC_Common/Src/MESCmeasure.c:142:			_motor->FOC.Vdq.q = 0.0f;//
./MESC_Common/Src/MESCmeasure.c:143:			_motor->FOC.Idq_int_err.d = 0.0f;
./MESC_Common/Src/MESCmeasure.c:144:			_motor->FOC.Idq_int_err.q = 0.0f;
./MESC_Common/Src/MESCmeasure.c:145:			_motor->meas.count_top = 0.0f;
./MESC_Common/Src/MESCmeasure.c:146:			_motor->meas.count_bottom = 0.0f;
./MESC_Common/Src/MESCmeasure.c:147:			_motor->meas.top_I_L = 0.0f;
./MESC_Common/Src/MESCmeasure.c:148:			_motor->meas.bottom_I_L = 0.0f;
./MESC_Common/Src/MESCmeasure.c:151:			_motor->meas.state = MEAS_STATE_COLLECT_LD;
./MESC_Common/Src/MESCmeasure.c:152:			_motor->meas.PWM_cycles = 0;
./MESC_Common/Src/MESCmeasure.c:155:			_motor->HFI.Type = HFI_TYPE_SPECIAL;
./MESC_Common/Src/MESCmeasure.c:156:			_motor->HFI.inject = 1;  // flag to the SVPWM writer to inject at top
./MESC_Common/Src/MESCmeasure.c:157:			_motor->HFI.special_injectionVd = _motor->meas.measure_voltage;
./MESC_Common/Src/MESCmeasure.c:158:			_motor->HFI.special_injectionVq = 0.0f;
./MESC_Common/Src/MESCmeasure.c:160:			_motor->FOC.Vdq.d = _motor->meas.Vd_temp;
./MESC_Common/Src/MESCmeasure.c:161:			_motor->FOC.Vdq.q = 0.0f;
./MESC_Common/Src/MESCmeasure.c:164:			if (_motor->HFI.inject_high_low_now == 1) {
./MESC_Common/Src/MESCmeasure.c:165:			  _motor->meas.top_I_L = _motor->meas.top_I_L + _motor->FOC.Idq.d;
./MESC_Common/Src/MESCmeasure.c:166:			  _motor->meas.count_top++;
./MESC_Common/Src/MESCmeasure.c:167:			} else if (_motor->HFI.inject_high_low_now == 0) {
./MESC_Common/Src/MESCmeasure.c:168:			  _motor->meas.bottom_I_L = _motor->meas.bottom_I_L + _motor->FOC.Idq.d;
./MESC_Common/Src/MESCmeasure.c:169:			  _motor->meas.count_bottom++;
./MESC_Common/Src/MESCmeasure.c:171:			if(_motor->meas.PWM_cycles > _motor->FOC.pwm_frequency){ // 1second
./MESC_Common/Src/MESCmeasure.c:172:				_motor->meas.state = MEAS_STATE_INIT_LQ;
./MESC_Common/Src/MESCmeasure.c:173:				_motor->meas.PWM_cycles = 0;
./MESC_Common/Src/MESCmeasure.c:178:			_motor->m.L_D =
./MESC_Common/Src/MESCmeasure.c:179:			  fabsf((_motor->HFI.special_injectionVd) /
./MESC_Common/Src/MESCmeasure.c:180:			  ((_motor->meas.top_I_L - _motor->meas.bottom_I_L) / (_motor->meas.count_top * _motor->FOC.pwm_period)));
./MESC_Common/Src/MESCmeasure.c:181:			_motor->meas.top_I_Lq = 0.0f;
./MESC_Common/Src/MESCmeasure.c:182:			_motor->meas.bottom_I_Lq = 0.0f;
./MESC_Common/Src/MESCmeasure.c:183:			_motor->meas.count_topq = 0.0f;
./MESC_Common/Src/MESCmeasure.c:184:			_motor->meas.count_bottomq = 0.0f;
./MESC_Common/Src/MESCmeasure.c:185:			if(_motor->meas.PWM_cycles > 2){ //Wait a bit
./MESC_Common/Src/MESCmeasure.c:189:				_motor->meas.state = MEAS_STATE_COLLECT_LQ;
./MESC_Common/Src/MESCmeasure.c:190:				_motor->meas.PWM_cycles = 0;
./MESC_Common/Src/MESCmeasure.c:194:			_motor->HFI.special_injectionVd = 0.0f;
./MESC_Common/Src/MESCmeasure.c:195:			_motor->HFI.special_injectionVq = _motor->meas.measure_voltage;
./MESC_Common/Src/MESCmeasure.c:196:			_motor->HFI.inject = 1;  // flag to the SVPWM writer to update at top
./MESC_Common/Src/MESCmeasure.c:197:			_motor->FOC.Vdq.d = _motor->meas.Vd_temp;  // Vd_temp to keep it aligned with D axis
./MESC_Common/Src/MESCmeasure.c:198:			_motor->FOC.Vdq.q = 0.0f;
./MESC_Common/Src/MESCmeasure.c:201:			if (_motor->HFI.inject_high_low_now == 1) {
./MESC_Common/Src/MESCmeasure.c:202:			  _motor->meas.top_I_Lq = _motor->meas.top_I_Lq + _motor->FOC.Idq.q;
./MESC_Common/Src/MESCmeasure.c:203:			  _motor->meas.count_topq++;
./MESC_Common/Src/MESCmeasure.c:204:			} else if (_motor->HFI.inject_high_low_now == 0) {
./MESC_Common/Src/MESCmeasure.c:205:			  _motor->meas.bottom_I_Lq = _motor->meas.bottom_I_Lq + _motor->FOC.Idq.q;
./MESC_Common/Src/MESCmeasure.c:206:			_motor->meas.count_bottomq++;
./MESC_Common/Src/MESCmeasure.c:209:			if(_motor->meas.PWM_cycles > _motor->FOC.pwm_frequency){ // 1second
./MESC_Common/Src/MESCmeasure.c:211:				_motor->m.L_Q =
./MESC_Common/Src/MESCmeasure.c:212:				  fabsf((_motor->HFI.special_injectionVq) /
./MESC_Common/Src/MESCmeasure.c:213:				  ((_motor->meas.top_I_Lq - _motor->meas.bottom_I_Lq) / (_motor->meas.count_top * _motor->FOC.pwm_period)));
./MESC_Common/Src/MESCmeasure.c:216:			      _motor->HFI.Type = _motor->meas.previous_HFI_type;
./MESC_Common/Src/MESCmeasure.c:217:			      _motor->MotorState = MOTOR_STATE_IDLE;
./MESC_Common/Src/MESCmeasure.c:219:			      _motor->HFI.inject = 0;  // flag to the SVPWM writer stop injecting at top
./MESC_Common/Src/MESCmeasure.c:220:			      _motor->HFI.special_injectionVd = 0.0f;
./MESC_Common/Src/MESCmeasure.c:221:			      _motor->HFI.special_injectionVq = 0.0f;
./MESC_Common/Src/MESCmeasure.c:222:			      _motor->HFI.Vd_injectionV = 0.0f;
./MESC_Common/Src/MESCmeasure.c:223:			      _motor->HFI.Vq_injectionV = 0.0f;
./MESC_Common/Src/MESCmeasure.c:225:			      _motor->MotorState = MOTOR_STATE_TRACKING;
./MESC_Common/Src/MESCmeasure.c:226:			      _motor->meas.PWM_cycles = 0;
./MESC_Common/Src/MESCmeasure.c:231:			      _motor->meas.state = MEAS_STATE_IDLE;
./MESC_Common/Src/MESCmeasure.c:236:	 		_motor->meas.state = MEAS_STATE_IDLE;
./MESC_Common/Src/MESCmeasure.c:241:	 _motor->meas.PWM_cycles++;
./MESC_Common/Src/MESCmeasure.c:246:	_motor->meas.previous_HFI_type = _motor->HFI.Type;
./MESC_Common/Src/MESCmeasure.c:247:	_motor->HFI.Type=HFI_TYPE_NONE;
./MESC_Common/Src/MESCmeasure.c:248: 	_motor->HFI.inject = 0;
./MESC_Common/Src/MESCmeasure.c:253:   	_motor->m.flux_linkage_max = 0.1f;
./MESC_Common/Src/MESCmeasure.c:254:   	_motor->m.flux_linkage_min = 0.00001f;//Set really wide limits
./MESC_Common/Src/MESCmeasure.c:255:   	_motor->FOC.openloop_step = 0;
./MESC_Common/Src/MESCmeasure.c:256:   	_motor->FOC.flux_observed = _motor->m.flux_linkage_min;
./MESC_Common/Src/MESCmeasure.c:257:   	old_HFI_type = _motor->HFI.Type;
./MESC_Common/Src/MESCmeasure.c:258:   	_motor->HFI.Type = HFI_TYPE_NONE;
./MESC_Common/Src/MESCmeasure.c:269:       _motor->FOC.Idq_req.d = _motor->meas.measure_current*0.5f;  //
./MESC_Common/Src/MESCmeasure.c:270:       _motor->FOC.Idq_req.q = 0.0f;
./MESC_Common/Src/MESCmeasure.c:271:   	_motor->meas.angle_delta = temp_angle-_motor->FOC.FOCAngle;
./MESC_Common/Src/MESCmeasure.c:272:   	_motor->FOC.openloop_step = (uint16_t)(ERPM_MEASURE*65536.0f/(_motor->FOC.pwm_frequency*60.0f)*(float)cycles/65000.0f);
./MESC_Common/Src/MESCmeasure.c:273:   	_motor->FOC.FOCAngle = temp_angle;
./MESC_Common/Src/MESCmeasure.c:275:       temp_angle = _motor->FOC.FOCAngle;
./MESC_Common/Src/MESCmeasure.c:277:       	_motor->meas.temp_flux = sqrtf(_motor->FOC.Vdq.d*_motor->FOC.Vdq.d+_motor->FOC.Vdq.q*_motor->FOC.Vdq.q)/(6.28f * (float)_motor->FOC.openloop_step * (float)_motor->FOC.pwm_frequency/65536.0f);
./MESC_Common/Src/MESCmeasure.c:278:       	_motor->FOC.flux_observed  = _motor->meas.temp_flux;
./MESC_Common/Src/MESCmeasure.c:279:       	_motor->FOC.flux_a = _motor->FOC.sincosangle.cos*_motor->FOC.flux_observed;
./MESC_Common/Src/MESCmeasure.c:280:       	_motor->FOC.flux_b = _motor->FOC.sincosangle.sin*_motor->FOC.flux_observed;
./MESC_Common/Src/MESCmeasure.c:281:       	_motor->m.flux_linkage_max = 1.7f*_motor->FOC.flux_observed;
./MESC_Common/Src/MESCmeasure.c:282:       	_motor->m.flux_linkage_min = 0.5f*_motor->FOC.flux_observed;
./MESC_Common/Src/MESCmeasure.c:283:       	_motor->meas.temp_FLA = _motor->FOC.flux_a;
./MESC_Common/Src/MESCmeasure.c:284:       	_motor->meas.temp_FLB = _motor->FOC.flux_b;
./MESC_Common/Src/MESCmeasure.c:289:     _motor->FOC.Idq_req.d = 0.0f;
./MESC_Common/Src/MESCmeasure.c:290:     _motor->FOC.Idq_req.q = _motor->meas.measure_closedloop_current;
./MESC_Common/Src/MESCmeasure.c:294:      _motor->m.flux_linkage = _motor->FOC.flux_observed;
./MESC_Common/Src/MESCmeasure.c:296:     _motor->MotorState = MOTOR_STATE_TRACKING;
./MESC_Common/Src/MESCmeasure.c:297:     _motor->HFI.Type = old_HFI_type;
./MESC_Common/Src/MESCmeasure.c:299:     _motor->HFI.Type = _motor->meas.previous_HFI_type;
./MESC_Common/Src/MESCmeasure.c:300:     if (_motor->m.flux_linkage > 0.0001f && _motor->m.flux_linkage < 200.0f) {
./MESC_Common/Src/MESCmeasure.c:301:   	_motor->MotorSensorMode = MOTOR_SENSOR_MODE_SENSORLESS;
./MESC_Common/Src/MESCmeasure.c:303:       _motor->MotorState = MOTOR_STATE_ERROR;
./MESC_Common/Src/MESCmeasure.c:319: 	_motor->meas.previous_HFI_type = _motor->HFI.Type;
./MESC_Common/Src/MESCmeasure.c:320: 	_motor->HFI.Type = HFI_TYPE_D;
./MESC_Common/Src/MESCmeasure.c:321: 	_motor->input_vars.UART_req = 0.25f;
./MESC_Common/Src/MESCmeasure.c:327: 		_motor->HFI.Type = HFI_TYPE_D;
./MESC_Common/Src/MESCmeasure.c:328: 		dinductance = dinductance + _motor->FOC.didq.d;
./MESC_Common/Src/MESCmeasure.c:335: 	_motor->FOC.d_polarity = -1;
./MESC_Common/Src/MESCmeasure.c:339: 		_motor->HFI.Type = HFI_TYPE_D;
./MESC_Common/Src/MESCmeasure.c:340: 		qinductance = qinductance + _motor->FOC.didq.d;
./MESC_Common/Src/MESCmeasure.c:345: 	_motor->HFI.mod_didq = sqrtf(qinductance*qinductance+dinductance*dinductance);
./MESC_Common/Src/MESCmeasure.c:346: 	_motor->HFI.Gain = 5000.0f/_motor->HFI.mod_didq; //Magic numbers that seem to work
./MESC_Common/Src/MESCmeasure.c:347: 	_motor->input_vars.UART_req = 0.0f;
./MESC_Common/Src/MESCmeasure.c:348: 	_motor->FOC.d_polarity = 1;
./MESC_Common/Src/MESCmeasure.c:350: 	_motor->HFI.Type = _motor->meas.previous_HFI_type;
./MESC_Common/Src/MESCmeasure.c:352: 	return _motor->HFI.mod_didq;
./MESC_Common/Src/MESCmeasure.c:367:			_motor->mtimer->Instance->CCR1 = test_on_time;
./MESC_Common/Src/MESCmeasure.c:368:			_motor->mtimer->Instance->CCR2 = 0;
./MESC_Common/Src/MESCmeasure.c:369:			_motor->mtimer->Instance->CCR3 = 0;
./MESC_Common/Src/MESCmeasure.c:370:			if(_motor->Conv.Iu<1.0f){ test_on_time=test_on_time+1;}
./MESC_Common/Src/MESCmeasure.c:371:			if(_motor->Conv.Iu>1.0f){ test_on_time=test_on_time-1;}
./MESC_Common/Src/MESCmeasure.c:376:			_motor->mtimer->Instance->CCR1 = 0;
./MESC_Common/Src/MESCmeasure.c:377:			_motor->mtimer->Instance->CCR2 = test_on_time;
./MESC_Common/Src/MESCmeasure.c:378:			_motor->mtimer->Instance->CCR3 = 0;
./MESC_Common/Src/MESCmeasure.c:379:			if(_motor->Conv.Iv<1.0f){ test_on_time=test_on_time+1;}
./MESC_Common/Src/MESCmeasure.c:380:			if(_motor->Conv.Iv>1.0f){ test_on_time=test_on_time-1;}
./MESC_Common/Src/MESCmeasure.c:385:			_motor->mtimer->Instance->CCR1 = 0;
./MESC_Common/Src/MESCmeasure.c:386:			_motor->mtimer->Instance->CCR2 = 0;
./MESC_Common/Src/MESCmeasure.c:387:			_motor->mtimer->Instance->CCR3 = test_on_time;
./MESC_Common/Src/MESCmeasure.c:388:			if(_motor->Conv.Iw<1.0f){ test_on_time=test_on_time+1;}
./MESC_Common/Src/MESCmeasure.c:389:			if(_motor->Conv.Iw>1.0f){ test_on_time=test_on_time-1;}
./MESC_Common/Src/MESCmeasure.c:395:			_motor->MotorState = MOTOR_STATE_TRACKING;
./MESC_Common/Src/MESCmeasure.c:400:			_motor->FOC.deadtime_comp = test_on_time_acc[0];
./MESC_Common/Src/MESCmeasure.c:419:  hallstate = _motor->hall.current_hall_state;
./MESC_Common/Src/MESCmeasure.c:431:    _motor->FOC.Idq_req.d = 10.0f;
./MESC_Common/Src/MESCmeasure.c:432:    _motor->FOC.Idq_req.q = 0.0f;
./MESC_Common/Src/MESCmeasure.c:434:    _motor->FOC.FOCAngle = 0.0f;
./MESC_Common/Src/MESCmeasure.c:437:    _motor->FOC.Idq_req.d = 10.0f;
./MESC_Common/Src/MESCmeasure.c:438:    _motor->FOC.Idq_req.q = 0.0f;
./MESC_Common/Src/MESCmeasure.c:441:      if (_motor->FOC.FOCAngle < (anglestep)) {
./MESC_Common/Src/MESCmeasure.c:444:      if ((_motor->FOC.FOCAngle < (30000)) &&
./MESC_Common/Src/MESCmeasure.c:445:          (_motor->FOC.FOCAngle > (29000 - anglestep))) {
./MESC_Common/Src/MESCmeasure.c:455:      _motor->FOC.FOCAngle =
./MESC_Common/Src/MESCmeasure.c:456:          _motor->FOC.FOCAngle + anglestep;  // Increment the angle
./MESC_Common/Src/MESCmeasure.c:459:          _motor->FOC.FOCAngle;       // Accumulate the angles through the sweep
./MESC_Common/Src/MESCmeasure.c:468:      if ((_motor->FOC.FOCAngle < (12000)) && (hallstate != _motor->hall.last_hall_state)) {
./MESC_Common/Src/MESCmeasure.c:471:      if ((_motor->FOC.FOCAngle < (65535)) &&
./MESC_Common/Src/MESCmeasure.c:472:          (_motor->FOC.FOCAngle > (65535 - anglestep))) {
./MESC_Common/Src/MESCmeasure.c:482:      _motor->FOC.FOCAngle =
./MESC_Common/Src/MESCmeasure.c:483:          _motor->FOC.FOCAngle - anglestep;  // Increment the angle
./MESC_Common/Src/MESCmeasure.c:486:          _motor->FOC.FOCAngle;       // Accumulate the angles through the sweep
./MESC_Common/Src/MESCmeasure.c:501:          _motor->m.hall_table[i][2] = hallangles[i + 1][0];//This is the center angle of the hall state
./MESC_Common/Src/MESCmeasure.c:502:          _motor->m.hall_table[i][3] = hallangles[i + 1][1];//This is the width of the hall state
./MESC_Common/Src/MESCmeasure.c:503:          _motor->m.hall_table[i][0] = _motor->m.hall_table[i][2]-_motor->m.hall_table[i][3]/2;//This is the start angle of the hall state
./MESC_Common/Src/MESCmeasure.c:504:          _motor->m.hall_table[i][1] = _motor->m.hall_table[i][2]+_motor->m.hall_table[i][3]/2;//This is the end angle of the hall state
./MESC_Common/Src/MESCmeasure.c:506:    _motor->MotorState = MOTOR_STATE_TRACKING;
./MESC_Common/Src/MESCmeasure.c:507:    _motor->FOC.Idq_req.d = 0;
./MESC_Common/Src/MESCmeasure.c:508:    _motor->FOC.Idq_req.q = 0;
./MESC_Common/Src/MESCmeasure.c:519:		__HAL_TIM_DISABLE_IT(_motor->mtimer,TIM_IT_UPDATE); //DISABLE INTERRUPT, DANGEROUS
./MESC_Common/Src/MESCmeasure.c:523:		_motor->mtimer->Instance->CCR1 = 0;
./MESC_Common/Src/MESCmeasure.c:524:		_motor->mtimer->Instance->CCR2 = 0;
./MESC_Common/Src/MESCmeasure.c:525:		_motor->mtimer->Instance->CCR3 = 0;
./MESC_Common/Src/MESCmeasure.c:526:		_motor->test_vals.dp_current_final[dp_counter] = _motor->Conv.Iv;
./MESC_Common/Src/MESCmeasure.c:529:    	_motor->mtimer->Instance->CCR1 = 0;
./MESC_Common/Src/MESCmeasure.c:530:    	_motor->mtimer->Instance->CCR2 = 0;
./MESC_Common/Src/MESCmeasure.c:531:    	_motor->mtimer->Instance->CCR3 = _motor->mtimer->Instance->ARR;
./MESC_Common/Src/MESCmeasure.c:535:    	_motor->test_vals.dp_current_final[dp_counter] = _motor->Conv.Iv;
./MESC_Common/Src/MESCmeasure.c:538:	  _motor->mtimer->Instance->CCR2 = 0;
./MESC_Common/Src/MESCmeasure.c:539:	  _motor->mtimer->Instance->CCR3 = 0;
./MESC_Common/Src/MESCmeasure.c:540:	  _motor->test_vals.dp_current_final[dp_counter] = _motor->Conv.Iv;
./MESC_Common/Src/MESCmeasure.c:543:	   _motor->mtimer->Instance->CCR2 = 0;
./MESC_Common/Src/MESCmeasure.c:544:	   _motor->mtimer->Instance->CCR3 = 200;
./MESC_Common/Src/MESCmeasure.c:545:	   _motor->test_vals.dp_current_final[dp_counter] = _motor->Conv.Iv;
./MESC_Common/Src/MESCmeasure.c:548:	   _motor->mtimer->Instance->CCR2 = 0;
./MESC_Common/Src/MESCmeasure.c:549:	   _motor->mtimer->Instance->CCR3 = 0;
./MESC_Common/Src/MESCmeasure.c:550:	   _motor->test_vals.dp_current_final[dp_counter] = _motor->Conv.Iv;
./MESC_Common/Src/MESCmeasure.c:553:    	_motor->mtimer->Instance->CCR1 = 0;
./MESC_Common/Src/MESCmeasure.c:554:    	_motor->mtimer->Instance->CCR2 = 0;
./MESC_Common/Src/MESCmeasure.c:555:    	_motor->mtimer->Instance->CCR3 = 0;
./MESC_Common/Src/MESCmeasure.c:556:    	_motor->test_vals.dp_current_final[dp_counter] = _motor->Conv.Iv;
./MESC_Common/Src/MESCmeasure.c:559:    	__HAL_TIM_ENABLE_IT(_motor->mtimer, TIM_IT_UPDATE);///RE-ENABLE INTERRUPT
./MESC_Common/Src/MESCmeasure.c:560:    	_motor->MotorState = MOTOR_STATE_TRACKING;
./MESC_Common/Src/MESCinput.c:40:	_motor->input_vars.max_request_Idq.d = 0.0f; //Not supporting d-axis input current for now
./MESC_Common/Src/MESCinput.c:41:	_motor->input_vars.min_request_Idq.d = 0.0f;
./MESC_Common/Src/MESCinput.c:42:	if(!_motor->input_vars.max_request_Idq.q){
./MESC_Common/Src/MESCinput.c:43:		_motor->input_vars.max_request_Idq.q = MAX_IQ_REQUEST;
./MESC_Common/Src/MESCinput.c:45:		// _motor->input_vars.min_request_Idq.q = MIN_IQ_REQUEST; //ToDo, SETTING THESE ASSYMETRIC WILL CAUSE ISSUES WITH REVERSE..
./MESC_Common/Src/MESCinput.c:46:		_motor->input_vars.min_request_Idq.q = _motor->input_vars.max_request_Idq.q * -1.0f;
./MESC_Common/Src/MESCinput.c:49:	_motor->input_vars.IC_pulse_MAX = IC_PULSE_MAX;
./MESC_Common/Src/MESCinput.c:50:	_motor->input_vars.IC_pulse_MIN = IC_PULSE_MIN;
./MESC_Common/Src/MESCinput.c:51:	_motor->input_vars.IC_pulse_MID = IC_PULSE_MID;
./MESC_Common/Src/MESCinput.c:52:	_motor->input_vars.IC_pulse_DEADZONE = IC_PULSE_DEADZONE;
./MESC_Common/Src/MESCinput.c:53:	_motor->input_vars.IC_duration_MAX = IC_DURATION_MAX;
./MESC_Common/Src/MESCinput.c:54:	_motor->input_vars.IC_duration_MIN = IC_DURATION_MIN;
./MESC_Common/Src/MESCinput.c:56:	if(!_motor->input_vars.adc1_MAX){
./MESC_Common/Src/MESCinput.c:57:		_motor->input_vars.adc1_MAX = ADC1MAX;
./MESC_Common/Src/MESCinput.c:58:		_motor->input_vars.adc1_MIN = ADC1MIN;
./MESC_Common/Src/MESCinput.c:59:		_motor->input_vars.adc1_OOR = ADC1OOR;
./MESC_Common/Src/MESCinput.c:60:		_motor->input_vars.ADC1_polarity = ADC1_POLARITY;
./MESC_Common/Src/MESCinput.c:62:	if(!_motor->input_vars.adc2_MAX){
./MESC_Common/Src/MESCinput.c:63:		_motor->input_vars.adc2_MAX = ADC2MAX;
./MESC_Common/Src/MESCinput.c:64:		_motor->input_vars.adc2_MIN = ADC2MIN;
./MESC_Common/Src/MESCinput.c:65:		_motor->input_vars.adc2_OOR = ADC2OOR;
./MESC_Common/Src/MESCinput.c:66:		_motor->input_vars.ADC2_polarity = ADC2_POLARITY;
./MESC_Common/Src/MESCinput.c:69:	_motor->input_vars.adc1_gain[0] = 1.0f/(_motor->input_vars.adc1_MAX-_motor->input_vars.adc1_MIN);
./MESC_Common/Src/MESCinput.c:70:	_motor->input_vars.adc1_gain[1] = 1.0f/(_motor->input_vars.adc1_MAX-_motor->input_vars.adc1_MIN);
./MESC_Common/Src/MESCinput.c:72:	_motor->input_vars.adc2_gain[0] = 1.0f/(_motor->input_vars.adc2_MAX-_motor->input_vars.adc2_MIN);
./MESC_Common/Src/MESCinput.c:73:	_motor->input_vars.adc2_gain[1] = 1.0f/(_motor->input_vars.adc2_MAX-_motor->input_vars.adc2_MIN);
./MESC_Common/Src/MESCinput.c:76:	_motor->input_vars.RCPWM_gain[0][0] = 1.0f/((float)_motor->input_vars.IC_pulse_MAX - (float)_motor->input_vars.IC_pulse_MID - (float)_motor->input_vars.IC_pulse_DEADZONE);
./MESC_Common/Src/MESCinput.c:77:	_motor->input_vars.RCPWM_gain[0][1] = 1.0f/(((float)_motor->input_vars.IC_pulse_MID - (float)_motor->input_vars.IC_pulse_DEADZONE)-(float)_motor->input_vars.IC_pulse_MIN);
./MESC_Common/Src/MESCinput.c:79:	if(!_motor->input_vars.input_options){
./MESC_Common/Src/MESCinput.c:80:		_motor->input_vars.input_options = DEFAULT_INPUT;
./MESC_Common/Src/MESCinput.c:83:	_motor->input_vars.UART_req = 0.0f;
./MESC_Common/Src/MESCinput.c:84:	_motor->input_vars.RCPWM_req = 0.0f;
./MESC_Common/Src/MESCinput.c:85:	_motor->input_vars.ADC1_req = 0.0f;
./MESC_Common/Src/MESCinput.c:86:	_motor->input_vars.ADC2_req = 0.0f;
./MESC_Common/Src/MESCinput.c:95:	  if(_motor->input_vars.remote_ADC_timeout > 0){
./MESC_Common/Src/MESCinput.c:96:		  _motor->input_vars.remote_ADC_timeout--;
./MESC_Common/Src/MESCinput.c:98:		  _motor->input_vars.remote_ADC1_req = 0.0f;
./MESC_Common/Src/MESCinput.c:99:		  _motor->input_vars.remote_ADC2_req = 0.0f;
./MESC_Common/Src/MESCinput.c:105:	  if((_motor->input_vars.input_options & 0b100000)&&(_motor->input_vars.remote_ADC_can_id > 0)){
./MESC_Common/Src/MESCinput.c:108:		  _motor->input_vars.remote_ADC1_req = 0.0f;//Set the input variable to zero
./MESC_Common/Src/MESCinput.c:112:	  if((_motor->input_vars.input_options & 0b1000000)&&(_motor->input_vars.remote_ADC_can_id > 0)){
./MESC_Common/Src/MESCinput.c:115:		  _motor->input_vars.remote_ADC2_req = 0.0f;//Set the input variable to zero
./MESC_Common/Src/MESCinput.c:119:	  if((_motor->input_vars.input_options & 0b10000000)){
./MESC_Common/Src/MESCinput.c:123:		  _motor->input_vars.ADC12_diff_req = 0.0f; //Set the input variable to zero
./MESC_Common/Src/MESCinput.c:132:	  //if(0 == (_motor->input_vars.input_options & 0b1000)){
./MESC_Common/Src/MESCinput.c:133:		 // _motor->input_vars.UART_req = 0.0f;
./MESC_Common/Src/MESCinput.c:137:	  if ((_motor->input_vars.input_options & 0b1000) == 0 &&
./MESC_Common/Src/MESCinput.c:138:	      _motor->input_vars.input_options != 1 &&
./MESC_Common/Src/MESCinput.c:139:	      _motor->input_vars.input_options != 32) {
./MESC_Common/Src/MESCinput.c:141:	      _motor->input_vars.UART_req = 0.0f;
./MESC_Common/Src/MESCinput.c:145:	  if(_motor->input_vars.input_options & 0b0100){
./MESC_Common/Src/MESCinput.c:146:		  if(_motor->input_vars.pulse_recieved){
./MESC_Common/Src/MESCinput.c:147:			  if((_motor->input_vars.IC_duration > _motor->input_vars.IC_duration_MIN) && (_motor->input_vars.IC_duration < _motor->input_vars.IC_duration_MAX)){
./MESC_Common/Src/MESCinput.c:148:				  if(_motor->input_vars.IC_pulse>(_motor->input_vars.IC_pulse_MID + _motor->input_vars.IC_pulse_DEADZONE)){
./MESC_Common/Src/MESCinput.c:149:					  _motor->input_vars.RCPWM_req = (float)(_motor->input_vars.IC_pulse - (_motor->input_vars.IC_pulse_MID + _motor->input_vars.IC_pulse_DEADZONE))*_motor->input_vars.RCPWM_gain[0][1];
./MESC_Common/Src/MESCinput.c:150:					  if(fabsf(_motor->input_vars.RCPWM_req>1.1f)){
./MESC_Common/Src/MESCinput.c:153:					  if(_motor->input_vars.RCPWM_req>1.0f){_motor->input_vars.RCPWM_req=1.0f;}
./MESC_Common/Src/MESCinput.c:154:					  if(_motor->input_vars.RCPWM_req<-1.0f){_motor->input_vars.RCPWM_req=-1.0f;}
./MESC_Common/Src/MESCinput.c:156:				  else if(_motor->input_vars.IC_pulse<(_motor->input_vars.IC_pulse_MID - _motor->input_vars.IC_pulse_DEADZONE)){
./MESC_Common/Src/MESCinput.c:157:					  _motor->input_vars.RCPWM_req = ((float)_motor->input_vars.IC_pulse - (float)(_motor->input_vars.IC_pulse_MID - _motor->input_vars.IC_pulse_DEADZONE))*_motor->input_vars.RCPWM_gain[0][1];
./MESC_Common/Src/MESCinput.c:158:					  if(fabsf(_motor->input_vars.RCPWM_req>1.1f)){
./MESC_Common/Src/MESCinput.c:161:					  if(_motor->input_vars.RCPWM_req>1.0f){_motor->input_vars.RCPWM_req=1.0f;}
./MESC_Common/Src/MESCinput.c:162:					  if(_motor->input_vars.RCPWM_req<-1.0f){_motor->input_vars.RCPWM_req=-1.0f;}
./MESC_Common/Src/MESCinput.c:164:					  _motor->input_vars.RCPWM_req = 0.0f;
./MESC_Common/Src/MESCinput.c:168:				  _motor->input_vars.RCPWM_req = 0.0f;
./MESC_Common/Src/MESCinput.c:171:			  _motor->input_vars.RCPWM_req = 0.0f;
./MESC_Common/Src/MESCinput.c:174:		  _motor->input_vars.RCPWM_req = 0.0f;
./MESC_Common/Src/MESCinput.c:178:	  if(_motor->input_vars.input_options & 0b0010){
./MESC_Common/Src/MESCinput.c:179:			  if(_motor->Raw.ADC_in_ext2>_motor->input_vars.adc2_MIN){
./MESC_Common/Src/MESCinput.c:180:				  _motor->input_vars.ADC2_req = ((float)_motor->Raw.ADC_in_ext2-(float)_motor->input_vars.adc2_MIN)*_motor->input_vars.adc2_gain[1]*_motor->input_vars.ADC2_polarity;
./MESC_Common/Src/MESCinput.c:181:				  if(_motor->Raw.ADC_in_ext2>_motor->input_vars.adc2_OOR){
./MESC_Common/Src/MESCinput.c:187:				  _motor->input_vars.ADC2_req = 0.0f;
./MESC_Common/Src/MESCinput.c:189:			  if(_motor->input_vars.ADC2_req>1.0f){_motor->input_vars.ADC2_req=1.0f;}
./MESC_Common/Src/MESCinput.c:190:			  if(_motor->input_vars.ADC2_req<-1.0f){_motor->input_vars.ADC2_req=-1.0f;}
./MESC_Common/Src/MESCinput.c:192:		  _motor->input_vars.ADC2_req = 0.0f;
./MESC_Common/Src/MESCinput.c:196:	  if(_motor->input_vars.input_options & 0b0001){
./MESC_Common/Src/MESCinput.c:197:			  if(_motor->Raw.ADC_in_ext1>_motor->input_vars.adc1_MIN){
./MESC_Common/Src/MESCinput.c:198:				  _motor->input_vars.ADC1_req = ((float)_motor->Raw.ADC_in_ext1-(float)_motor->input_vars.adc1_MIN)*_motor->input_vars.adc1_gain[1]*_motor->input_vars.ADC1_polarity;
./MESC_Common/Src/MESCinput.c:199:				  if(_motor->Raw.ADC_in_ext1>_motor->input_vars.adc1_OOR){
./MESC_Common/Src/MESCinput.c:205:				  _motor->input_vars.ADC1_req = 0.0f;
./MESC_Common/Src/MESCinput.c:207:		  if(_motor->input_vars.ADC1_req>1.0f){_motor->input_vars.ADC1_req=1.0f;}
./MESC_Common/Src/MESCinput.c:208:		  if(_motor->input_vars.ADC1_req<-1.0f){_motor->input_vars.ADC1_req=-1.0f;}
./MESC_Common/Src/MESCinput.c:210:		  _motor->input_vars.ADC1_req = 0.0f;
./MESC_Common/Src/MESCinput.c:214:	  if(_motor->input_vars.input_options & 0b10000){//Killswitch
./MESC_Common/Src/MESCinput.c:216:			_motor->input_vars.nKillswitch = 1;
./MESC_Common/Src/MESCinput.c:217:			_motor->key_bits &= ~KILLSWITCH_KEY;
./MESC_Common/Src/MESCinput.c:219:			_motor->input_vars.nKillswitch = 0;
./MESC_Common/Src/MESCinput.c:220:			_motor->key_bits |= KILLSWITCH_KEY;
./MESC_Common/Src/MESCinput.c:222:		if(_motor->input_vars.invert_killswitch){
./MESC_Common/Src/MESCinput.c:223:			_motor->input_vars.nKillswitch = !_motor->input_vars.nKillswitch;
./MESC_Common/Src/MESCinput.c:224:			_motor->key_bits ^= KILLSWITCH_KEY;
./MESC_Common/Src/MESCinput.c:227:		  _motor->input_vars.nKillswitch = 1;
./MESC_Common/Src/MESCinput.c:228:			_motor->key_bits &= ~KILLSWITCH_KEY;
./MESC_Common/Src/MESCinput.c:231:	  _motor->input_vars.nKillswitch = 1;
./MESC_Common/Src/MESCinput.c:232:	  _motor->key_bits &= ~KILLSWITCH_KEY;
./MESC_RTOS/MESC/MESCinterface.c:531://	_motor->FOC.FOC_advance
./MESC_F401CC/Core/Src/MESChw_setup.c:70:  _motor->Raw.Iu = hadc1.Instance->JDR1;// PhaseU Current
./MESC_F401CC/Core/Src/MESChw_setup.c:71:  _motor->Raw.Iv = hadc1.Instance->JDR2;
./MESC_F401CC/Core/Src/MESChw_setup.c:72:  _motor->Raw.Iw = hadc1.Instance->JDR3;
./MESC_F401CC/Core/Src/MESChw_setup.c:73:  _motor->Raw.Vbus = hadc1.Instance->JDR4; //Bus/battery voltage
./MESC_F401CC/Core/Src/MESChw_setup.c:79:  _motor->Raw.MOSu_T = ADC_buffer[5]; //Temperature on PB1
./MESC_F401CC/Core/Src/MESChw_setup.c:81:  _motor->Raw.ADC_in_ext2 = ADC_buffer[4];
./MESC_F401CC/Core/Src/MESChw_setup.c:86:	  _motor->Raw.Vu = ADC_buffer[0]; //PhaseU Voltage
./MESC_F401CC/Core/Src/MESChw_setup.c:87:	  _motor->Raw.Vv = ADC_buffer[1];
./MESC_F401CC/Core/Src/MESChw_setup.c:88:	  _motor->Raw.Vw = ADC_buffer[2];
./MESC_F401CC/Core/Src/MESChw_setup.c:199:	HAL_TIM_PWM_Start(_motor->mtimer, TIM_CHANNEL_4);
./MESC_F401CC/Core/Src/MESChw_setup.c:201:	HAL_TIM_PWM_Start(_motor->mtimer, TIM_CHANNEL_1);
./MESC_F401CC/Core/Src/MESChw_setup.c:202:	HAL_TIMEx_PWMN_Start(_motor->mtimer, TIM_CHANNEL_1);
./MESC_F401CC/Core/Src/MESChw_setup.c:204:	HAL_TIM_PWM_Start(_motor->mtimer, TIM_CHANNEL_2);
./MESC_F401CC/Core/Src/MESChw_setup.c:205:	HAL_TIMEx_PWMN_Start(_motor->mtimer, TIM_CHANNEL_2);
./MESC_F401CC/Core/Src/MESChw_setup.c:207:	HAL_TIM_PWM_Start(_motor->mtimer, TIM_CHANNEL_3);
./MESC_F401CC/Core/Src/MESChw_setup.c:208:	HAL_TIMEx_PWMN_Start(_motor->mtimer, TIM_CHANNEL_3);
./MESC_F401CC/Core/Src/MESChw_setup.c:214:	__HAL_TIM_ENABLE_IT(_motor->mtimer, TIM_IT_UPDATE);
./MESC_F401CC/Core/Src/MESChw_setup.c:217:	HAL_TIM_IC_Start(_motor->stimer, TIM_CHANNEL_1);
./MESC_F401CC/Core/Src/MESChw_setup.c:218:	HAL_TIM_IC_Start(_motor->stimer, TIM_CHANNEL_2);
./MESC_F401CC/Core/Src/MESChw_setup.c:219:	__HAL_TIM_ENABLE_IT(_motor->stimer, TIM_IT_UPDATE);
./MESC_F401CC/Core/Src/MESChw_setup.c:221:	__HAL_TIM_SET_PRESCALER(_motor->stimer, (HAL_RCC_GetHCLKFreq() / 1000000 - 1));
./MESC_F411CE/Core/Src/MESChw_setup.c:67:  _motor->Raw.Iu = hadc1.Instance->JDR1;// PhaseU Current
./MESC_F411CE/Core/Src/MESChw_setup.c:68:  _motor->Raw.Iv = hadc1.Instance->JDR2;
./MESC_F411CE/Core/Src/MESChw_setup.c:69:  _motor->Raw.Iw = hadc1.Instance->JDR3;
./MESC_F411CE/Core/Src/MESChw_setup.c:70:  _motor->Raw.Vbus = hadc1.Instance->JDR4; //Bus/battery voltage
./MESC_F411CE/Core/Src/MESChw_setup.c:76:  _motor->Raw.MOSu_T = ADC_buffer[5]; //Temperature on PB1
./MESC_F411CE/Core/Src/MESChw_setup.c:78:  _motor->Raw.ADC_in_ext2 = ADC_buffer[4];
./MESC_F411CE/Core/Src/MESChw_setup.c:83:	  _motor->Raw.Vu = ADC_buffer[0]; //PhaseU Voltage
./MESC_F411CE/Core/Src/MESChw_setup.c:84:	  _motor->Raw.Vv = ADC_buffer[1];
./MESC_F411CE/Core/Src/MESChw_setup.c:85:	  _motor->Raw.Vw = ADC_buffer[2];
./MESC_F411CE/Core/Src/MESChw_setup.c:106:	HAL_TIM_PWM_Start(_motor->mtimer, TIM_CHANNEL_4);
./MESC_F411CE/Core/Src/MESChw_setup.c:108:	HAL_TIM_PWM_Start(_motor->mtimer, TIM_CHANNEL_1);
./MESC_F411CE/Core/Src/MESChw_setup.c:109:	HAL_TIMEx_PWMN_Start(_motor->mtimer, TIM_CHANNEL_1);
./MESC_F411CE/Core/Src/MESChw_setup.c:111:	HAL_TIM_PWM_Start(_motor->mtimer, TIM_CHANNEL_2);
./MESC_F411CE/Core/Src/MESChw_setup.c:112:	HAL_TIMEx_PWMN_Start(_motor->mtimer, TIM_CHANNEL_2);
./MESC_F411CE/Core/Src/MESChw_setup.c:114:	HAL_TIM_PWM_Start(_motor->mtimer, TIM_CHANNEL_3);
./MESC_F411CE/Core/Src/MESChw_setup.c:115:	HAL_TIMEx_PWMN_Start(_motor->mtimer, TIM_CHANNEL_3);
./MESC_F411CE/Core/Src/MESChw_setup.c:121:	__HAL_TIM_ENABLE_IT(_motor->mtimer, TIM_IT_UPDATE);
./MESC_F411CE/Core/Src/MESChw_setup.c:124:	HAL_TIM_IC_Start(_motor->stimer, TIM_CHANNEL_1);
./MESC_F411CE/Core/Src/MESChw_setup.c:125:	HAL_TIM_IC_Start(_motor->stimer, TIM_CHANNEL_2);
./MESC_F411CE/Core/Src/MESChw_setup.c:126:	__HAL_TIM_ENABLE_IT(_motor->stimer, TIM_IT_UPDATE);
./MESC_F411CE/Core/Src/MESChw_setup.c:128:	__HAL_TIM_SET_PRESCALER(_motor->stimer, (HAL_RCC_GetHCLKFreq() / 1000000 - 1));
./MESC_F405RG_Stepper/Core/Src/MESChw_setup.c:74://  _motor->Raw.Iu = hadc1.Instance->JDR1;  // U Current
./MESC_F405RG_Stepper/Core/Src/MESChw_setup.c:75://  _motor->Raw.Iv = hadc2.Instance->JDR1;  // V Current
./MESC_F405RG_Stepper/Core/Src/MESChw_setup.c:76://  _motor->Raw.Iw = hadc3.Instance->JDR1;  // W Current
./MESC_F405RG_Stepper/Core/Src/MESChw_setup.c:77://  _motor->Raw.Vbus = hadc3.Instance->JDR3;  // DC Link Voltage
./MESC_F405RG_Stepper/Core/Src/MESChw_setup.c:79:  _motor->Raw.Iu = ADC2_buffer[1];  // U Current
./MESC_F405RG_Stepper/Core/Src/MESChw_setup.c:80:  _motor->Raw.Iv = ADC2_buffer[0];	// V Current
./MESC_F405RG_Stepper/Core/Src/MESChw_setup.c:81:  _motor->Raw.Iw = 2048;			//Dummy W current, only 2 phase
./MESC_F405RG_Stepper/Core/Src/MESChw_setup.c:82:  _motor->Raw.Vbus = ADC2_buffer[2];
./MESC_F405RG_Stepper/Core/Src/MESChw_setup.c:84:  _motor->Raw.Iu = hadc1.Instance->JDR1;  // U Current
./MESC_F405RG_Stepper/Core/Src/MESChw_setup.c:85:  _motor->Raw.Iv = hadc1.Instance->JDR2;	// V Current
./MESC_F405RG_Stepper/Core/Src/MESChw_setup.c:86:  _motor->Raw.Iw = 2048;			//Dummy W current, only 2 phase
./MESC_F405RG_Stepper/Core/Src/MESChw_setup.c:87:  _motor->Raw.Vbus = hadc1.Instance->JDR3;
./MESC_F405RG_Stepper/Core/Src/MESChw_setup.c:230:    HAL_TIM_PWM_Start(_motor->mtimer, TIM_CHANNEL_4 );
./MESC_F405RG_Stepper/Core/Src/MESChw_setup.c:233:    HAL_TIM_PWM_Start(    _motor->mtimer, TIM_CHANNEL_1 );
./MESC_F405RG_Stepper/Core/Src/MESChw_setup.c:234:    HAL_TIMEx_PWMN_Start( _motor->mtimer, TIM_CHANNEL_1 );
./MESC_F405RG_Stepper/Core/Src/MESChw_setup.c:236:    HAL_TIM_PWM_Start(    _motor->mtimer, TIM_CHANNEL_2 );
./MESC_F405RG_Stepper/Core/Src/MESChw_setup.c:237:    HAL_TIMEx_PWMN_Start( _motor->mtimer, TIM_CHANNEL_2 );
./MESC_F405RG_Stepper/Core/Src/MESChw_setup.c:239:    HAL_TIM_PWM_Start(    _motor->mtimer, TIM_CHANNEL_3 );
./MESC_F405RG_Stepper/Core/Src/MESChw_setup.c:240:    HAL_TIMEx_PWMN_Start( _motor->mtimer, TIM_CHANNEL_3 );
./MESC_F405RG_Stepper/Core/Src/MESChw_setup.c:245:    __HAL_TIM_ENABLE_IT(_motor->mtimer, TIM_IT_UPDATE);
