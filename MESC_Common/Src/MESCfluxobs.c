/*
 **
 ******************************************************************************
 * @file           : MESCfluxobs.c
 * @brief          : Flux observer
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

#include "MESCfluxobs.h"
#include "MESCsin_lut.h"

  /////////////////////////////////////////////////////////////////////////////
  // SENSORLESS IMPLEMENTATION//////////////////////////////////////////////////
  float fluxa, fluxb, fluxd, fluxq, fbd, fbq;
void MESCfluxobs_v2_run(MESC_motor_typedef *_motor){
/*	Inspired by the dq reference frame use of Alex Evers, author of the UniMoC
	This observer will attempt to track flux in the dq frame with the following actions:
	1) Carry out the fluxa = integral(Va-Rxia) and fluxb = integral(Vb-Rxib)
	2) Magically remove the inductive fluxes from the integral (subtract inverse park transform of LqIq and LdId?) and Apply atan2 to calculate the angle

	3) Convert the fluxes  to dq frame via park
	4) Modify with the subtraction of Lqxiq and Ldxid respectively
	5) Apply the feedback based on the deviation from reference fluxes
		5)a) From here, could get feedback for an observer in dq frame?
	6) Modify with the addition of Lqxiq and Ldxid respectively (inverse of 4)
	7) Convert fluxes from dq frame to ab frame overwriting fluxa and fluxb

*/
fluxa = fluxa + (_motor->FOC.Vab.a - _motor->FOC.Iab.a * _motor->m.R)*_motor->FOC.pwm_period;
fluxb = fluxb + (_motor->FOC.Vab.b - _motor->FOC.Iab.b * _motor->m.R)*_motor->FOC.pwm_period;


// Park transform
fluxd = _motor->FOC.sincosangle.cos * fluxa +
	    _motor->FOC.sincosangle.sin * fluxb;
fluxq = _motor->FOC.sincosangle.cos * fluxb -
		_motor->FOC.sincosangle.sin * fluxa;

//This is not part of the final version
if (fluxa > _motor->FOC.flux_observed) {
	fluxa = _motor->FOC.flux_observed;}
if (fluxa < -_motor->FOC.flux_observed) {
	fluxa = -_motor->FOC.flux_observed;}
if (fluxb > _motor->FOC.flux_observed) {
	fluxb = _motor->FOC.flux_observed;}
if (fluxb < -_motor->FOC.flux_observed) {
	fluxb = -_motor->FOC.flux_observed;}
}

void MESCfluxobs_run(MESC_motor_typedef *_motor) {
	//flux_observer_V2(_motor); //For testing comparison, running both at the same time
	// LICENCE NOTE REMINDER:
    // This work deviates slightly from the BSD 3 clause licence.
    // The work here is entirely original to the MESC FOC project, and not based
    // on any appnotes, or borrowed from another project. This work is free to
    // use, as granted in BSD 3 clause, with the exception that this note must
    // be included in where this code is implemented/modified to use your
    // variable names, structures containing variables or other minor
    // rearrangements in place of the original names I have chosen, and credit
    // to David Molony as the original author must be noted.

    // With thanks to C0d3b453 for generally keeping this compiling and Elwin
    // for producing data comparing the output to a 16bit encoder.
	float flux_linked_norm;
	float flux_err;
switch(_motor->options.observer_type){
case NONE:
	break;
case MXLEMMING_LAMBDA:
	  //Variant of the flux linkage observer created by/with Benjamin Vedder to
	  //eliminate the need to accurately know the flux linked motor parameter.
	  //This may be useful when approaching saturation; currently unclear but
	  //definitely makes setup less critical.
	  //It basically takes the normal of the flux linkage at any time and
	  //changes the flux limits accordingly, ignoring using a sqrt for computational efficiency
		flux_linked_norm = _motor->FOC.flux_a*_motor->FOC.flux_a+_motor->FOC.flux_b*_motor->FOC.flux_b;
		flux_err = flux_linked_norm-_motor->FOC.flux_observed*_motor->FOC.flux_observed;
		_motor->FOC.flux_observed = _motor->FOC.flux_observed+ _motor->m.flux_linkage_gain*flux_err;
		if(_motor->FOC.flux_observed>_motor->m.flux_linkage_max){_motor->FOC.flux_observed = _motor->m.flux_linkage_max;}
		if(_motor->FOC.flux_observed<_motor->m.flux_linkage_min){_motor->FOC.flux_observed = _motor->m.flux_linkage_min;}
	//Fallthrough
case MXLEMMING:
	// This is the actual observer function.
	// We are going to integrate Va-Ri and clamp it positively and negatively
	// the angle is then the arctangent of the integrals shifted 180 degrees
#ifdef USE_SALIENT_OBSERVER
	  float La, Lb;
	  getLabFast(_motor->FOC.FOCAngle, _motor->m.L_D, _motor->m.L_QD, &La, &Lb);

	  _motor->FOC.flux_a = _motor->FOC.flux_a +
			  (_motor->FOC.Vab.a - _motor->m.R * _motor->FOC.Iab.a)*_motor->FOC.pwm_period -
        La * (_motor->FOC.Iab.a - _motor->FOC.Ia_last) - //Salient inductance NOW
		_motor->FOC.Iab.a * (La - La_last); //Differential of phi = Li -> Ldi/dt+idL/dt
	  _motor->FOC.flux_b = _motor->FOC.flux_b +
			  (_motor->FOC.Vab.b - _motor->m.R * _motor->FOC.Iab.b)*_motor->FOC.pwm_period -
        Lb * (_motor->FOC.Iab.b - _motor->FOC.Ib_last) -
		_motor->FOC.Iab.b * (Lb-Lb_last);
//Store the inductances
    La_last = La;
    Lb_last = Lb;
#else
	  _motor->FOC.flux_a =
			  _motor->FOC.flux_a + (_motor->FOC.Vab.a - _motor->m.R * _motor->FOC.Iab.a)*_motor->FOC.pwm_period-
        _motor->m.L_D * (_motor->FOC.Iab.a - _motor->FOC.Ia_last);
	  _motor->FOC.flux_b =
			  _motor->FOC.flux_b + (_motor->FOC.Vab.b - _motor->m.R * _motor->FOC.Iab.b)*_motor->FOC.pwm_period -
        _motor->m.L_D * (_motor->FOC.Iab.b - _motor->FOC.Ib_last);
#endif
//Store the currents
    _motor->FOC.Ia_last = _motor->FOC.Iab.a;
    _motor->FOC.Ib_last = _motor->FOC.Iab.b;

#ifdef USE_NONLINEAR_OBSERVER_CENTERING
///Try directly applying the centering using the same method as the flux linkage observer
    float err = _motor->FOC.flux_observed*_motor->FOC.flux_observed-_motor->FOC.flux_a*_motor->FOC.flux_a-_motor->FOC.flux_b*_motor->FOC.flux_b;
    _motor->FOC.flux_b = _motor->FOC.flux_b+err*_motor->FOC.flux_b*_motor->m.non_linear_centering_gain;
    _motor->FOC.flux_a = _motor->FOC.flux_a+err*_motor->FOC.flux_a*_motor->m.non_linear_centering_gain;
#endif
#ifdef USE_CLAMPED_OBSERVER_CENTERING
    if (_motor->FOC.flux_a > _motor->FOC.flux_observed) {
    	_motor->FOC.flux_a = _motor->FOC.flux_observed;}
    if (_motor->FOC.flux_a < -_motor->FOC.flux_observed) {
    	_motor->FOC.flux_a = -_motor->FOC.flux_observed;}
    if (_motor->FOC.flux_b > _motor->FOC.flux_observed) {
    	_motor->FOC.flux_b = _motor->FOC.flux_observed;}
    if (_motor->FOC.flux_b < -_motor->FOC.flux_observed) {
    	_motor->FOC.flux_b = -_motor->FOC.flux_observed;}
#endif
    //Calculate the angle
    if(_motor->HFI.inject==0){
    	_motor->FOC.FOCAngle = (uint16_t)(32768.0f + 10430.0f * fast_atan2(_motor->FOC.flux_b, _motor->FOC.flux_a)) - 32768;
    }

	break;

case ORTEGA_ORIGINAL:
//Seems that in the presence of hig salience, the Ortega observer stabilises better
//This is becuase the application of the flux error adaptation is rotated by the L_ia and L_ib components
//This results in a decreasing power factor with higher gain but remains stable.

//Adapt inductance
	float Lnow = _motor->m.L_D + 0.5*(_motor->m.L_Q - _motor->m.L_D) * (_motor->FOC.Idq.q * _motor->FOC.Idq.q) / (_motor->FOC.Idq.q * _motor->FOC.Idq.q + _motor->FOC.Idq.d * _motor->FOC.Idq.d);

	float L_ia = Lnow * _motor->FOC.Iab.a;
	float L_ib = Lnow * _motor->FOC.Iab.b;

//Flux rate of changes
		float flux_a_dot =
			  (_motor->FOC.Vab.a
			  - _motor->m.R * _motor->FOC.Iab.a);
//				+ _motor->FOC.ortega_gain * (_motor->FOC.flux_a - L_ia) * err);
		float flux_b_dot =
			  (_motor->FOC.Vab.b
			  - _motor->m.R * _motor->FOC.Iab.b);
//				+ _motor->FOC.ortega_gain * (_motor->FOC.flux_b - L_ib) * err);

//Integrate
		_motor->FOC.flux_a = _motor->FOC.flux_a
				+ flux_a_dot * _motor->FOC.pwm_period;
		_motor->FOC.flux_b = _motor->FOC.flux_b
				+ flux_b_dot * _motor->FOC.pwm_period;

//Ortega Error
//Contrary to the usual implementation, do the integration before calculating the error for more up to date data.
		float err = _motor->m.flux_linkage * _motor->m.flux_linkage
				- (_motor->FOC.flux_a * _motor->FOC.flux_a + _motor->FOC.flux_b*_motor->FOC.flux_b);
		if (err > 0.0f) {
			err = 0.0f;
				}

//Apply error correction
		_motor->FOC.flux_a = _motor->FOC.flux_a
				+ _motor->FOC.ortega_gain * (_motor->FOC.flux_a - L_ia) * err * _motor->FOC.pwm_period;
		_motor->FOC.flux_b = _motor->FOC.flux_b
				+ _motor->FOC.ortega_gain * (_motor->FOC.flux_b - L_ib) * err * _motor->FOC.pwm_period;
//Calculate the angle
	    if(_motor->HFI.inject==0){
	    	_motor->FOC.FOCAngle = (uint16_t)(32768.0f + 10430.0f * fast_atan2((_motor->FOC.flux_b - L_ib), _motor->FOC.flux_a - L_ia)) - 32768;
	    }

	break;

}//End of observer type switch



#ifdef TRACK_ENCODER_OBSERVER_ERROR
    //This does not apply the encoder angle,
    //It tracks the difference between the encoder and the observer.
    _motor->FOC.enc_obs_angle = _motor->FOC.FOCAngle - _motor->FOC.enc_angle;
#endif
  }
