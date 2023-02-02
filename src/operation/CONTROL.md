# Control Loops


## Fast Control loop
All the FOC gets done in the fastLoop. This is the only critical part of the MESC, the rest can actually be removed providing the parameters get set and MESC is initialised correctly .
If you remove the slowloop, you can write directly to mtr[n]->FOC.Idq_req.q, set the mtr[n]->MotorState to MOTOR_STATE_RUN and enable the inverter.

### The geometric transforms
The forward transforms take place in the ADCConversion(_motor) function. This converts the read currents firstly from 3 phase to two phase (Clarke) and then rotates them to the estimated/read from encoder rotor reference frame (Park).

Geometric transforms are the Clark and Park, which take the form:

Clarke:
\\[\begin{bmatrix}I\alpha \cr I\beta \cr I\gamma \end{bmatrix} = 2/3 \begin{bmatrix} 1 & -0.5 & -0.5\cr0 & \sqrt{3}/2 & -\sqrt{3}/2 \cr 0.5 & 0.5 & 0.5 \end{bmatrix} \begin{bmatrix}Iu \cr Iv \cr Iw \end{bmatrix}  \\]
where MESC selects for the lower of the PWM values at high modulation using the substitution \\( Iu + Iv + Iw = 0\\) .

Park (rotation matrix around \\( \gamma\\)):
\\[ \begin{bmatrix}Id \cr Iq \cr I0 \end{bmatrix} = \begin{bmatrix}cos\theta & sin\theta & 0 \cr -sin\theta & cos\theta & 0 \cr 0 & 0 & 1\end{bmatrix} \begin{bmatrix}I\alpha \cr I\beta \cr I\gamma \end{bmatrix}\\]
Where \\( \theta\\) is the electrical angle of the rotor; the mechanical angle divided by pole pairs and where the bottom and right rows of the matrix are ignored (we assume \\( \gamma\\) is zero).

\\( sin\theta\\) and \\( cos\theta\\) are calculated from a lookup table 320 elements (=256x1.25) long optionally with interpolation. With interpolation, the maximum error from this is very small; less than the ADC or PWM resolution.

### The Sensorless Observer
The sensorless observer is very simple. The implementation is unique to MESC and was developed without recourse to appnotes or papers. It is probably not unique in industry, but so far I have not seen it in any other open source or commercial source project.
It works (as most successful observers do) on the basis of flux integration, that is the assumption that for a spinning magnet passing a coil, the voltage is given by:
\\[V = nd \phi /over dt\\] 
and we observe from watching the motor on a scope that the voltages are sinusoidal.

Therefore:
\\[int V dt = n \phi +C \\] 
We do not need to care for n, and C varies only dependent on where we start the integration for a sin wave.
The key recognition is that \\( \phi \\) is a constant dependent on the magnets, and therefore the max and min of the resulting integral are symetric and constant.
Since the voltage is sinusoidal, the flux integral will thus also be sinusoidal, with a phase shift of 90 degrees.
(remember to insert pics of sin and integral...)
Further, the addition of noise on the incoming voltage signal is effectively filtered out by this integral since \\( \int cosn\theta dt = cosn\theta\n (+C) \\) and so noise and higher harmonics are greatly reduced.

Within MESC, we choose to carry out this integral in alpha beta frame, so we first remove the effects of resistance and inductance, and then integrate the resulting voltage as:
\\[ V\alpha = VBEMF\alpha + Ri\alpha + Ldi\alpha \over dt\\]
\\[ V\beta = VBEMF\beta + Ri\beta + Ldi\beta \over dt\\]
where \\[V\alpha \\] is the electrical voltage output by the inverter and \\[i\alpha\\] is the clarke transformed current measured by the ADC.
Thusly, we generate two estimated back EMF voltages which we can integrate to get two flux linkages with a 90 degree phase shift.
We have to deal with teh +C term in the integral, and also with integration drift which would result in arctangent not working. MESC simply clamps the flux integral at hard limits which can either be fixed or calculated in realtime by the flux linkage observer. 
Since they are shifted by 90 degrees and already filtered by integration, we need only find the arctangent of the two to calculate an estimated angle.

Alternatively to the arctangent we could construct a true observer:
\\[ \theta-est_(n+1) = \theta-est_n + d\theta + k_p*\theta-calc-\theta-est\\]
where
\\[ d\theta_(n+1) = d\theta_n + k_i*((\theta-est_n + d\theta)-\theta-calc_(n+1))\\]
(here we calculate theta-calc through arctangent as above and forward predict/correct our prediction each cycle)
or: 
\\[ \theta-est_(n+1) = \theta-est_n + d\theta + k_p*\phi_d\\]
where
\\[ d\theta_(n+1) = d\theta_n + k_i*(\phi_d))\\] 
(here we use the d axis flux linkage, which we derive from a rotation of the alpha beta flux linkage as an estimate of the error to be corrected )

MESC chooses not to use a true observer, since there is absolutely no measurable advantage, there are gains to be "tuned" with a true observer and there are additional calculation steps.

MESC contains an observer for salient motors, which accounts for the differing d and q inductances. This is not usually required, is not well tested and will not work for outrunner motors since they saturate so heavily.
It relies on the assumption that the salience travels with the dq frame, and can be transformed into the alpha beta frame, then:
\\[ D[Li] = Ldi\over dt + idL\over dt\\]
And therefore the above estimates for VBEMF can be modified to account for this changing salience in alpha beta frame.

When \\(a \ne 0\\), there are two solutions to \\(ax^2 + bx + c = 0\\) and they are: 

\\[ x = {-b \pm \sqrt{b^2-4ac} \over 2a} \\]

This is the reference github mathjax example...

### The FOC PI

### The Field Weakening

### The Circle Limiter

### The Hall start

### Tracking


## The Hyperloop

### HFI

#### HFI D

#### HFI45

### PLL and Speed estimation

### Interpolation


## Slow control loop

### Input collection

### Duty cycle control

### The Speed Controller

### Temperature limiting

### MTPA

### Field weakening q-current rollback

### Power limiting





## SimpleComs loop

## RTOS and terminal

