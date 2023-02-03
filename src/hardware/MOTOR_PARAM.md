# Calculation of BLDC Motor Parameters

The MP2 as well as other VESC-based systems provide the capability to measure motor parameters using the controller. For example, [vesc-tool](https://vesc-project.com/vesc_tool) collects several motor values from a connected motor and controller by selecting "FOC" on left panel, going to the "General" tab, and following the "Detect and Calculate Parameters" directions on that page. 

So vesc-tool is great for retreiving parameters. However, in cases where you want to measure the parameters yourself or check if your VESC is gathering up the right numbers, this document is for you. 

## PREAMBLE
Some notes before beginning.

**Expertise.** This document assumes you have a reasonable understanding of concepts such as Ohm's law and inductance. It also assume that you're comfortable connecting up high amperage equipment which should be not any more complex than wiring your motor to a controller. You're also going to need to have a lot of familiarity with using VESC-based controllers. 

**Equipment.** In order to do what is described here you'll need the following: 
* A multimeter
* A current limiting power supply
* A LCR meter
* An oscilloscope

You'll also need some way to turn your disconnected motor at a relatively precise rpm. More about that later. 

**BLDC motor configuration.** BLDC motors have two basic achitectures:

<img src="pics/MOTOR_PARAM_wye_v_delta.png" title="Figure 1: motor configurations">

If your BLDC has three big wires coming out of it, we are going to assume that motor is in Wye configuration for the sake of simplifying the math used here.

## RESISTANCE

The copper wires inside of a BLDC are long, and the resistance found in those windings is an important number. In some cases the resistance can be measure direcly with an ohmmeter, but cheap commerical ohmmeters often will not get down to the range needed to get a good value. Instead, perform the following:

* Find the rated amperage of your motor
* Select a value around 10% of this rating
* Set your current limited power supply to your selected amperage

Your motor has three wires -- these are referred to as "phase wires" and are connected to the the termination points of the windings shown in **Figure 2**. Arbitrarily choose two wires, and connect them to your power supply. Note: the motor will [jump](https://www.youtube.com/watch?v=oRXPFaZ0nJI) and then freeze to a holding position when you connect the power supply.

Now, at the same connection point of each phase wire, connect your multimeter to measure volts (you'll probably need something in the millivolt range). Get a number. Your results may look something like this:

<img src="pics/MOTOR_PARAM_resistance.png" title="Figure 1. Capturing resistance based on Ohm's law">

Notice that it would NOT work well to use the measurement that is coming from the power supply itself. Better to use what your multimeter is telling you. The connection used for this measurement is referred to as "phase to phase" because you're actually connecting between two phases of the motor to get your value. Grab your phase to phase voltage, and use Ohm's law to calculate the resistance. In our case:

* R = V / I
* R = 129 mV / 10A
* R = 12.9
* Convert to single phase resistance: R = 12.9 / 2, or 6.45mOhms

## INDUCTANCE VALUES Lq AND Ld
In our case we're going to measure inductance using an LCR meter. LCR meters measures inductance (L), capacitance (C), and resistance (R) and commercial meters are not particularly expensive.

Gathering inductance values requires that we find two important positions inside of your motor that have to do with the position of the stator coils and the rotor magnets. To find these positions you can use the power supply set at the values as before (or less). These positions are described below:

<img src="pics/MOTOR_PARAM_Ld_Lq1.png" title="Figure 3. D-axis and Q-axis motor positions">

Also see [this video](https://youtu.be/oRXPFaZ0nJI) which shows a motor jumping around when the phase wires are connected to a power supply. The marked positions are referred to as d-axis and q-axis (note the above figure). Once you have these positions, do the following:

* Disconnect power supply
* Turn the motor to d-axis position
* Set your LCR to measure inductance
* Using the same phase wires as when connected to the power supply, connect LCR
* Get a reading for Ld
* Turn motor to q-axis position
* Get a reading for Lq

Note, the motor may have residual inductance, or something, that means it takes a while for the L value to settle. In some cases metalic desk tops and placement of hands on the motor have influenced the L value. Take your time, and shorting the leads on the motor also helps to settle residual inductance.

Your measurements for Ld and Lq are phase to phase. Divide by 2 to get single phase Ld and Lq. To get total L for the motor calculate their average, or L = (Ld + Lq) / 2.

In our example here we get the following; (notice the position of the motor in each of the pics):
<img src="pics/MOTOR_PARAM_Ld_Lq2.png" title="Figure 4. Ld/Lq results">

* Phase to phase Ld measure = 174 uH
* Phase to phase Lq measure = 199 uH
* Thus, for single phase:  Ld = 87uH, Lq = 99.5uH, and L = 93uH

## FLUX LINKAGE
You'll discover there is an abundance of ways to measure flux linkage out on the internet. In our case we're going to use an oscilloscope to emperically measure amplitude and period of the volts generated by your BLDC. From this we'll calculate lambda, which is the integral of volt x seconds. 

To get this measurement you're going to need some way to turn your disconnected motor at a relatively precise rpm. You'll probably need to mount the motor, and then connect it to an electric drill or another motor. One method of turning a motor is [shown here](https://youtube.com/shorts/YvglKtxUaCQ). Turn the motor and collect the output of the motor on your scope.

Connect the ground end of your oscilloscope to one phase of the motor, and the probe to another phase. Be sure to note the scope settings for volts / div for the amplitude and ms / div for period. Once you have all this extract values for Vpeak and period of your sine wave. Here's an example:

<img src="pics/MOTOR_PARAM_scope.png" title="Figure 5. Oscilloscope measurements of BLDC motor output">

The scope is set to 2V / div for voltage and and 5ms / div for period. In our pic, Vpeak = 5V, and period, or T = 28ms. The formula to calculate lambda is:

<img src="pics/MOTOR_PARAM_lambda.png" title="equation for flux linkage">

Note: your measurement for the scope is collected from phase to phase; however, this formula does not require that you convert to single phase lambda, the formula handles it for you. 

## COMPARISON TO VESC VALUES

Let's compare these values to what was collected using a 75V 100A single ESC single ubox from Spintend. For the motor used here, this is what you get in vesc-tool by selecting "FOC" on left panel, going to the "General" tab, and following the "Detect and Calculate Parameters" directions on that page:

<img src="pics/MOTOR_PARAM_vesc-tool.png" title="Figure 6. vesc-tool detection results">

Comparing to our emperical values, we get:

<img src="pics/MOTOR_PARAM_table.png" title="comparing results">

These are values are very close. Note that your results are always going to vary based on your controller, wiring, and motor parameters. 

(*) One exception is the resistance. In this example the resistance of the motor is relatively low and the discrepancy here is probably due to added resistance of the MOSFETs in the ubox. The MOSFET resistance would not be as significant for motors with a higher resistance. 
