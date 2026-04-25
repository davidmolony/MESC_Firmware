# davidmolony.github.io/MESC_Firmware/

Documentation will be gradually created/migrated to github pages.

View the book [here](https://davidmolony.github.io/MESC_Firmware/)

# MESC_Firmware
MESC is a project for embedded BLDC FOC, serving a number of purposes
1) Easy to follow and learn FOC
2) Easy to port to other platforms
3) High performance motor control offering all the FOC goodies: Sensorless, HFI, Encoder, Hall, (and combinations of), Field weakening, MTPA, Torque, Speed and Duty control.
4) Permissive licensing making commercial use easy (Additional conditions attached to integration into other open source projects).

# Firmware comments

- This code is so out of date it is scary
- But it has several important updates, which are ground fault protection and CAN input
- It requires that one of the ADC inputs does not have a pullups, and connects a pullup resistor located near the hall sensor in the throttle
- If anything happens to that ground, the pullup shuts off the controller
- It also connects to a specialized [Axis controller]https://github.com/davidmolony/MESC_Firmware/tree/master/Axis-Throttle created by David
- In the MESC firmware, requires that can_adc = 11, input_opt = 33
- This uses a custom PCB with the axis throttle that fits in handle bar, Owen is not sure where the throttle file is. 
- Uses get_ext_failsafe.bin for the controller, which could also be recompiled from code that is in this branch
  
