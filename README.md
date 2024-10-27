# davidmolony.github.io/MESC_Firmware/

Documentation will be gradually created/migrated to github pages.

View the book [here](https://davidmolony.github.io/MESC_Firmware/)

# MESC_Firmware
MESC is a project for embedded BLDC FOC, serving a number of purposes
1) Easy to follow and learn FOC
2) Easy to port to other platforms
3) High performance motor control offering all the FOC goodies: Sensorless, HFI, Encoder, Hall, (and combinations of), Field weakening, MTPA, Torque, Speed and Duty control.
4) Permissive licensing making commercial use easy (Additional conditions attached to integration into other open source projects).

# Targets/Hardware
MESC runs primarily on any STM32 target with an FPU. Tested with the targets in the repo, but easily portable to any other STM. Compatibility with other MCU brands TBC.

The reference hardware is now the [MP2 ESC](https://github.com/badgineer/MP2-ESC), since it allows testing with many targets by simply swapping the MCU pill. It also offers adequate performance for most light EV applications (high power scooters and ~10kW E-motorbikes.

The original hardware based on F303 target [MESC_FOC_ESC](https://github.com/davidmolony/MESC_FOC_ESC) will be supported for some time, but not encouraged.

All STM32F405RG hardware (AKA VESC compatible hardware e.g. Trampa, SHUL, Triforce, FSESC and many others) compatible with MESC_firmware.

With thanks to all that have helped in the creation of MESC.

Happy spinning!
(testpush)