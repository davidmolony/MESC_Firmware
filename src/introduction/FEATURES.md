# MESC_Firmware

## Noteable features
* Simple and robust sensorless observer 
* PWM-Synchronous Encoder readings
* HFI using d-q coupled current or 45 degree injection
* Dead time compensation and characterisation
* 100% modulation techniques
* Field weakening and MTPA
* Fast fault shutdown from three possible sources - BRK input (<1clock cycle MCU propogation), ADC Watchdog (optional, runs at PWM frequency by default and guards Out of range) and software comparison.
* Parameter (Rs, Ld, Lq, flux linkage detection, HFI thresholds)
* Operation up to 100kHz+ when running V0 only (HFI, encoder, logging disabled)
* Operation up to ~70kHz PWM (140kHz V0V7 frequency) with F405 MCU, and some options disabled (e.g. SPI encoder). Operation to about 40kHz with F401 MCU and about 35kHz with F303. Stable to <2.5kHz PWM frequency, though this is definitely not advised for most applications.
* Most hardware cannot cope with current measurements above about 60kHz, noise becomes prohibitive.
* Easy porting to any STM32 with a floating point unit and timer1
* Probably easily portable to any other MCU with FPU, 3 phase timer and a 1MHz+ ADC