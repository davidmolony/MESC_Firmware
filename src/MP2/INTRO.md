# MESC Reference ESC: The Multi-Platform, Modular, Powerful (MP2) Speed Control

The MESC project is migrating away from the original F303 hardware. This is for a few reasons:
1) ST decided to simply stop selling F303s for several years and even in 2023 they are still 3x the price they were in 2020.
2) The F303 is very much an old MCU, core clock MHz is low. USB peripheral is less good than the F4/L4.
3) The F3 onboard opamps are a blessing and a curse. They reduce peripheral count, but it is hard to do differential shunt routing. Not an issue at low currents with 5+mohm resistors, but crippling beyond ~100A.

The MP2 is a great platform since it allows easy testing with a variety of MCU targets. I can test with F401 and F411 "blackpills" from ebay, or the F405 pill made by Owhite for either the F405 MCU or a 64pin L4 series MCU.

The cost of the MP2 is very low, and the parts easily swappable and cheap to replace. Full boards can be built for about 60$ each for 5 off including a pill, depending on the choice of MOS an caps (perhaps less if you build it with only 6/18MOS and 1/6 capacitors). For that, you get bursts approaching 300A and continuous of ~120A with a 20s battery.
The application of all features required for MESC plus non compulsorary ones like hardware overcurrent and voltage protection, broad range of powers it can cover and the low costs and the open source nature make this an excellent reference board for the project. There are definitely better performing ESCs out there, but they cost many times as much and the design files are not accessible to all.

There are occasionally group orders of parts made in Europe and the US, so it may be possible to get one or two offs SMT populated. The design repo tries to keep a BoM plus alternatives and set of GERBERs that can be easily ordered from your favourite PCB house. 

##The MP2 Features
* Relatively compact design, fitting for ebikes / medium-large electric scooters.
* Inexpensive, optimized for low cost assembly by JLCPCB
* 100-300A operation with 18 FET, bus bars, and cooling
* Modular, with easily replaceable modules 
* Compatible with several open source projects including VESC, EBICS, SmartESC, STM32 Motor Control Workbench and MESC

Documentation for [MP2 project](https://github.com/badgineer/MP2-ESC):
* Gathering motor parameters [[LINK](https://github.com/badgineer/MP2-ESC/docs/MOTOR_PARAM.md)]
* Pin mappings between MP2 and the F405 pill [[LINK](https://github.com/badgineer/MP2-ESC/docs/MP2_F405PILL_PINOUTS.md)]
* MP2 assembly, testing and firmware [[LINK](https://github.com/badgineer/MP2-ESC/docs/PCB_ASSEMBLY_TESTING.md)]
* MP2 bus bar methods [[LINK](https://github.com/badgineer/MP2-ESC/docs/HIGHER_AMP_ASSEMBLY.md)]
* Some (bad) examples of connecting the MP2 to a motor [[LINK](https://github.com/badgineer/MP2-ESC/docs/QS165_MP2_WIRING.md)]
* Gallery of enclosures [[LINK](https://github.com/badgineer/MP2-ESC/docs/ENCLOSURE_GALLERY.md)]
* MESC Firmware on the MP2 -- getting started with STM32CubeIDE [[LINK](https://github.com/badgineer/MP2-ESC/docs/FIRMWARE_INTRO.md)]

