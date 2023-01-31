# MP2 Assembly and Testing


## MP2 PCB assembly
The following is not a step by step guide for finishing the MP2 but it still may be useful for people putting together their first PCB. 

Note: in this assembly we have not added any extra copper along the bus bars which is a common practice to increase the total amount of amperage used for these boards. Adding thicker bus bars is shown here [[LINK]](../docs/HIGHER_AMP_ASSEMBLY.md). However, **we recommend that if you are going to solder bus bars on your MP2 that you begin with that step first**. 

There are MANY tutorials on how to solder and hopefully this is not your first rodeo. At a minimum you are going to need a high wattage soldering iron, decent quality solder, hopefully some flux remover and a multimeter to be able solder this board. 

Looking at the top side of the board, it tends to be easiest to solder in the shorter parts first (so they rest flat on the bench when getting soldered in. So a possible order is:
* JST connectors
* 12V to 5V DC-DC converter
* the larger 12V DC-DC converter
* 20 pin headers

When doing the 20 pin headers...

<img src="pics/PCB_ASSEMBLY01.png" title="header pin alignment">

Another suggestion:

<img src="pics/PCB_ASSEMBLY02.png" title="use flux remover">

The 12V-5V DC converter sits funny:

<img src="pics/PCB_ASSEMBLY03.png" title="DC-DC sits funny">

Heat shrink tubing is your friend!

<img src="pics/PCB_ASSEMBLY04.png" title="soldering Vbat and GND">

Use precision:

<img src="pics/PCB_ASSEMBLY05.png" title="watch out for components">
<img src="pics/PCB_ASSEMBLY06.png" title="watch out for solder bridges">

**NOTE:** These pics are included to talk about soldering issues, **BUT BE ADVISED WE STRONGLY RECOMMEND** users take advantage of the three holes on each phase to solder in larger wires and get improved current sharing with the MOSFETs. A better example is shown here [[LINK](pics/HIGH_AMP_ASSEMBLY02.jpeg)] and take a look at [THIS FUTURE DOCUMENTATION] for other higher amperage connections. 

<img src="pics/PCB_ASSEMBLY07.png" title="it aint pretty">

## MP2 continuity testing
Before you're finished with soldering be sure to do some important testing of your board. One set of tests involved using a continuity checker on your multimeter. This is a **short video** to help you get started:

[![MP2 continuity checking](https://img.youtube.com/vi/L9bziAqBU64/0.jpg)](https://www.youtube.com/watch?v=L9bziAqBU64)

Once your set up for testing run through the following tests:

<img src="pics/PCB_ASSEMBLY08.png" title="pcb continuity chart">

<img src="pics/PCB_ASSEMBLY09.png" title="VBat continuity check">

<img src="pics/PCB_ASSEMBLY10.png" title="GND continuity check">

<img src="pics/PCB_ASSEMBLY11.png" title="Phase wire continuity check">

## MP2 resistance testing
Since you have your multimeter out there are some other useful measurements to take. These tests can vary based on what type of meter you have. Also sometimes the readings will take a while to settle down to the final value.

<img src="pics/PCB_ASSEMBLY12.png" title="Phase wire resistance check1">

<img src="pics/PCB_ASSEMBLY13.png" title="Phase wire resistance check2">

<img src="pics/PCB_ASSEMBLY14.png" title="Ground to testpoint check">

<img src="pics/PCB_ASSEMBLY15.png" title="Ground to VBat check">

## MP2 powering up

Hopefully you have experience powering up a PCB for the first time, but here a subset of things you can do when starting up the MP2. 

Connect a switch or put in a jumper over on the [middle-left side of the board](pics/PCB_ASSEMBLY16.png). Then: 

* First remove the pill from the MP
* Connect your multimeter to GND
* Apply power to VBat
* Note that in some cases the DC-DC converter wont activate at <42 volts
* Test for voltages on the points shown...

<img src="pics/PCB_ASSEMBLY17.png" title="Voltage check">

If youre voltages look good, power off, plug in the pill and hopefully you'll see a comforting power light on the pill.

## MP2 loading the firmware. 

* Disconnect the F405 pill from the MP2 board;
* Search the internet and break a record ($2USD) for the most inexpensive ["st-link v2"](https://www.google.com/search?q=st-link+v2) in existence;
* Search on ["STM32CubeProgrammer software STM32"](https://www.google.com/search?q=STM32CubeProgrammer+software+STM32&oq=STM32CubeProgrammer+software+STM32), download and install the programmer code;
* Search, find, 'request from a friend' or compile for yourself a copy of the VESC bin file for the F405 pill;
* Search ["st-link V2 STM32 programming bluepill"](https://www.google.com/search?q=st-link2+STM32+programming+bluepill) to learn how to [connect](pics/IDE_7_stlink.png) your st-link to a pill;
* Use the STM32CubeProgrammer to upload the bin file;
* Search, find, 'request from a friend' or compile for yourself a copy of the VESCtool;
* Do not plug the pill into the MP2, upload the VESC firmware to the pill, connect the USB and start VESCtool;

Select "Connection" on the left hand side of VESCtool. If you see this on your screen:
<img src="pics/PCB_ASSEMBLY18.png" title="VESC tool">

That is good news.  

