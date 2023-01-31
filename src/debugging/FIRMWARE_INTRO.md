# GETTING STARTED: Install and Debug MESC Firmware

## Debugging in CubeIDE
Almost all variables of interest can be found in the struct mtr[]
mtr[] is an array of structs containing the FOC, the raw and converted ADC values, the states, the motor parameter...
mtr[] can contain n motors. Adding this to the live expressions allows you to view everything about the setup.
mtr[n]->mtimer points to the instance of the ST timer.
mtr[n]->stimer points to the timer used for the slow loop.
Mutliple motors can be accomodated, but need patching to the correct ADC readings, which need triggering byt the associated timer.
Some variables are assumed to be the same for multiple motors e.g. current limits, voltage limits... 
These can be found in struct g_hw_setup

## Getting started
Install STM32CubeIDE from https://www.st.com/en/development-tools/stm32cubeide.html

On the command line clone the github repository using:
```
% git clone https://github.com/davidmolony/MESC_Firmware.git
```

Start up the STM32CubeIDE. A workspace area is used by STM32CubeIDE to store all of your projects. The first time you start STM32CubeIDE it will want to create a workspace, if you dont have one. 

<img src="pics/IDE_1_create_workspace.png" title="Creating a workspace">

Create a name if needed, select "Launch" 

When the IDE comes up for the first time, it may show an information center.

<img src="pics/IDE_2_splashscreen.png" title="The IDE splash screen">

Close on this feature.

The IDE may show a project explorer tab on the left, if you do not see the project explorer:

Window-->Show View-->Project explorer


The project explorer may show "Import projects..." select that, or if you do not see that function, try:

File-->Import...

An import selector should come up:

<img src="pics/IDE_3_import_selector.png" title="Import selection tool">

Set up General-->Existing Projects into Workspace, select Next

Browse for the top-level github folder you created, hit open the import function will show this:

<img src="pics/IDE_4_import_projects.png" title="Importing projects">

Select finish. This should load a set of MESC projects into the Project Explorer, expand the top-level folder and you should see:

At this point if you select

Project-->Build All (or the icons on top of the IDE also help with builds)

you should see the IDE compile the MESC_firmware in all the sub directories. 

<img src="pics/IDE_5_firmware_build.png" title="Compiling firmware">

When you select the .ioc file in the file browser this tool pops up. By working with the pull down menus on each pin, you can view or set the functions on that pin. This tool also allows you to change many other functions like the timers, UARTS, and ADCs. 

<img src="pics/IDE_6_IOC_review1.png" title="View pin functions">

## Debugging -- getting started

To connect up your pill, you will use the abundantly available ST-LINK V2 debugger, and get connected like this:

<img src="pics/IDE_7_stlink.png" title="ST-LINK pin connections">

There are a couple of settings to watch out for in the configuration of the debugger. Make sure these parameters are set up:
<img src="pics/IDE_8_settings1.png" title="Configure the debugger">

There are a couple of settings to watch out for in the configuration of the debugger. Make sure these parameters are set up:
<img src="pics/IDE_9_settings2.png" title="Configure the debugger">

Once your st-link is connected and the debugger is configured you can get started by hitting this button. Sometimes you have to use that button's pulldown menu to select the debugging style that you want: 
<img src="pics/IDE_10_debug.png" title="ST-LINK pin connections">

## Debugging -- what to do next

There are approximately 10<sup>5</sup> google results for [STM32cubeIDE debugging](https://www.google.com/search?q=stm32cubeide+debugging). Do your best. 

