# MESC Firmware and documentation
## see this [Using the STM32CubeIDE debugger with MP2-ESC](https://www.youtube.com/watch?v=OsqP2L_DqyM)

## Using the STM32CubeIDE debugger
These are variables to load up in the live expression viewer of STM32CubeIDE:
+ MESC_error
+ mtr[0]

Some variables that are interesting
+ mtr[0].MotorState
+ mtr[0].Raw.ADC_in_ext1
+ mtr[0].Conv.VBus

## Interpretting MESC_error values, example
when we turned down the power supply mtr[0].Conv.VBus changed, and when it went below ABS_MIN_BUS_VOLTAGE, the debugger reported MESC_errors = 16

To figure out what that means, convert MESC_error to binary, which is 10000

The look at:
MESC_Common/Inc/MESC_MOTOR_DEFAULTS.h

10000 corresponds to the fifth error in this list: 

```
#define ERROR_OVERCURRENT_PHA 1
#define ERROR_OVERCURRENT_PHB 2
#define ERROR_OVERCURRENT_PHC 3
#define ERROR_OVERVOLTAGE 4
#define ERROR_UNDERVOLTAGE 5 // 16 = 10000 in binary -- the one means fifth error in list
#define ERROR_BRK 6
#define ERROR_OVERTEMPU 7
#define ERROR_OVERTEMPV 8
#define ERROR_OVERTEMPW 9
#define ERROR_OVERTEMP_MOTOR 10
#define ERROR_HARDFAULT 11
#define ERROR_BUSFAULT 12
#define ERROR_NMI 13
```

## Using the MESC terminal

You'll need to connect your VT100 terminal to the port. Here are some links to find your com port:
+ https://www.carminenoviello.com/2015/03/02/how-to-use-stm32-nucleo-serial-port/
+ https://shawnhymel.com/1795/getting-started-with-stm32-nucleo-usb-virtual-com-port/
+ https://www.youtube.com/watch?v=92A98iEFmaA
+ https://www.youtube.com/watch?v=dEQwSl8mCFs

My favorite command line method to use the terminal. This doesnt work for windows:
```
Find your available port with:
$ ls /dev/* | grep cu
/dev/cu.BLTH
/dev/cu.Bluetooth-Incoming-Port
/dev/cu.usbmodem3555356532321

Then use this screen command with the port name:
$ screen /dev/cu.usbmodem3555356532321 9600
```
to stop the screen command type: "Ctrl-a k (yes)"

## MESC terminal commands used during the tutorial

You'll need to go through the tutorial to see how these are used, but here's a summary of MESC terminal commands.

To remove all previously saved values:
```
save -d 
```

An example of setting a variable:
```
set curr_max 30
```

View all variables with get:
```
get
```

Or use get to see a specific value:
```
get motor_sensor
```

One way to spin the motor is go to open loop, and then use uart_req:
```
set motor_sensor 2
  Parameter   | Value             | Min      | Max      | Description
  motor_sensor| 2                 | 0        | 30       | 0=SL, 1=Hall, 2=OL, 3=ABSENC, 4=INC_ENC, 5=HFI

set uart_req 0 
  Parameter   | Value             | Min      | Max      | Description
  uart_req    | 0.000000          | -1000.00 | 1000.00  | Uart input
```

Make sure you're taking input from the UART:
```
set input_opt 8
```

Set the angle of rotation for each PWM period:
```
set ol_step 20
```

Spin the motor:
```
set uart_req  20
```

Stop the motor:
```
set uart_req  0
```

You can change from to openloop:
```
set motor_sensor 0
  Parameter   | Value             | Min      | Max      | Description
  motor_sensor| 0
```

This is useful to stream values to the terminal:
```
status json
```
To stop streaming:
```
status stop
```

When ever you make changes that you like, type:
```
save
```

