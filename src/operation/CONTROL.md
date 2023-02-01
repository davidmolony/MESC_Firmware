# Control Loops


## Fast Control loop
All the FOC gets done in the fastLoop. This is the only critical part of the MESC, the rest can actually be removed providing the parameters get set and MESC is initialised correctly .
If you remove the slowloop, you can write directly to mtr[n]->FOC.Idq_req.q, set the mtr[n]->MotorState to MOTOR_STATE_RUN and enable the inverter.

### The geometric transforms
Geometric transforms are the Clark and Park, which take the form:
$\begin{bmatrix}I\alpha \\ I\beta \\ I\gamma \end{bmatrix} = 2/3 \begin{bmatrix}1 & -0.5 & -0.5\\0 & \sqrt{3}/2 & -\sqrt{3}/2 \\ 0.5 & 0.5 & 0.5 \end{bmatrix} \begin{bmatrix}Iu \\Iv \\Iw \end{bmatrix} $

### The Sensorless Observer

When \\(a \ne 0$\\), there are two solutions to \\(ax^2 + bx + c = 0\\) and they are: 

$ x = {-b \pm \sqrt{b^2-4ac} \over 2a} $

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

