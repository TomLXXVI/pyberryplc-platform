# DEMO 3 : Single Stepper Motor - Non-Blocking Mode - Motion Profile

Hardware used:
- 1 Raspberry Pi 4
- 1 stepper driver MKS TMC2208 v2.0
- 1 stepper motor NEMA17 12 Vdc - 350 mA (model XY42STH34-0354A)
- 2 LEDs (red and yellow) and 2 resistors 220 Ohm

One stepper motor executes a predefined motion profile in non-blocking mode, 
either in the forward rotation direction when the f-key is pressed, or the 
backward rotation direction when the b-key is pressed.

“Non-blocking” means that the PLC scan cycle can continue while the motor is
rotating. This is achieved by sending the step pulse signal to the driver in a 
separate thread. To demonstrate the non-blocking behavior, an LED is turned on 
after the command to start a rotation is given. When forward rotation is active,
a yellow LED (L1) is lit. When backward rotation is active, a red LED (L2) is 
lit.

The stepper motor is controlled by a TMC2208 driver, which is configured via 
its UART-interface.

The Python script is started from a shell script to allow the keyboard buttons 
to be used as input for the PLC program. The shell script starts the Python 
script and its associated virtual environment as the root user, because the 
external package `keyboard` requires the root user.
