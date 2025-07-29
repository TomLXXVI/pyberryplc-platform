 # DEMO 2 : Single Stepper Motor - Blocking Mode - Motion Profile

Hardware used:
- 1 Raspberry Pi 4
- 1 stepper driver MKS TMC2208 v2.0
- 1 stepper motor NEMA17 12 Vdc - 350 mA (model XY42STH34-0354A)

One stepper motor executes a predefined motion profile in blocking mode, either 
in the forward rotation direction when the f-key is pressed, or the backward 
rotation direction when the b-key is pressed.

“Blocking” means that the PLC scan cycle is blocked until the movement is 
completed.

The stepper motor is controlled by a TMC2208 driver, which is configured via 
its UART-interface.

The Python script is started from a shell script to allow the keyboard buttons 
to be used as input for the PLC program. The shell script starts the Python 
script and its associated virtual environment as the root user, because the 
external package `keyboard` requires the root user.
