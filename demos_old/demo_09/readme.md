# DEMO 9 : MULTIPLE PLC-UNITS

Demo of how multiple machine-units - each running their own PLC program - can be
implemented using threading and shared data memory.

In this demo, a loading station transfers items to a rotary disk of a
multi-operation machine. The loading station and the multi-operation machine each 
run their own PLC program. The multi-operation machine must signal to the loading 
station when it is ready to accept a new item.

The demo also demonstrates the implementation of a stepper motor driver that 
runs in a separate subprocess of the operating system and communicates with the 
loading station through a TCP/IP socket.
