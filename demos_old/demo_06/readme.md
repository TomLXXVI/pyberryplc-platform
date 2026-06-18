# DEMO 6 : TRAJECTORY PLANNING AND EXECUTION

Hardware used:
- 1 Raspberry Pi 4
- 3 stepper drivers BIGTREETECH TMC2208 v3.0
- 3 stepper motors NEMA17 24 Vdc - 1.3 A

In the previous demo 5, the motion axes were manually and individually 
controlled via push buttons (jog mode). In this demo, a 2D trajectory is 
executed segment by segment, with motion axes moving simultaneously.

The trajectory is first created in the script `trajectory_planning.py` and saved
as a JSON file. The 2D trajectory is defined by a series of points in the XY 
plane. Each pair of consecutive points defines a segment of the trajectory.

The trajectory is executed in the script `trajectory_execution.py`, which is 
started via the shell script of the same name (with file extension .sh).

The path is executed using the `XYZMotionController` class. The stepper motors 
are configured using a TOML file, the path of which is passed to the 
`XYZMotionController`. The program sequence is started by pressing the s key on 
the keyboard. In step X1, the path file (JSON file) is loaded into the 
`XYZMotionController`. In step X2, the path is executed segment by segment. 
After each segment, the s key must be pressed again to execute the next segment. 
When all segments have been executed, the PLC program is exited.
