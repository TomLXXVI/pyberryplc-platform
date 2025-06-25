"""
Automatic Mode Subroutine
-------------------------
Program that loads a trajectory JSON file. When the user presses the 's' key on 
the keyboard of the Raspberry Pi, a segment of the trajectory is executed. At 
the end of a segment, a lamp turns on for 3 seconds. When the lamp turns off, 
the user can press the 's' key again to run the next segment of the trajectory. 
When the final segment has been executed and the user presses the 's' key again,
the cycle is repeated. To exit the program, <Ctrl-Z> needs to pressed (from the 
computer where the console that launched the program, is running).
The 'e' key serves as an emergency button. When the 'e' key is pressed an 
`EmergencyException` is raised that starts the `emergency_routine()` of the
main program.
"""
from pyberryplc.core import TAbstractPLC, MemoryVariable, EmergencyException
from pyberryplc.core.timers import TimerOnDelay
from pyberryplc.core.counters import CounterDown
from pyberryplc.utils.keyboard_input import KeyInput
from pyberryplc.stepper.controller import XYZMotionController, MotionControlStatus


class AutomaticModeSubRoutine:
    
    def __init__(
        self, 
        main_routine: TAbstractPLC, 
        cfg_filepath: str,
        prg_filepath: str
    ) -> None:
        self.main = main_routine
        self.prg_filepath = prg_filepath
        self.logger = self.main.logger
        self.key_input = KeyInput()
        
        self.xyz_controller = XYZMotionController(self.main, cfg_filepath, self.logger)
        self.motion_control_status: MotionControlStatus | None = None
        
        self.L1, _ = self.main.add_digital_output(23, "L1")
        self.T1 = TimerOnDelay(3)
        self.C1 = CounterDown(0)

        self.X0 = self.main.add_marker("Auto.X0")
        self.X1 = self.main.add_marker("Auto.X1")
        self.X2 = self.main.add_marker("Auto.X2")
        self.X3 = self.main.add_marker("Auto.X3")
        self.X4 = self.main.add_marker("Auto.X4")

        self.trajectory_loaded = MemoryVariable()
        self.trajectory_finished = MemoryVariable()
        self.num_segments = 0
        self._init_flag = True
        
    def _init_control(self) -> None:
        if self._init_flag:
            self._init_flag = False
            self.xyz_controller.enable()
            self.logger.info("Press 's' to start.")
            self.X0.activate()

    def _sequence_control(self) -> None:
        if self.key_input.rising_edge("e"):
            raise EmergencyException
        
        if self.X0.active and self.key_input.rising_edge("s"):
            self.logger.info("Loading trajectory...")
            self.X0.deactivate()
            self.X1.activate()

        if self.X1.active and self.trajectory_loaded.active:
            self.logger.info("Running trajectory...")
            self.X1.deactivate()
            self.X2.activate()

        if self.X2.active and self.motion_control_status.ready and self.C1.value < self.num_segments:
            self.logger.info(f"Movement {self.num_segments - self.C1.value} is completed.")
            self.logger.info("Turn L1 on for 3 s.")
            self.X2.deactivate()
            self.X3.activate()

        if self.X3.active and self.T1.has_elapsed:
            self.X3.deactivate()
            self.X4.activate()

        if self.X4.active:
            if self.X4.rising_edge:
                if not self.trajectory_finished.active:
                    self.logger.info("Press 's' again to continue.")
                else:
                    self.logger.info("Trajectory completed. Going back to X0.")
                    self.logger.info("Press 's' to repeat the trajectory.")

            if not self.L1.active:
                if not self.trajectory_finished.active and self.key_input.rising_edge("s"):
                    self.X4.deactivate()
                    self.X2.activate()
                elif self.trajectory_finished.active:
                    self.X4.deactivate()
                    self.X0.activate()

    def _execute_actions(self) -> None:
        if self.X0.rising_edge:
            self.trajectory_loaded.deactivate()
            self.trajectory_finished.deactivate()
            self.C1.reset()

        if self.X1.rising_edge:
            self.num_segments = self.xyz_controller.load_trajectory(self.prg_filepath)
            self.C1.set(self.num_segments)
            self.trajectory_loaded.activate()

        if self.X2.rising_edge:
            if self.C1.value > 0:
                self.xyz_controller.move()
                self.C1.count_down()
            if self.C1.value == 0:
                self.trajectory_finished.activate()

        if self.X3.rising_edge:
            self.L1.activate()

        if self.X4.rising_edge:
            self.L1.deactivate()
            self.T1.reset()
    
    def call(self) -> None:
        self.key_input.update()
        self.motion_control_status = self.xyz_controller.get_status()
        self._init_control()
        self._sequence_control()
        self._execute_actions()
    
    def exit(self) -> None:
        self.xyz_controller.disable()
        self.logger.info("Motion controller disabled.")
