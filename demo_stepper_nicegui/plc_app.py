from pyberryplc.core.plc import AbstractPLC
from pyberryplc.core.shared_data import SharedData
from pyberryplc.stepper import TMC2208StepperMotor, TMC2208UART
from pyberryplc.motion_profiles import TrapezoidalProfile


class StepperMotorPLC(AbstractPLC):
    
    def __init__(self, shared_data: SharedData, logger=None):
        super().__init__(scan_time=0.1, shared_data=shared_data, logger=logger)
        
        self.motor = TMC2208StepperMotor(
            step_pin=27,
            dir_pin=26,
            microstep_resolution="1/8",
            uart=TMC2208UART(port="/dev/ttyAMA0"),
            logger=self.logger,
        )
        
        self.profile = TrapezoidalProfile(
            v_m=90.0,
            dt_acc=0.5,
            ds_tot=720.0
        )
        
        self.X0 = self.add_marker("X0")
        self.X1 = self.add_marker("X1")
        self.X2 = self.add_marker("X2")
        
        self._init_done = False
        
        self.start_motor_left = self.hmi_input_register["start_motor_left"]
        self.start_motor_right = self.hmi_input_register["start_motor_right"]
        self.stop_motor = self.hmi_input_register["stop_motor"]
        
        self.status_motor_running = self.hmi_output_register["motor_running"]
    
    def _init_control(self):
        if not self._init_done:
            self._init_done = True
            self.motor.enable()
            self.motor.set_microstepping()
            self.motor.set_current_via_uart(run_current_pct=19, hold_current_pct=10)
            
            self.logger.info("Driver enabled and configured.")
                        
            self.X0.activate()
            
    def _sequence_control(self):
        if self.X0.active and not self.motor.busy:
            if self.start_motor_left.rising_edge and not self.start_motor_right.active:
                self.X0.deactivate()
                self.X1.activate()
            if self.start_motor_right.rising_edge and not self.start_motor_left.active:
                self.X0.deactivate()
                self.X2.activate()
        
        if self.X1.active and self.motor.busy and self.stop_motor.rising_edge:
            self.X1.deactivate()
            self.X0.activate()
        
        if self.X2.active and self.motor.busy and self.stop_motor.rising_edge:
            self.X2.deactivate()
            self.X0.activate()
    
    def _execute_actions(self):
        if self.X0.rising_edge and self.motor.busy:
            self.motor.stop_rotation_dynamic()
            
        if self.X1.rising_edge:
            self.motor.start_rotation_dynamic(self.profile, "forward")
         
        if self.X2.rising_edge:
            self.motor.start_rotation_dynamic(self.profile, "backward")
    
    def _update_statuses(self):
        self.status_motor_running.update(self.motor.busy)
    
    def control_routine(self):
        self._init_control()
        self._sequence_control()
        self._execute_actions()
        self._update_statuses()
    
    def exit_routine(self):
        self.motor.disable()

    def emergency_routine(self):
        self.motor.disable()

    def crash_routine(self, exception):
        self.motor.disable()
        raise exception
