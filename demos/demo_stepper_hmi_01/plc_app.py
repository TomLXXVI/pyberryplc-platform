from pyberryplc.core.plc import AbstractPLC
from pyberryplc.core.shared_data import SharedData
from pyberryplc.stepper import (
    TMC2208StepperMotor,
    PinConfig,
    TMC2208UART,
    RotatorType,
    Direction
)
from pyberryplc.motion.single_axis import TrapezoidalProfile


class StepperMotorPLC(AbstractPLC):
    
    def __init__(self, shared_data: SharedData, logger=None):
        super().__init__(scan_time=0.1, shared_data=shared_data, logger=logger)

        # Set up stepper motor driver
        self.stepper = TMC2208StepperMotor(
            pin_config=PinConfig(
                step_pin_ID=21,
                dir_pin_ID=27
            ),
            logger=self.logger,
            name="motor X",
            uart=TMC2208UART(port="/dev/ttyUSB1")
        )
        self.stepper.attach_rotator(RotatorType.DYNAMIC_THREADED)
        self.stepper.rotator.profile = TrapezoidalProfile(v_m=90.0, dt_ini=0.5, dt_tot=720.0)
        
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

            # Stepper driver configuration via UART
            self.stepper.enable(high_sensitivity=True)
            self.stepper.configure_microstepping(
                resolution="1/16",
                ms_pins=None,
                full_steps_per_rev=200
            )
            self.stepper.set_current_via_uart(
                run_current_pct=35.0,
                hold_current_pct=10.0
            )
            
            self.logger.info("Driver enabled and configured.")
                        
            self.X0.activate()
            
    def _sequence_control(self):
        if self.X0.active and not self.stepper.rotator.busy:
            if self.start_motor_left.rising_edge and not self.start_motor_right.active:
                self.X0.deactivate()
                self.X1.activate()
            if self.start_motor_right.rising_edge and not self.start_motor_left.active:
                self.X0.deactivate()
                self.X2.activate()
        
        if self.X1.active and self.stepper.rotator.busy and self.stop_motor.rising_edge:
            self.X1.deactivate()
            self.X0.activate()
        
        if self.X2.active and self.stepper.rotator.busy and self.stop_motor.rising_edge:
            self.X2.deactivate()
            self.X0.activate()
    
    def _execute_actions(self):
        if self.X0.rising_edge and self.stepper.rotator.busy:
            self.stepper.rotator.stop()
            
        if self.X1.rising_edge:
            self.stepper.rotator.direction = Direction.COUNTERCLOCKWISE
            self.stepper.rotator.start()
         
        if self.X2.rising_edge:
            self.stepper.rotator.direction = Direction.CLOCKWISE
            self.stepper.rotator.start()
    
    def _update_statuses(self):
        self.status_motor_running.update(self.stepper.rotator.busy)
    
    def control_routine(self):
        self._init_control()
        self._sequence_control()
        self._execute_actions()
        self._update_statuses()
    
    def exit_routine(self):
        self.logger.info("Exiting PLC routine. Shutting down...")
        self.stepper.disable()
        self.stepper = None

    def emergency_routine(self):
        self.logger.warning("Emergency routine invoked. Shutting down...")
        self.stepper.disable()
        self.stepper = None

    def crash_routine(self, exception):
        self.logger.error("PLC crash routine invoked. Shutting down...")
        self.stepper.disable()
        self.stepper = None
        raise exception
