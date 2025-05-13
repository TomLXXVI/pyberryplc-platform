import logging
from multiprocessing import Pipe

from pyberryplc.core import AbstractPLC, SharedData
from pyberryplc.stepper import (
    TMC2208StepperMotor,
    PinConfig,
    TMC2208UART, 
    StepperMotorProcess
)


class XYMotionPLC(AbstractPLC):

    def __init__(
        self,
        shared_data: SharedData,
        logger: logging.Logger,
    ) -> None:
        super().__init__(shared_data=shared_data, logger=logger)

        # Create pipes for interprocess communication
        self._px_parent, px_child = Pipe()
        self._py_parent, py_child = Pipe()
        
        def _config_motor(motor: TMC2208StepperMotor) -> None:
            motor.enable(high_sensitivity=True)
            motor.configure_microstepping(
                resolution="full",
                ms_pins=None,
                full_steps_per_rev=200
            )
            motor.set_current_via_uart(
                run_current_pct=35.0,
                hold_current_pct=10.0
            )

        # Launch stepping motor processes
        port_x: str = "/dev/ttyUSB1"
        port_y: str = "/dev/ttyUSB0"

        self._proc_x = StepperMotorProcess(
            conn=px_child,
            motor_class=TMC2208StepperMotor,
            motor_kwargs={
                "pin_config": PinConfig(step_pin_number=21, dir_pin_number=27, use_pigpio=True),
                "logger": self.logger,
                "name": "motor X",
                "uart": TMC2208UART(port=port_x),
            },
            config_callback=_config_motor,
            name="X-axis"
        )
        
        self._proc_y = StepperMotorProcess(
            conn=py_child,
            motor_class=TMC2208StepperMotor,
            motor_kwargs={
                "pin_config": PinConfig(step_pin_number=19, dir_pin_number=20, use_pigpio=True),
                "logger": self.logger,
                "name": "motor Y",
                "uart": TMC2208UART(port=port_y),
            },
            config_callback=_config_motor,
            name="Y-axis"
        )

        self._x_prepared = False
        self._x_busy = False
        self._y_prepared = False
        self._y_busy = False
        self._init_flag = False

        self.X0 = self.add_marker("X0")
        self.X1 = self.add_marker("X1")
        self.X2 = self.add_marker("X2")

        self.start_motion = self.hmi_input_register["start_motion"]
        self.motor_x_ready = self.hmi_output_register["motor_x_ready"]
        self.motor_x_busy = self.hmi_output_register["motor_x_busy"]
        self.motor_y_ready = self.hmi_output_register["motor_y_ready"]
        self.motor_y_busy = self.hmi_output_register["motor_y_busy"]
        
        self.travel_time_x = self.hmi_output_register["travel_time_x"]
        self.travel_time_y = self.hmi_output_register["travel_time_y"]

    def _init_control(self):
        if not self._init_flag:
            self._init_flag = True

            self._proc_x.start()
            self._proc_y.start()
                           
            self.X0.activate()

    def _sequence_control(self):
        # Check for 'done' messages from motor subprocesses
        if self._px_parent.poll():
            msg = self._px_parent.recv()
            status = msg.get("status")
            if status == "ready":
                self._x_prepared = True
            elif status == "done":
                self._x_busy = False
                self._x_prepared = False
                self.travel_time_x.update(msg.get("travel_time", float('nan')))
            elif status == "startup_error":
                self.logger.error(f"[motor X] Startup error: {msg.get('message', 'No message')}")
                raise RuntimeError("Startup error in motor X")
        
        if self._py_parent.poll():
            msg = self._py_parent.recv()
            status = msg.get("status")
            if status == "ready":
                self._y_prepared = True
            elif status == "done":
                self._y_busy = False
                self._y_prepared = False
                self.travel_time_y.update(msg.get("travel_time", float('nan')))
            elif status == "startup_error":
                self.logger.error(f"[motor Y] Startup error: {msg.get('message', 'No message')}")
                raise RuntimeError("Startup error in motor Y")
        
        # Update HMI output markers
        self.motor_x_ready.update(self._x_prepared)
        self.motor_x_busy.update(self._x_busy)
        self.motor_y_ready.update(self._y_prepared)
        self.motor_y_busy.update(self._y_busy)
        
        # Transition logic
        if self.X0.active and self.start_motion.active:
            self.logger.info("Initialize motion")
            self.X0.deactivate()
            self.X1.activate()  # prepare X- and Y-motion profiles

        if self.X1.active and self._all_profiles_ready():
            self.logger.info("Ready to move.")
            self.X1.deactivate()
            self.X2.activate()  # perform X- and Y-rotation
        
        if self.X2.active and self._motion_finished():
            self.logger.info("Motion finished.")
            self.logger.info(f"Travel time in X: {self.travel_time_x.state} s")
            self.logger.info(f"Travel time in Y: {self.travel_time_y.state} s")
            self.X2.deactivate()
            self.X0.activate()

    def _motion_finished(self) -> bool:
        c1 = not self.motor_x_busy.active and self.motor_y_busy.falling_edge
        c2 = not self.motor_y_busy.active and self.motor_x_busy.falling_edge
        if c1 or c2:
            return True
        return False
    
    def _all_profiles_ready(self) -> bool:
        if self.motor_x_ready.active and self.motor_y_ready.active:
            return True
        return False
    
    def _execute_actions(self):
        if self.X1.rising_edge:
            self.logger.info("Prepare motion profiles")
            self._x_busy = True
            self._y_busy = True
            mp_x = self.shared_data.hmi_data["motion_profile_x"]
            mp_y = self.shared_data.hmi_data["motion_profile_y"]
            
            self._px_parent.send({
                "cmd": "prepare_profile",
                "profile": mp_x,
                "direction": "forward",
            })

            self._py_parent.send({
                "cmd": "prepare_profile",
                "profile": mp_y,
                "direction": "forward",
            })

        if self.X2.rising_edge:
            self.logger.info(f"Start motion")
            self._px_parent.send({"cmd": "start"})
            self._py_parent.send({"cmd": "start"})

    def control_routine(self):
        self._init_control()
        self._sequence_control()
        self._execute_actions()

    def exit_routine(self):
        self.logger.info("Exit. Shutting down...")

        self._px_parent.send({"cmd": "shutdown"})
        self._py_parent.send({"cmd": "shutdown"})

        self._proc_x.join()
        self._proc_y.join()

        self.logger.info("Subprocesses stopped.")

    def emergency_routine(self):
        self.logger.warning("Emergency stop. Shutting down...")
        self.exit_routine()

    def crash_routine(self, exception):
        self.logger.error("PLC crashed. Shutting down...")
        self.exit_routine()
        raise exception
