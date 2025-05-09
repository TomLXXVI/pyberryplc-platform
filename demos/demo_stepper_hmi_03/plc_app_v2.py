import time
from multiprocessing import Pipe

from pyberryplc.utils.log_utils import init_logger
from pyberryplc.core import AbstractPLC, SharedData
from pyberryplc.stepper import TMC2208StepperMotor, TMC2208UART, StepperMotorProcess


logger = init_logger()


class XYMotionPLC(AbstractPLC):

    def __init__(
        self,
        shared_data: SharedData,
        x_port: str = "/dev/ttyUSB1",
        y_port: str = "/dev/ttyUSB0"
    ) -> None:
        super().__init__(shared_data=shared_data, logger=logger)

        # Create pipes for interprocess communication
        self._px_parent, px_child = Pipe()
        self._py_parent, py_child = Pipe()
        
        def _config_motor_x(motor: TMC2208StepperMotor) -> None:
            motor.set_microstepping()
            motor.set_current_via_uart(run_current_pct=19, hold_current_pct=10)
        
        # Launch stepping motor processes
        self._proc_x = StepperMotorProcess(
            conn=px_child,
            motor_class=TMC2208StepperMotor,
            motor_kwargs={
                "step_pin": 21,
                "dir_pin": 27,
                "microstep_resolution": "1/4",
                "uart": TMC2208UART(port=x_port),
                "logger": self.logger,
                "name": "motor X"
            },
            config_callback=_config_motor_x,
            name="motor X proc"
        )

        def _config_motor_y(motor: TMC2208StepperMotor) -> None:
            motor.set_microstepping()
            motor.set_current_via_uart(run_current_pct=19, hold_current_pct=10)
        
        self._proc_y = StepperMotorProcess(
            conn=py_child,
            motor_class=TMC2208StepperMotor,
            motor_kwargs={
                "step_pin": 19,
                "dir_pin": 20,
                "microstep_resolution": "1/4",
                "uart": TMC2208UART(port=y_port),
                "logger": self.logger,
                "name": "motor Y"
            },
            config_callback=_config_motor_y,
            name="motor Y proc"
        )

        self._proc_x.start()
        self._proc_y.start()

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

    def _init_control(self):
        if not self._init_flag:
            self._init_flag = True
            self.logger.info("Stepper motor processes started.")
            self.X0.activate()

    def _sequence_control(self):
        # Check for 'done' messages from motor subprocesses
        if self._px_parent.poll():
            msg = self._px_parent.recv()
            if msg.get("status") == "ready":
                self._x_prepared = True
            elif msg.get("status") == "done":
                self._x_busy = False
                self._x_prepared = False
                
        if self._py_parent.poll():
            msg = self._py_parent.recv()
            if msg.get("status") == "ready":
                self._y_prepared = True
            elif msg.get("status") == "done":
                self._y_busy = False
                self._y_prepared = False

        # Update HMI output markers
        self.motor_x_ready.update(self._x_prepared)
        self.motor_x_busy.update(self._x_busy)
        self.motor_y_ready.update(self._y_prepared)
        self.motor_y_busy.update(self._y_busy)

        if self.X0.active and self.start_motion.active:
            self.logger.info("Start motion.")
            self.X0.deactivate()
            self.X1.activate()

        if self.X1.active and self._all_profiles_ready():
            self.logger.info("Ready to move.")
            self.X1.deactivate()
            self.X2.activate()
        
        if self.X2.active and self._motion_finished():
            self.logger.info("Motion finished.")
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
            start_time = time.perf_counter() + 0.2
            self.logger.info(f"Synchronized start time: {start_time:.6f}")

            self._px_parent.send({"cmd": "start", "start_time": start_time})
            self._py_parent.send({"cmd": "start", "start_time": start_time})

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
