from typing import Callable, Type, Any, TypeVar
from logging import Logger
from pyberryplc.core import AbstractPLC, SharedData, CounterUp
from pyberryplc.motion.trajectory import Trajectory

from pyberryplc.stepper import (
    StepperMotor,
    TMC2208StepperMotor,
    TMC2208UART,
    PinConfig,
    TrajectoryProcess
)

from config_loader import load_motor_config_toml


TStepperMotor = TypeVar("TStepperMotor", bound=StepperMotor)


class MotorController:
    
    def __init__(
        self,
        motor_name: str,
        motor_class: Type[StepperMotor],
        pin_config: PinConfig,
        cfg_callback: Callable[[TStepperMotor], None] | None = None,
        comm_port: str = "",
        logger: Logger | None = None,
    ) -> None:
        """Creates a `TrajectoryProcess` in which a `StepperMotor` instance will
        be instantiated.
        
        Parameters
        ----------
        motor_name:
            A name to identify the motor.
        motor_class:
            Specific derived class of abstract base class `StepperMotor` that
            will be instantiated inside the `TrajectoryProcess` instance.
        pin_config:
            Configuration of the GPIO pins that will be used to control the 
            stepper driver.
        cfg_callback:
            Callback function that accepts the derived `StepperMotor` object
            and will be called from inside the `TrajectoryProcess` to set the 
            stepper driver (enabling the driver, configuration of microstepping, 
            setting maximum run and hold current, ...).
        comm_port:
            In case the `TMC2208StepperMotor` class is used with the 
            UART-interface (class `TMC2208UART`), the serial port to be used 
            needs to be specified.
        logger:
            Logger to log messages from the `StepperMotor` instance to logfile 
            or console.
        """
        self.motor_name = motor_name
        self.motor_class = motor_class
        self.pin_config = pin_config
        self.cfg_callback = cfg_callback
        self.comm_port = comm_port
        self.logger = logger

        from multiprocessing import Pipe
        self.interface, _motor_proc = Pipe()

        motor_kwargs: dict[str, Any] = {
            "pin_config": self.pin_config,
            "logger": logger,
            "name": self.motor_name,
        }
        if issubclass(self.motor_class, TMC2208StepperMotor) and self.comm_port:
            motor_kwargs["uart"] = TMC2208UART(port=self.comm_port)
                
        self._motor_proc = TrajectoryProcess(
            conn=_motor_proc,
            motor_class=self.motor_class,
            motor_kwargs=motor_kwargs,
            config_callback=self.cfg_callback,
            name=self.motor_name
        )
    
    def enable(self) -> None:
        """
        Starts the `run()` method of the `TrajectoryProcess` object.
        """
        self._motor_proc.start()
    
    def disable(self) -> None:
        """
        Finishes the `TrajectoryProcess` object.
        """
        self._motor_proc.join()
    
    def update_status(
        self, 
        callback: Callable[[dict[str, Any]], None]
    ) -> None:
        """Receives status updates from the `TrajectorProcess` object.
        
        Parameters
        ----------
        callback:
            Function that accepts a status message and processes it in the PLC
            application. 
        """
        if self.interface.poll():
            msg = self.interface.recv()
            callback(msg)
    

class XYMotionPLC(AbstractPLC):
    
    def __init__(self, shared_data: SharedData, logger: Logger):
        super().__init__(shared_data=shared_data, logger=logger)
        self.trajectory: Trajectory | None = None
              
        # Internal markers
        self.X0 = self.add_marker("X0")
        self.X1 = self.add_marker("X1")
        self.X2 = self.add_marker("X2")
        self.trajectory_ready = self.add_marker("ready", init_value=False)
        
        # HMI-inputs
        self.start_motion = self.hmi_input_register["start_motion"]
        
        # HMI-outputs (status feedback)
        self.x_motor_busy = self.hmi_output_register["x_motor_busy"]
        self.y_motor_busy = self.hmi_output_register["y_motor_busy"]
        self.x_motor_ready = self.hmi_output_register["x_motor_ready"]
        self.y_motor_ready = self.hmi_output_register["y_motor_ready"]
        self.travel_time_x = self.hmi_output_register["travel_time_x"]
        self.travel_time_y = self.hmi_output_register["travel_time_y"]
        
        # Counters
        self.segment_counter = CounterUp(preset_val=1)
        
        # Internal data variables
        self._init_flag = False
        self.num_segments = 0
    
    def _setup_motion_control(self):
        config = load_motor_config_toml("motor_config.toml")
        x_cfg = config["x_motor"]
        y_cfg = config["y_motor"]

        # Setup of X-axis motor controller
        self.x_motor_controller = MotorController(
            motor_name="X-axis",
            motor_class=TMC2208StepperMotor,
            pin_config=PinConfig(
                step_pin_ID=x_cfg["step_pin_ID"], 
                dir_pin_ID=x_cfg["dir_pin_ID"], 
                use_pigpio=True
            ),
            cfg_callback=lambda m: _config_motor(m, x_cfg),
            comm_port=x_cfg["comm_port"],
            logger=self.logger
        )
        self.x_interface = self.x_motor_controller.interface
        
        # Setup of Y-axis motor controller
        self.y_motor_controller = MotorController(
            motor_name="Y-axis",
            motor_class=TMC2208StepperMotor,
            pin_config=PinConfig(
                step_pin_ID=y_cfg["step_pin_ID"], 
                dir_pin_ID=y_cfg["dir_pin_ID"], 
                use_pigpio=True
            ),
            cfg_callback=lambda m: _config_motor(m, y_cfg),
            comm_port=y_cfg["comm_port"],
            logger=self.logger
        )
        self.y_interface = self.y_motor_controller.interface
        
        # Callback function for configuring the TMC2208 stepper motor drivers
        def _config_motor(motor: TMC2208StepperMotor, cfg: dict) -> None:
            motor.enable(high_sensitivity=cfg.get("high_sensitivity", True))
            motor.configure_microstepping(
                resolution=cfg["microstepping"]["resolution"],
                ms_pins=None,
                full_steps_per_rev=cfg["microstepping"]["full_steps_per_rev"]
            )
            motor.set_current_via_uart(
                run_current_pct=cfg["current"]["run_current_pct"],
                hold_current_pct=cfg["current"]["hold_current_pct"]
            )

    def _init_control(self):
        if not self._init_flag:
            self._init_flag = True
            self._setup_motion_control()
            self.x_motor_controller.enable()
            self.y_motor_controller.enable()
            self.X0.activate()
    
    def _sequence_control(self):
        # Get feedback from motor controllers
        self.x_motor_controller.update_status(self._process_status_x_axis)
        self.y_motor_controller.update_status(self._process_status_y_axis)
        
        # Transition logic
        if self.X0.active and self.start_motion.active:
            self.logger.info("prepare trajectory")
            self.X0.deactivate()
            self.X1.activate()  # prepare trajectory

        if self.X1.active and self.trajectory_ready.active:
            self.logger.info("run trajectory")
            self.trajectory_ready.update(False)
            self.X1.deactivate()
            self.X2.activate()  # execute trajectory

        if self.X2.active and self._trajectory_finished():
            self.logger.info("trajectory done")
            self.X2.deactivate()
            self.X0.activate()

    def _execute_actions(self):
        if self.X0.rising_edge:
            self.trajectory = None
            self.num_segments = 0
            self.segment_counter.reset()
        
        if self.X1.rising_edge:
            # Get trajectory segments
            self.trajectory = self.shared_data.hmi_data["segments"]
            self.num_segments = len(self.trajectory)
            self.trajectory_ready.update(True)
            self.x_motor_busy.update(True)
            self.y_motor_busy.update(True)
            self.x_motor_ready.update(True)
            self.y_motor_ready.update(True)
        
        if self.X2.active and self.x_motor_ready.active and self.y_motor_ready.active:
            # Execute trajectory segment by segment
            self.logger.info(f"run segment {self.segment_counter.value}")

            # block step X2 while a segment is still running
            self.x_motor_ready.update(False)
            self.y_motor_ready.update(False)
            
            segment = self.trajectory[self.segment_counter.value - 1]
            self.shared_data.hmi_data["segment_count"] = self.segment_counter.value
            
            self.x_interface.send({
                "cmd": "start_segment",
                "delays": segment.x_delays,
                "direction": segment.x_direction,
            })
            self.y_interface.send({
                "cmd": "start_segment",
                "delays": segment.y_delays,
                "direction": segment.y_direction,
            })
            
            self.segment_counter.count_up()
    
    def _process_status_x_axis(self, msg: dict[str, Any]) -> None:
        status = msg.get("status")
        if status == "segment_done":
            self.travel_time_x.update(msg.get("travel_time", float('nan')))
            self.logger.info(
                f"segment {self.segment_counter.value - 1}: "
                f"travel time X-axis = {self.travel_time_x.state:.3f} s"
            )
            if self.segment_counter.value <= self.num_segments:
                self.x_motor_ready.update(True)
            else:
                self.x_motor_ready.update(False)
                self.x_motor_busy.update(False)
        elif status == "startup_error":
            self.logger.error(
                f"[motor X] Startup error: "
                f"{msg.get('message', 'No message')}"
            )
            raise RuntimeError("Startup error in motor X")
    
    def _process_status_y_axis(self, msg: dict[str, Any]) -> None:
        status = msg.get("status")
        if status == "segment_done":
            self.travel_time_y.update(msg.get("travel_time", float('nan')))
            self.logger.info(
                f"segment {self.segment_counter.value - 1}: "
                f"travel time Y-axis = {self.travel_time_y.state:.3f} s"
            )
            if self.segment_counter.value <= self.num_segments:
                self.y_motor_ready.update(True)
            else:
                self.y_motor_ready.update(False)
                self.y_motor_busy.update(False)
        elif status == "startup_error":
            self.logger.error(
                f"[motor Y] Startup error: "
                f"{msg.get('message', 'No message')}"
            )
            raise RuntimeError("Startup error in motor Y")
        
    def _trajectory_finished(self):
        c1 = self.segment_counter.value > self.num_segments
        c2 = not self.x_motor_busy.active
        c3 = not self.y_motor_busy.active
        if c1 and c2 and c3:
            return True
        return False
    
    def control_routine(self):
        self._init_control()
        self._sequence_control()
        self._execute_actions()

    def exit_routine(self):
        self.logger.info("Exit. Shutting down...")
        self.x_interface.send({"cmd": "shutdown"})
        self.y_interface.send({"cmd": "shutdown"})
        self.x_motor_controller.disable()
        self.y_motor_controller.disable()
        self.logger.info("Motion controller stopped.")

    def emergency_routine(self):
        self.logger.warning("Emergency stop. Shutting down...")
        self.exit_routine()

    def crash_routine(self, exception):
        self.logger.error("PLC crashed. Shutting down...")
        self.exit_routine()
        raise exception
