from typing import Callable, Any, Iterator
import logging
from multiprocessing import Pipe
import tomllib
import json
from pathlib import Path
from abc import abstractmethod
from dataclasses import dataclass

from pyberryplc.stepper.driver.base import StepperMotor, PinConfig, TStepperMotor
from pyberryplc.stepper.driver.tmc2208 import TMC2208StepperMotor
from pyberryplc.stepper.uart.tmc2208_uart import TMC2208UART
from pyberryplc.stepper.controller.process import SPMCProcess

from pyberryplc.core import MemoryVariable, CounterUp, AbstractPLC, HMISharedData
from pyberryplc.motion import RotationDirection


class SPMotorController:
    """
    Creates and controls a separate `SPMCProcess` which has a concrete 
    `StepperMotor` instance inside. A `multiprocessing.Pipe` is used to
    send messages to and receive messages from the `SPMCProcess`. To
    send messages (commands) to the `SPMCProcess` use the `send(...)` method
    of instance attribute `com_interface`.
    """
    def __init__(
        self,
        motor_name: str,
        motor_class: type[StepperMotor],
        pin_config: PinConfig,
        cfg_callback: Callable[[TStepperMotor], None] | None = None,
        comm_port: str = "",
        logger: logging.Logger | None = None,
    ) -> None:
        """
        Creates a `SPMotorController` instance.

        Parameters
        ----------
        motor_name:
            A name to identify the motor.
        motor_class:
            Concrete class derived from base class `StepperMotor` that will be 
            instantiated inside the `SPMCProcess`.
        pin_config:
            Configuration of the GPIO pins that will be used to control the 
            stepper motor.
        cfg_callback:
            Callback function that must accept the concrete `StepperMotor` 
            instance from `SPMCProcess` and that will be called inside the 
            `SPMCProcess` to configure the stepper motor (enabling the motor, 
            configuration of microstepping, setting the maximum run and hold 
            current, ...).
        comm_port:
            In case the `TMC2208StepperMotor` class is used together with a 
            UART-interface (class `TMC2208UART`), the serial communication port 
            that needs to be used must be specified.
        logger:
            `Logger` instance to send log messages from the concrete 
            `StepperMotor` instance to a logfile and/or to the console.
        """
        self.motor_name = motor_name
        self.motor_class = motor_class
        self.pin_config = pin_config
        self.cfg_callback = cfg_callback
        self.comm_port = comm_port
        self.logger = logger
        self._motor_busy = MemoryVariable(False, False)
        self._motor_ready = MemoryVariable(False, False)
        self._travel_time = MemoryVariable(float("nan"), float("nan"))
        self.segment_counter = CounterUp()
        self.num_segments: int = 0
        self._create_spmc_process()
    
    def _create_spmc_process(self) -> None:
        """
        Creates the `SPMCProcess` that will be controlled by the 
        `SPMotorController`.
        """
        self.com_interface, _spmc_endpoint = Pipe()
        motor_kwargs: dict[str, Any] = {
            "pin_config": self.pin_config,
            "logger": self.logger,
            "name": self.motor_name,
        }
        c1 = issubclass(self.motor_class, TMC2208StepperMotor)
        c2 = self.comm_port
        if c1 and c2: motor_kwargs["uart"] = TMC2208UART(port=self.comm_port)
        self._motor_control_proc = SPMCProcess(
            conn=_spmc_endpoint,
            motor_class=self.motor_class,
            motor_kwargs=motor_kwargs,
            config_callback=self.cfg_callback,
            name=self.motor_name
        )
        
    def enable(self) -> None:
        """
        Starts the `run()` method of the `SPMCProcess`. After this function
        has been called, messages (commands) can be send to the `SPMCProcess`.
        """
        self._motor_control_proc.start()
        # Signal that the motor is ready to rotate.
        self._motor_ready.update(True)
        
    def shutdown(self) -> None:
        """
        Sends shutdown command to the `SPMPProcess`. 
        """
        self.com_interface.send({"cmd": "shutdown"})
    
    def disable(self) -> None:
        """
        Disables or closes the `SPMPProcess` by calling its `join()` method.
        """
        self._motor_control_proc.join()
    
    def _set_status(self, msg: dict[str, Any]) -> None:
        """
        This method is called when the external program requests a status
        update (by calling public method `update_status()`).
        """
        status = msg.get("status")
        
        if status == "segment_done":
            # A rotation is completed: report its travel time via the logger.
            travel_time = msg.get("travel_time", float("nan"))
            self._travel_time.update(travel_time)
            self.logger.info(
                f"[{self.motor_name}] "
                f"segment {self.segment_counter.value + 1}: "
                f"travel time {self.motor_name} = "
                f"{self._travel_time.state:.3f} s"
            )
            if self.segment_counter.value < self.num_segments - 1:
                # Signal that the motor has finished a rotation and is ready to 
                # accept a new rotation command.
                self._motor_ready.update(True)
                self.segment_counter.count_up()
            else:
                # Signal that all rotations are completed (the motor is not
                # "busy" anymore), but stays ready to accept a new rotation
                # command.
                self._motor_ready.update(True)
                self._motor_busy.update(False)
                self.segment_counter.reset()
        
        elif status == "startup_error":
            # Report that the `SPMPProcess` failed during startup.
            self.logger.error(
                f"[{self.motor_name}] Startup error: "
                f"{msg.get('message', 'No message')}"
            )
            raise RuntimeError(f"Startup error in {self.motor_name}")
    
    def update_status(self) -> None:
        """
        When this method is called, the `SPMotorController` requests a status 
        update from its internal `SPMCProcess`.
        """
        if self.com_interface.poll():
            msg = self.com_interface.recv()
            self._set_status(msg)
    
    @property
    def motor_busy(self) -> bool | int:
        """
        Indicates if the motor has completed the current trajectory, i.e. all of
        its rotations are done (returns `False`), or not (returns `True`).
        """
        return self._motor_busy.state
    
    @property
    def motor_ready(self) -> bool | int:
        """
        Indicates if the motor is ready to accept a new rotation command 
        (returns `True`) or not (returns `False`).
        """
        return self._motor_ready.state
    
    @property
    def travel_time(self) -> float:
        """
        Returns the currently stored travel time of the motor.
        """
        return self._travel_time.state
    
    def send_step_pulse_signal(
        self, 
        signal: list[float], 
        rdir: RotationDirection
    ) -> None:
        """
        Sends the step pulse signal to the `SPMCProcess` to start a new
        rotation.
        """
        # If it is the first rotation of the trajectory, turn the motor state 
        # to "busy".
        if self.segment_counter.value == 0: 
            self._motor_busy.update(True)
        
        # Indicate that the motor is not ready anymore.
        self._motor_ready.update(False)
        
        self.com_interface.send({
            "cmd": "start_segment",
            "delays": signal,
            "direction": rdir
        })


@dataclass
class MotorStatus:
    """
    Holds the status attributes of a motor (controller).
    """
    ready: bool = False
    busy: bool = False
    travel_time: float = float("nan")


@dataclass
class MotionControlStatus:
    """
    Holds the status attributes of the individual motors and maps them to global 
    status attributes of the motion control through properties.  
    """
    x: MotorStatus | None = None
    y: MotorStatus | None = None
    z: MotorStatus | None = None
    
    def __post_init__(self):
        self.motor_statuses = tuple(s for s in (self.x, self.y, self.z) if s is not None)
    
    @property
    def ready(self) -> bool:
        """
        Returns `True` when all motors are ready, else `False`.
        """
        if all([m.ready for m in self.motor_statuses]):
            return True
        return False
    
    @property
    def busy(self) -> bool:
        """
        Returns `True` when any motor is still busy, else `False`.
        """
        if any([m.busy for m in self.motor_statuses]):
            return True
        return False
    
    @property
    def travel_time(self) -> float:
        """
        Returns the maximum of the travel times of the motors.
        """
        travel_time = max([m.travel_time for m in self.motor_statuses])
        return travel_time


class XYZMotionController:
    """
    High-level class that can encapsulate three motor controllers 
    (`SPMotorController`) to control simultaneous X-axis, Y-axis, and Z-axis
    motion (3D-motion). The class uses the `TMC2208StepperMotor`. The
    configuration settings of each `TMC2208StepperMotor` are taken from a 
    TOML file.
    """
    def __init__(
        self,
        master: 'XYZMotionPLC',
        config_filepath: str = "motor_config.toml",
        logger: logging.Logger | None = None
    ) -> None:
        """
        Creates a `XYZMotionController` instance.
        
        Parameters
        ----------
        master:
            Concrete instance of `AbstractPLC`.
        config_filepath:
            File path to the TOML-file with the motor configuration settings.
        logger:
            `Logger` instance to send log messages from the motor controllers to
             a file and/or to the console.
        """
        self.master = master
        self.motor_class: type[StepperMotor] = TMC2208StepperMotor
        self.config_filepath = config_filepath
        self.logger = logger
        
        self.x_motor_cfg: dict | None = None
        self.y_motor_cfg: dict | None = None
        self.z_motor_cfg: dict | None = None
        self.x_motor_ctrl: SPMotorController | None = None
        self.y_motor_ctrl: SPMotorController | None = None
        self.z_motor_ctrl: SPMotorController | None = None
        self.motor_ctrls: tuple[SPMotorController, ...] | None = None
        self.trajectory: list[dict[str, Any]] | None = None
        self.segments: Iterator[tuple[int, dict[str, Any]]] | None = None
        
        self._load_motor_configurations()
        self._create_motor_controllers()
    
    def _load_motor_configurations(self) -> None:
        """
        Loads the motor configurations .toml-file and assigns the configuration 
        of each motor to its own configuration-dict. If a motor is missing, its
        configuration-dict will be set to `None`.
        """
        with Path(self.config_filepath).open("rb") as f:
            config = tomllib.load(f)
            self.x_motor_cfg = config.get("x_motor")
            self.y_motor_cfg = config.get("y_motor")
            self.z_motor_cfg = config.get("z_motor")

    def _create_motor_controller(self, axis_name: str, cfg: dict) -> SPMotorController:
        """
        Creates and returns a `SPMotorController` for the specified axis.
        """
        step_pin_ID = cfg.get("step_pin_ID")
        dir_pin_ID = cfg.get("dir_pin_ID")
        comm_port = cfg.get("comm_port")

        motor_controller = SPMotorController(
            motor_name=axis_name,
            motor_class=self.motor_class,
            pin_config=PinConfig(
                step_pin_ID=step_pin_ID,
                dir_pin_ID=dir_pin_ID,
                use_pigpio=True
            ),
            cfg_callback=lambda motor: self._config_motor(motor, cfg),
            comm_port=comm_port,
            logger=self.logger
        )
        return motor_controller
    
    @staticmethod
    def _config_motor(motor: TMC2208StepperMotor, cfg: dict):
        """
        This is a callback-function which is called by the `SPMProcess` inside 
        the `SPMotorController` when it is first started. The `TMC2208StepperMotor` 
        is only created when the `SPMProcess` is started. After its creation, 
        the process then calls this method to set its motor configuration.
        """
        high_sensitivity = cfg.get("high_sensitivity", True)
        resolution = cfg["microstepping"]["resolution"]
        full_steps_per_rev = cfg["microstepping"]["full_steps_per_rev"]
        run_current_pct = cfg["current"]["run_current_pct"]
        hold_current_pct = cfg["current"]["hold_current_pct"]

        motor.enable(
            high_sensitivity=high_sensitivity
        )
        motor.configure_microstepping(
            resolution=resolution,
            ms_pins=None,
            full_steps_per_rev=full_steps_per_rev
        )
        motor.set_current_via_uart(
            run_current_pct=run_current_pct,
            hold_current_pct=hold_current_pct
        )
    
    def _create_motor_controllers(self) -> None:
        """
        Creates a `SPMotorController` for each motion axis if a motor 
        configuration exists for that axis.
        """
        motor_ctrls = []
        if self.x_motor_cfg is not None:
            self.x_motor_ctrl = self._create_motor_controller(
                "X-axis", 
                self.x_motor_cfg
            )
            motor_ctrls.append(self.x_motor_ctrl)
        if self.y_motor_cfg is not None:
            self.y_motor_ctrl = self._create_motor_controller(
                "Y-axis", 
                self.y_motor_cfg
            )
            motor_ctrls.append(self.y_motor_ctrl)
        if self.z_motor_cfg is not None:
            self.z_motor_ctrl = self._create_motor_controller(
                "Z-axis", 
                self.z_motor_cfg
            )
            motor_ctrls.append(self.z_motor_ctrl)
        self.motor_ctrls = tuple(motor_ctrls)
        
    def enable(self) -> None:
        """
        Enabels the motor controllers of the configured axes. Enabling a 
        `SPMotorController` implies that the `SPMCProcess` inside the controller
        is started and that the concrete `StepperMotor` instance is created and
        configured.
        """
        for motor_ctrl in self.motor_ctrls:
            motor_ctrl.enable()
    
    def disable(self) -> None:
        """
        Disables the motor controllers of the configured axes. Disabling a
        `SPMotorController` implies that the `SPMCProcess` inside the controller
        is closed.
        """
        for motor_ctrl in self.motor_ctrls:
            motor_ctrl.shutdown()
        for motor_ctrl in self.motor_ctrls:
            motor_ctrl.disable()
    
    def get_status(self) -> MotionControlStatus:
        """
        Returns the current operating state of each individual motor and the 
        global operating state of the motion control.
        """
        def get_motor_status(motor_ctrl: SPMotorController) -> MotorStatus:
            return MotorStatus(
                ready=motor_ctrl.motor_ready,
                busy=motor_ctrl.motor_busy,
                travel_time=motor_ctrl.travel_time
            )
        
        kwargs = {}
        for motor_ctrl in self.motor_ctrls:
            motor_ctrl.update_status()
            motor_status = get_motor_status(motor_ctrl)
            if motor_ctrl.motor_name == "X-axis":
                kwargs["x"] = motor_status
            if motor_ctrl.motor_name == "Y-axis":
                kwargs["y"] = motor_status
            if motor_ctrl.motor_name == "Z-axis":
                kwargs["z"] = motor_status
        motion_control_status = MotionControlStatus(**kwargs)
        return motion_control_status
    
    def load_trajectory(self, filepath: str) -> int:
        """
        Loads a trajectory JSON file from the given filepath and returns the
        number of segments (movements) in the trajectory. 

        A trajectory JSON file is created by calling method 
        `save_stepper_driver_signals()` on a `Trajectory` object.
        """
        with open(filepath, "r") as f:
            self.trajectory = json.load(f)
        
        # Assign the number of segments in the trajectory to each motor 
        # controller.
        num_segments = len(self.trajectory)
        for motor_ctrl in self.motor_ctrls:
            motor_ctrl.num_segments = num_segments
        
        # Create a generator to iterate on command over the segments in the 
        # trajectory (see method `move()`)
        self.segments = ((i, s) for i, s in enumerate(self.trajectory))
        
        return num_segments
        
    @staticmethod
    def _get_rotation_direction(rdir: str) -> RotationDirection | None:
        match rdir:
            case "counterclockwise":
                return RotationDirection.COUNTERCLOCKWISE
            case "clockwise":
                return RotationDirection.CLOCKWISE
        return None
    
    def _send_step_pulse_signals(self, segment: dict[str, Any]) -> None:
        """
        Sends the step pulse signals (including the rotation direction) of a 
        single segment in the loaded trajectory to the motor controllers.
        """
        if self.x_motor_ctrl is not None: 
            self.x_motor_ctrl.send_step_pulse_signal(
                signal=segment["delays_x"],
                rdir=self._get_rotation_direction(segment["rdir_x"])
            )
        if self.y_motor_ctrl is not None:
            self.y_motor_ctrl.send_step_pulse_signal(
                signal=segment["delays_y"],
                rdir=self._get_rotation_direction(segment["rdir_y"])
            )
        if self.z_motor_ctrl is not None:
            self.z_motor_ctrl.send_step_pulse_signal(
                signal=segment["delays_z"],
                rdir=self._get_rotation_direction(segment["rdir_z"])
            )
    
    def move(self) -> int:
        """
        Sends the signals for the next segment axis rotations in the trajectory 
        to the motor controllers. Returns the sequence number of the segment in
        the trajectory or -1 when the trajectory is finished.
        """
        if self.trajectory:
            try:
                i, segment = next(self.segments)
            except StopIteration:
                return -1
            else:
                self._send_step_pulse_signals(segment)
                return i
        else:
            raise AttributeError("No trajectory has been loaded yet.")


class XYZMotionPLC(AbstractPLC):
    """
    Extends `AbstractPLC` by incorporating an `XYZMotionController`. The
    motion controller is accessible through attribute `self.motion_controller`.
    To read the current state of the motors controlled by the motion controller,
    access attribute `self.motion_control_status`.
    """
    def __init__(
        self, 
        hmi_data: HMISharedData | None, 
        logger: logging.Logger,
        motor_config_filepath: str,
    ) -> None:
        """
        Instantiates the PLC application.
        """
        super().__init__(hmi_data=hmi_data, logger=logger)
        self.motion_controller = XYZMotionController(self, motor_config_filepath, logger)
        self.motion_control_status: MotionControlStatus | None = None
        self._init_flag = True
    
    def _init_control(self) -> None:
        if self._init_flag:
            self._init_flag = False
            self.motion_controller.enable()
            self.init_control()
    
    @abstractmethod
    def init_control(self) -> None:
        """
        This method is executed only once the moment the PLC program steps into 
        its first scan cycle.
        """
        pass    
    
    @abstractmethod
    def sequence_control(self) -> None:
        """
        Applies the transition logic to move from one step to another 
        (or others in case of simultaneous tasks) in the PLC program sequence 
        (SFC-approach).
        """
        pass
    
    @abstractmethod
    def execute_actions(self) -> None:
        """
        Implements the actions connected to each step of the PLC sequence. 
        Only executes the actions that are connected to the currently active 
        step(s) in the PLC sequence.
        """
        pass
    
    def control_routine(self) -> None:
        # Run any initialization tasks when the PLC is launched.
        self._init_control()
        
        # Request the current operating state of the motors at the start 
        # of the sequential control routine. Returns a `MotionControlStatus`
        # object.
        self.motion_control_status = self.motion_controller.get_status()
        
        # PLC program implementation.
        self.sequence_control()
        self.execute_actions()
    
    def exit_routine(self) -> None:
        """Default exit routine which is called when the `exit()` method is 
        called or when the operator presses <Ctrl-Z>.
        
        Override this method or use `super()` to extend it. However, be aware
        that the motion controller must always be disabled.
        """
        self.logger.info("Exit. Shutting down...")
        self.motion_controller.disable()
        self.logger.info("Motion controller stopped.")

    def emergency_routine(self):
        """Default emergency routine which is called when an `EmergencyException`
        is raised from the `control_routine()`.
        
        Override this method or use `super()` to extend it. However, be aware
        that the motion controller must always be disabled.
        """
        self.logger.warning("Received emergency stop. Shutting down...")
        self.exit_routine()

    def crash_routine(self, exception):
        """Default crash routine in case an unexpected error should occur.
        
        Override this method or use `super()` to extend it. However, be aware
        that the motion controller must always be disabled.
        """
        self.logger.error("PLC program crashed. Shutting down...")
        self.exit_routine()
        raise exception
