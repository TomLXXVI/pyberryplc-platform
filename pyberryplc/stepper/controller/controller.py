from typing import Callable, Any, Iterator
import logging
from multiprocessing import Pipe
import tomllib
import json
from pathlib import Path
from dataclasses import dataclass

from pyberryplc.stepper.driver.base import StepperMotor, PinConfig, TStepperMotor
from pyberryplc.stepper.driver.tmc2208 import TMC2208StepperMotor
from pyberryplc.stepper.uart.tmc2208_uart import TMC2208UART
from pyberryplc.stepper.controller.process import SPMCProcess

from pyberryplc.motion.profile import RotationDirection, MotionProfile

from pyberryplc.core import MemoryVariable, CounterUp, TAbstractPLC


class MotorController:
    """
    Creates and controls a separate `SPMCProcess` which holds a concrete 
    `StepperMotor` instance inside it. A `multiprocessing.Pipe` is used to
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
        jog_mode_profile: MotionProfile | None = None,
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
        jog_mode_profile:
            Motion profile for acceleration/deceleration in jog mode.
        """
        self.motor_name = motor_name
        self.motor_class = motor_class
        self.pin_config = pin_config
        self.cfg_callback = cfg_callback
        self.comm_port = comm_port
        self.logger = logger
        self.jog_mode_profile = jog_mode_profile

        self._motions_finished = MemoryVariable(True)
        self._motor_ready = MemoryVariable(True)
        self._travel_time = MemoryVariable(float("nan"))
        self._jog_mode_active = MemoryVariable(False)

        self.move_counter = CounterUp()
        self.num_moves: int = 0

        self._create_process()
    
    def _create_process(self) -> None:
        """
        Creates the `SPMCProcess` that will be controlled by the 
        `SPMotorController`.
        """
        self.comm_interface, _proc_endpoint = Pipe()

        motor_kwargs: dict[str, Any] = {
            "pin_config": self.pin_config,
            "logger": self.logger,
            "name": self.motor_name,
        }

        c1 = issubclass(self.motor_class, TMC2208StepperMotor)
        c2 = self.comm_port
        if c1 and c2:
            motor_kwargs["uart"] = TMC2208UART(port=self.comm_port)

        self._process = SPMCProcess(
            conn=_proc_endpoint,
            motor_class=self.motor_class,
            motor_kwargs=motor_kwargs,
            config_callback=self.cfg_callback,
            motor_name=self.motor_name,
            jog_mode_profile=self.jog_mode_profile
        )

    def enable(self) -> None:
        """
        Starts the `run()` method of the `SPMCProcess`. After this function
        has been called, messages (commands) can be send to the `SPMCProcess`.
        """
        self._process.start()

    def shutdown(self) -> None:
        """
        Sends shutdown command to the `SPMPProcess`. 
        """
        self.comm_interface.send({"cmd": "shutdown"})
    
    def disable(self) -> None:
        """
        Disables or closes the `SPMPProcess` by calling its `join()` method.
        """
        self._process.join()
    
    def _set_status(self, msg: dict[str, Any]) -> None:
        """
        This method is called when the external program requests a status
        update (by calling public method `update_status()`).
        """
        status = msg.get("status")
        
        if status == "motion_done":
            # A motion is completed:
            self.move_counter.count_up()
            # Report its travel time through the logger:
            travel_time = msg.get("travel_time", float("nan"))
            self._travel_time.update(travel_time)
            self.logger.info(
                f"[{self.motor_name}] "
                f"movement {self.move_counter.value}: "
                f"travel time {self.motor_name} = "
                f"{self._travel_time.state:.3f} s"
            )
            if self.move_counter.value < self.num_moves:
                # Signal that the motor has finished a movement and is ready to
                # accept a new rotation command.
                self._motor_ready.update(True)
                self._motions_finished.update(False)
            else:
                # Signal that all movements are completed, but that the motor
                # stays ready to accept a new rotation command.
                self._motor_ready.update(True)
                self._motions_finished.update(True)
                self.move_counter.reset()
        
        elif status == "startup_error":
            # Report that the `SPMPProcess` failed during startup.
            self.logger.error(
                f"[{self.motor_name}] Startup error: "
                f"{msg.get('message', 'No message')}"
            )
            raise RuntimeError(f"Startup error in {self.motor_name}")

        elif status == "jog_started":
            self._jog_mode_active.update(True)
            self.logger.info(
                f"[{self.motor_name}] {msg.get('message', '')}"
            )

        elif status == "jog_stopped":
            self._jog_mode_active.update(False)
            self.logger.info(
                f"[{self.motor_name}] {msg.get('message', '')}"
            )

        elif status == "error":
            self.logger.info(
                f"[{self.motor_name}] {msg.get('message', '')}"
            )

    def update_status(self) -> None:
        """
        When this method is called, the `SPMotorController` requests a status 
        update from its internal `SPMCProcess`. Updated status info can then be
        read through properties `motions_finished`, `motor_ready`, `travel_time`,
        and `jog_mode_active`.
        """
        if self.comm_interface.poll():
            msg = self.comm_interface.recv()
            self._set_status(msg)
    
    @property
    def motions_finished(self) -> bool | int:
        """
        Indicates whether the motor has completed all its motions in the current
        trajectory.
        """
        return self._motions_finished.state
    
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

    @property
    def jog_mode_active(self) -> bool:
        """
        Indicates whether jog mode is active.
        """
        return self._jog_mode_active.state

    def move(
        self, 
        step_pulses: list[float],
        rdir: RotationDirection
    ) -> None:
        """
        Sends the step pulse signal to the `SPMCProcess` to start a new
        rotation.
        """
        # When a move-command is given, `self.motor_ready` must be set to False
        # to signal that a move-command is busy. When the move-command is
        # completed, it will be signaled by the process (see method
        # `_set_status()`) and `self.motor_ready` is then set to True again.
        if self._motor_ready.state:
            self._motor_ready.update(False)
            self.comm_interface.send({
                "cmd": "start_motion",
                "delays": step_pulses,
                "direction": rdir
            })

    def start_jog_mode(self, rdir: RotationDirection) -> None:
        """
        Turns jog mode on.
        """
        if self.motor_ready and self.motions_finished:
            self.comm_interface.send({
                "cmd": "start_jog",
                "direction": rdir
            })

    def stop_jog_mode(self) -> None:
        """
        Turns jog mode off.
        """
        if self.jog_mode_active:
            self.comm_interface.send({
                "cmd": "stop_jog"
            })

    def set_init_state_trajectory(self, num_moves: int) -> None:
        """
        Sets the initial state of the controller when a series of consecutive
        movements in a trajectory is to be started.
        """
        self.num_moves = num_moves
        self.move_counter.reset()
        # When the first move-command is given, `self_motions_finished` must be
        # reset to False and will become True again when all motor movements
        # are completed (see method `_set_status()`).
        self._motions_finished.update(False)

    def set_jog_mode_profile(self, mp: MotionProfile):
        self.jog_mode_profile = mp
        profile_args = {
            "ds_tot": mp.ds_tot,
            "a_max": mp.a_max,
            "v_max": mp.v_max,
            "v_i": mp.v_i,
            "v_f": mp.v_f
        }
        self.comm_interface.send({
            "cmd": "set_jog_mode_profile_args",
            "args": profile_args
        })


@dataclass
class MotorStatus:
    """
    Holds the status attributes of a single motor (controller).
    """
    ready: bool = True
    motions_finished: bool = True
    travel_time: float = float("nan")
    jog_mode_active: bool = False


@dataclass
class MotionStatus:
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
    def all_ready(self) -> bool:
        """
        Returns `True` when all motors are ready, else `False`.
        """
        if all([m.ready for m in self.motor_statuses]):
            return True
        return False
    
    @property
    def all_finished(self) -> bool:
        """
        Returns `True` when all motor movements are finished, else `False`.
        """
        if all([m.motions_finished for m in self.motor_statuses]):
            return True
        return False
    
    @property
    def travel_time(self) -> float:
        """
        Returns the maximum of the travel times of the motors.
        """
        travel_time = max([m.travel_time for m in self.motor_statuses])
        return travel_time

    @property
    def jog_mode_active(self) -> bool:
        """
        Returns `True` when any motor is in jog mode, else `False`.
        """
        if any([m.jog_mode_active for m in self.motor_statuses]):
            return True
        return False


class XYZMotionController:
    """
    High-level class that can encapsulate three motor controllers 
    (`SPMotorController`) to simultaneousely control X-axis, Y-axis, and Z-axis
    motion (3D-motion). This class uses the `TMC2208StepperMotor` with UART. The
    configuration settings for each `TMC2208StepperMotor` are taken from a 
    TOML file.
    """
    def __init__(
        self,
        master: TAbstractPLC,
        config_filepath: str = "motor_config.toml",
        logger: logging.Logger | None = None,
        jog_mode_profile: MotionProfile | None = None
    ) -> None:
        """
        Creates a `XYZMotionController` instance.
        
        Parameters
        ----------
        master:
            Concrete instance of `XYZMotionPLC` that incorporates this 
            `XYZMotionController` instance.
        config_filepath:
            File path to the TOML-file with the motor configuration settings.
        logger:
            `Logger` instance to send log messages from the motor controllers to
             a file and/or to the console.
        jog_mode_profile:
            Motion profile for acceleration/deceleration in jog mode.
        """
        self.master = master
        self.motor_class: type[StepperMotor] = TMC2208StepperMotor
        self.config_filepath = config_filepath
        self.logger = logger
        self.jog_mode_profile = jog_mode_profile
        
        self.x_motor_cfg: dict | None = None
        self.y_motor_cfg: dict | None = None
        self.z_motor_cfg: dict | None = None

        self.x_motor_ctrl: MotorController | None = None
        self.y_motor_ctrl: MotorController | None = None
        self.z_motor_ctrl: MotorController | None = None
        self.motor_ctrls: tuple[MotorController, ...] | None = None

        TSegment = dict[str, tuple[list[float], RotationDirection]]
        TTrajectory = list[TSegment]
        self.trajectory: TTrajectory | None = None
        self.segments: Iterator[tuple[int, TSegment]] | None = None
        
        self._load_motor_configurations()
        self._create_motor_controllers()
    
    def _load_motor_configurations(self) -> None:
        """
        Loads the motor configuration TOML file and assigns the configuration 
        of each motor to its own configuration-dict. If a motor is missing, its
        configuration-dict will be set to `None`.
        """
        with Path(self.config_filepath).open("rb") as f:
            config = tomllib.load(f)
            self.x_motor_cfg = config.get("x_motor")
            self.y_motor_cfg = config.get("y_motor")
            self.z_motor_cfg = config.get("z_motor")

    def _create_motor_controller(
        self,
        axis_name: str,
        cfg: dict
    ) -> MotorController:
        """
        Creates and returns a `SPMotorController` for the specified axis.
        """
        step_pin_ID = cfg.get("step_pin_ID")
        dir_pin_ID = cfg.get("dir_pin_ID")
        comm_port = cfg.get("comm_port")

        motor_controller = MotorController(
            motor_name=axis_name,
            motor_class=self.motor_class,
            pin_config=PinConfig(
                step_pin=step_pin_ID,
                dir_pin=dir_pin_ID,
                use_pigpio=True
            ),
            cfg_callback=lambda motor: self._config_motor(motor, cfg),
            comm_port=comm_port,
            logger=self.logger,
            jog_mode_profile=self.jog_mode_profile,
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
        high_sensitivity = cfg.get("high_sensitivity", False)
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
    
    def get_motion_status(self) -> MotionStatus:
        """
        Returns the current operating state of each individual motor and the 
        global operating state of the motion control in a `MotionStatus` object.
        """
        def get_motor_status(motor_ctrl: MotorController) -> MotorStatus:
            return MotorStatus(
                ready=motor_ctrl.motor_ready,
                motions_finished=motor_ctrl.motions_finished,
                travel_time=motor_ctrl.travel_time,
                jog_mode_active=motor_ctrl.jog_mode_active
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
        return MotionStatus(**kwargs)
    
    def load_trajectory(
        self,
        filepath: str | None,
        trajectory: list | None = None
    ) -> int:
        """
        If `filepath` is not None, loads a trajectory JSON file from the given
        filepath. If a trajectory is directly passed, set `file_path` to None,
        and pass the trajectory through parameter `trajectory`.

        Note: A trajectory JSON file can be created by calling method
        `save_stepper_signals()` on a `PointToPointTrajectory` object.

        Returns
        -------
        int :
            The number of segments in the trajectory. If the trajectory is
            invalid, -1 is returned.
        """
        if filepath is not None:
            with open(filepath, "r") as f:
                self.trajectory = json.load(f)
        else:
            self.trajectory = trajectory

        if self.trajectory:
            # Assign the number of segments in the trajectory to each axis motor
            # controller if this is needed.
            num_segments = len(self.trajectory)
            segment1 = self.trajectory[0]
            if "x" in segment1.keys():
                self.x_motor_ctrl.set_init_state_trajectory(num_segments)
            if "y" in segment1.keys():
                self.y_motor_ctrl.set_init_state_trajectory(num_segments)
            if "z" in segment1.keys():
                self.z_motor_ctrl.set_init_state_trajectory(num_segments)

            # Create a generator to iterate on command over the segments in the
            # trajectory (see method `move()`)
            self.segments = ((i, s) for i, s in enumerate(self.trajectory))

            return num_segments
        return -1

    @staticmethod
    def get_rotation_direction(rdir: str) -> RotationDirection | None:
        match rdir:
            case "counterclockwise" | "ccw":
                return RotationDirection.CCW
            case "clockwise" | "cw":
                return RotationDirection.CW
        return None
    
    def _send_stepper_signals(self, segment: dict[str, tuple]) -> None:
        """
        Sends the step pulse signals (including the rotation direction) of a 
        single segment in the loaded trajectory to the motor controllers.
        """
        if self.x_motor_ctrl is not None and "x" in segment.keys():
            self.x_motor_ctrl.move(
                step_pulses=segment["x"][0],
                rdir=self.get_rotation_direction(segment["x"][1])
            )

        if self.y_motor_ctrl is not None and "y" in segment.keys():
            self.y_motor_ctrl.move(
                step_pulses=segment["y"][0],
                rdir=self.get_rotation_direction(segment["y"][1])
            )

        if self.z_motor_ctrl is not None and "z" in segment.keys():
            self.z_motor_ctrl.move(
                step_pulses=segment["z"][0],
                rdir=self.get_rotation_direction(segment["z"][1])
            )
    
    def move(self) -> int:
        """
        Sends the step pulse signals for the next segment axis rotations in the
        trajectory to the motor controllers. Returns the sequence number of the
        segment in the trajectory or -1 when the trajectory is finished.
        """
        if self.trajectory:
            try:
                i, segment = next(self.segments)
            except StopIteration:
                return -1
            else:
                self._send_stepper_signals(segment)
                return i
        else:
            raise AttributeError("No trajectory has been loaded yet.")

    def start_jog_mode(
        self,
        axis: str,
        rdir: RotationDirection = RotationDirection.CCW
    ) -> None:
        """
        Turns jog mode on for the specified axis.

        Jog mode can only be started when all motors are at rest and the
        execution of a trajectory has been finished.

        Parameters
        ----------
        axis: str, {"x", "y", "z"}
            Specifies the motion axis.
        rdir:
            Rotation direction in jog mode.

        Returns
        -------
        None
        """
        motion = self.get_motion_status()
        if motion.all_ready and motion.all_finished:
            if axis == "x" and self.x_motor_ctrl is not None:
                motor_ctrl = self.x_motor_ctrl
            elif axis == "y" and self.y_motor_ctrl is not None:
                motor_ctrl = self.y_motor_ctrl
            elif axis == "z" and self.z_motor_ctrl is not None:
                motor_ctrl = self.z_motor_ctrl
            else:
                raise ValueError(f"Axis {axis} is undefined.")
            motor_ctrl.start_jog_mode(rdir)
        else:
            self.logger.warning("Jog mode cannot be started.")

    def stop_jog_mode(self, axis: str) -> None:
        """
        Turns jog mode off.

        Parameters
        ----------
        axis: str, {"x", "y", "z"}
            Specifies the motion axis.

        Returns
        -------
        None
        """
        motion_status = self.get_motion_status()
        if motion_status.jog_mode_active:
            if axis == "x" and self.x_motor_ctrl is not None:
                motor_ctrl = self.x_motor_ctrl
            elif axis == "y" and self.y_motor_ctrl is not None:
                motor_ctrl = self.y_motor_ctrl
            elif axis == "z" and self.z_motor_ctrl is not None:
                motor_ctrl = self.z_motor_ctrl
            else:
                raise ValueError(f"Axis {axis} is undefined.")
            motor_ctrl.stop_jog_mode()

    def set_jog_mode_profile(self, mp: MotionProfile):
        for motor_ctrl in self.motor_ctrls:
            motor_ctrl.set_jog_mode_profile(mp)
