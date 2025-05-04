"""
Stepper Motor Base Class
=========================

This module defines an abstract base class `StepperMotor` for stepper motor 
control via GPIO pins, with support for both blocking and non-blocking motion 
execution. It provides a consistent interface for various motion modes, 
including:

1. **Fixed Speed Rotation**
   - Rotate a given angle at constant angular speed.
   - Available in both blocking (`rotate_fixed`) and non-blocking 
     (`start_rotation_fixed`) modes.

2. **Static Motion Profile Rotation**
   - Rotate using a predefined motion profile (e.g., trapezoidal, S-curve).
   - Delays between steps are precomputed.
   - Available in both blocking (`rotate_profile`) and non-blocking 
     (`start_rotation_profile`) modes.

3. **Dynamic Motion Profile Rotation**
   - Real-time step control using a `DynamicDelayGenerator`.
   - Supports live transitions (acceleration → cruising → deceleration) based on
     external triggers.
   - Available in blocking (`rotate_dynamic`) and non-blocking 
     (`start_rotation_dynamic`) modes.

The base class also handles:
- Microstepping configuration
- Step pulse generation (including pulse width)
- GPIO-based direction and enable control
- Step timing using monotonic timestamps

This class is intended to be subclassed by concrete driver implementations
(e.g., A4988, TMC2208) that provide hardware-specific microstepping logic.
"""

import time
import logging
from abc import ABC, abstractmethod
from collections import deque
import threading

from pyberryplc.core.gpio import DigitalOutput
from pyberryplc.motion_profiles import MotionProfile, DynamicDelayGenerator


class StepperMotor(ABC):
    """
    Abstract base class for a stepper motor controlled via GPIO, with 
    non-blocking or blocking motion control.

    This class provides the common foundation for implementing stepper motor 
    drivers.
    """
    MICROSTEP_FACTORS: dict[str, int] = {
        "full": 1,
        "1/2": 2,
        "1/4": 4,
        "1/8": 8,
        "1/16": 16,
        "1/32": 32,
        "1/64": 64,
        "1/128": 128,
        "1/256": 256
    }

    class _StepperThread(threading.Thread):
        """Control non-blocking rotation movement in a separate thread 
        independent of the PLC scan time.
        """
        def __init__(self, motor: "StepperMotor"):
            super().__init__(daemon=True)
            self.motor = motor
            self._stop_flag = threading.Event()

        def run(self):
            while not self._stop_flag.is_set():
                self.motor._do_single_step()
                time.sleep(0.001)

        def stop(self):
            self._stop_flag.set()

    def __init__(
        self,
        step_pin: int,
        dir_pin: int,
        enable_pin: int | None = None,
        full_steps_per_rev: int = 200,
        microstep_resolution: str = "full",
        logger: logging.Logger | None = None
    ) -> None:
        """
        Initialize the StepperMotor instance.

        Parameters
        ----------
        step_pin : int
            GPIO pin connected to the STEP input of the driver.
        dir_pin : int
            GPIO pin connected to the DIR input of the driver.
        enable_pin : int | None, optional
            GPIO pin connected to the EN input of the driver (active low).
        full_steps_per_rev : int, optional
            Number of full steps per revolution (i.e. at full step mode).
            Default is 200. However, this is a characteristic of the actual
            stepper motor.
        microstep_resolution : str, optional
            The microstep resolution to be used. Default is full-step mode.
            Valid microstep resolutions are defined in class attribute 
            MICROSTEP_FACTORS. However, it is possible that the actual driver
            does not support all of these. This should be checked in advance. 
        logger : logging.Logger | None, optional
            Logger for debug output.
        """
        self.step = DigitalOutput(step_pin, label="STEP", active_high=True)
        self.dir = DigitalOutput(dir_pin, label="DIR", active_high=True)
        self._enable = (
            DigitalOutput(enable_pin, label="EN", active_high=False)
            if enable_pin is not None
            else None
        )
        self.full_steps_per_rev = full_steps_per_rev
        res = self._validate_microstepping(microstep_resolution)
        self.microstep_resolution = res[0]
        self.microstep_factor = res[1]
        self.logger = logger or logging.getLogger(__name__)
        self.step_width = 10e-6  # time duration (sec) of single step pulse

        # State for non-blocking motion control
        self._busy = False
        self._next_step_time = 0.0
        self._delays = deque()
        
        self._dynamic_generator: DynamicDelayGenerator | None = None
        self._rotation_thread = None

    def enable(self) -> None:
        """Enable the stepper driver (if EN pin is defined)."""
        if self._enable:
            self._enable.write(True)
            self.logger.debug("Driver enabled")

    def disable(self) -> None:
        """Disable the stepper driver (if EN pin is defined)."""
        if self._enable:
            self._enable.write(False)
            self.logger.debug("Driver disabled")

    @property
    def busy(self) -> bool:
        """Return whether the motor is currently executing a motion."""
        return self._busy

    @abstractmethod
    def _validate_microstepping(self, microstep_resolution: str) -> tuple[str, int]:
        """
        Check whether the microstep resolution is valid for the driver. 

        Returns
        -------
        microstep_resolution : str
            The microstep resolution if valid.
        microstep_factor: int
            The microstep factor used for calculating the steps per degree and 
            the step angle.

        Raises
        ------
        ValueError :
            If the microstep resolution is unavailable on the driver.
        """
        pass

    @property
    def steps_per_degree(self) -> float:
        """Return the number of steps per degree of rotation."""
        return self.full_steps_per_rev * self.microstep_factor / 360

    @property
    def step_angle(self) -> float:
        """Return the rotation angle in degrees that corresponds with a single 
        step pulse.
        """
        return 1 / self.steps_per_degree

    @abstractmethod
    def set_microstepping(self) -> None:
        """
        Configure microstepping on the driver.
        """
        pass

    def rotate_fixed(
        self, 
        angle: float, 
        angular_speed: float, 
        direction: str = "forward"
    ) -> None:
        """Rotate the motor by a fixed angle at a constant angular speed.

        Parameters
        ----------
        angle : float
            Target rotation angle in degrees.
        angular_speed : float
            Constant speed in degrees per second.
        direction : str, optional
            Either "forward" or "backward". Default is "forward".
        """
        self._set_direction(direction)
        total_steps = int(angle * self.steps_per_degree)
        delay = 1.0 / (angular_speed * self.steps_per_degree) - self.step_width

        for _ in range(total_steps):
            self._pulse_step_pin()
            time.sleep(delay)

    def rotate_profile(
        self, 
        profile: MotionProfile, 
        direction: str = "forward"
    ) -> None:
        """Rotate the motor according to a static motion profile.

        Parameters
        ----------
        profile : MotionProfile
            Motion profile object that defines acceleration, cruising, and 
            deceleration phases.
        direction : str, optional
            Either "forward" or "backward". Default is "forward".
        """
        self._set_direction(direction)
        start_angle = 0.0
        final_angle = profile.ds_tot + self.step_angle
        angles = [
            start_angle + i * self.step_angle 
            for i in range(int(final_angle / self.step_angle))
        ]
        times = list(map(profile.get_fn_time_from_position(), angles))
        delays = [t2 - t1 - self.step_width for t1, t2 in zip(times, times[1:])]

        for delay in delays:
            self._pulse_step_pin()
            time.sleep(delay)

    def start_rotation_fixed(
        self, 
        angle: float, 
        angular_speed: float, 
        direction: str = "forward"
    ) -> None:
        """Start a non-blocking fixed-angle rotation at constant speed.
        
        When this method is called, property `busy` of the `StepperMotor` object
        is set to `True` indicating that the motor is rotating.

        Parameters
        ----------
        angle : float
            Rotation angle in degrees.
        angular_speed : float
            Constant angular speed in degrees per second.
        direction : str, optional
            Either "forward" or "backward". Default is "forward".
        """
        self._set_direction(direction)
        total_steps = int(angle * self.steps_per_degree)
        delay = 1.0 / (angular_speed * self.steps_per_degree) - self.step_width
        self._delays = deque([delay] * total_steps)
        self._busy = True
        self._next_step_time = time.time()
        self._start_thread()
        
    def start_rotation_profile(
        self, 
        profile: MotionProfile, 
        direction: str = "forward"
    ) -> None:
        """Start a non-blocking rotation using a static motion profile.
        
        When this method is called, property `busy` of the `StepperMotor` object
        is set to `True` indicating that the motor is rotating.
        
        Parameters
        ----------
        profile : MotionProfile
            Static motion profile object.
        direction : str, optional
            Either "forward" or "backward". Default is "forward".
        """
        self._set_direction(direction)
        start_angle = 0.0
        final_angle = profile.ds_tot + self.step_angle
        angles = [
            start_angle + i * self.step_angle 
            for i in range(int(final_angle / self.step_angle))
        ]
        times = list(map(profile.get_fn_time_from_position(), angles))
        delays = [t2 - t1 - self.step_width for t1, t2 in zip(times, times[1:])]
        self._delays = deque(delays)
        self._busy = True
        self._next_step_time = time.time()
        self._start_thread()

    def start_rotation_dynamic(
        self, 
        profile: MotionProfile, 
        direction: str = "forward"
    ) -> None:
        """Start a non-blocking rotation using a dynamic motion profile (the
        stop time of the rotation is not known in advance).
        
        When this method is called, property `busy` of the `StepperMotor` object
        is set to `True` indicating the motor is rotating.
        
        To stop the motor, call method `stop_rotation_dynamic()`.
        
        Parameters
        ----------
        profile : MotionProfile
            Motion profile from which the acceleration and deceleration phase
            of the motor will be determined.
        direction : str, optional
            Either "forward" or "backward". Default is "forward".
        """
        self._set_direction(direction)
        self._dynamic_generator = DynamicDelayGenerator(self.step_angle, profile)
        self._delays = None
        self._busy = True
        self._next_step_time = time.time()
        self._start_thread()
    
    def stop_rotation_dynamic(self) -> None:
        if self._dynamic_generator is not None:
            self._dynamic_generator.trigger_decel()
    
    def _start_thread(self):
        """Start the non-blocking rotation running inside a separate thread."""
        if self._rotation_thread and self._rotation_thread.is_alive():
            return
        self._rotation_thread = self._StepperThread(self)
        self._rotation_thread.start()
    
    def _do_single_step(self) -> None:
        """Perform one step of a non-blocking motion if timing is right.
        
        This method is called from inside the running rotation thread.
        When the rotation is finished, property `busy` of the `StepperMotor` 
        object will be set to `False` and the rotation thread will be stopped.
        """
        if self._dynamic_generator is not None:
            now = time.time()
            if now >= self._next_step_time:
                try:
                    self._pulse_step_pin()
                    delay = self._dynamic_generator.next_delay() - self.step_width
                    self._next_step_time = now + delay
                except StopIteration:
                    self.logger.info("Motion complete.")
                    self._busy = False
                    self._dynamic_generator = None
        else:
            now = time.time()
            if now >= self._next_step_time and self._delays:
                self._pulse_step_pin()
                self._next_step_time = now + self._delays.popleft()
                if not self._delays:
                    self.logger.info("Motion complete.")
                    self._busy = False
        if not self._busy:
            if self._rotation_thread:
                self._rotation_thread.stop()
                self._rotation_thread = None
        
    def _pulse_step_pin(self) -> None:
        """Generate a single pulse on the STEP pin."""
        self.step.write(True)
        time.sleep(self.step_width)
        self.step.write(False)

    def _set_direction(self, direction: str) -> None:
        """Set the motor direction pin.

        Parameters
        ----------
        direction : str
            Either "forward" or "backward".
        """
        if direction not in ("forward", "backward"):
            raise ValueError("Direction must be 'forward' or 'backward'")
        self.dir.write(direction == "forward")
