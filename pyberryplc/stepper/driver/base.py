import typing
from abc import ABC, abstractmethod
from enum import StrEnum
from dataclasses import dataclass
import time
import logging
import threading
from collections import deque

from pyberryplc.core import DigitalOutput, DigitalOutputPigpio
from pyberryplc.stepper.driver.dynamic_generator import DynamicDelayGenerator

from pyberryplc.motion.multi_axis import MotionProfile, RotationDirection


class Rotator(ABC):
    """
    Abstract base class for all stepper motor rotation strategies.
    
    A `Rotator` defines how the motor is commanded to move, e.g. fixed speed,
    acceleration profile, or dynamically adjusted trajectory.
    """
    def __init__(self, motor: 'StepperMotor') -> None:
        """
        Creates a `Rotator` object.
        
        Parameters
        ----------
        motor:
            The `StepperMotor` object to which the rotator is attached.
        """
        self.motor = motor
        self._step_width = 20e-6  # default step pulse width in seconds
        self._direction: RotationDirection | None = None
        self._busy = False
        self._start_time: float = 0.0
        self._end_time: float = 0.0
    
    @property
    def direction(self) -> RotationDirection:
        """
        Returns the current rotation direction setting.
        """
        return self._direction

    @direction.setter
    def direction(self, value: RotationDirection) -> None:
        """
        Sets the rotation direction of the stepper motor:
        - either `RotationDirection.CLOCKWISE`, or 
        - `RotationDirection.COUNTERCLOCKWISE`.
        """
        self._direction = value

    @property
    def busy(self) -> bool:
        """
        Returns whether the motor is currently executing a motion.
        """
        return self._busy
    
    @property
    def moving_time(self) -> float:
        """
        Returns the time duration of the rotation.
        """
        return self._end_time - self._start_time
    
    def _write_direction(self) -> None:
        """
        Applies the current rotation direction setting to the DIR pin of the 
        motor.
        """
        self.motor.dir.write(self._direction.to_bool())
    
    def _pulse_step_pin(self) -> None:
        """
        Sends a single step pulse to the STEP pin.
        The pulse duration is controlled by `self.step_width`.
        """
        if self.motor.step:
            self.motor.step.write(True)
            time.sleep(self._step_width)
            self.motor.step.write(False)


class BlockingRotator(Rotator):
    """
    Abstract base class for blocking rotation.
    "Blocking" means that the execution of the PLC scan cycle is blocked while 
    the rotation command is being executed.  
    """
    @abstractmethod
    def rotate(self) -> None:
        """
        Executes a blocking rotation.
        Must be implemented by all concrete `BlockingRotator` subclasses.
        """
        pass


class NonBlockingRotator(Rotator):
    """
    Abstract base class for non-blocking rotation.
    "Non-blocking" means that the execution of the PLC scan cycle will continue
    while the rotation command is being executed.
    """
    def __init__(self, motor: 'StepperMotor'):
        super().__init__(motor)
    
    @abstractmethod
    def start(self) -> None:
        """
        Starts a non-blocking rotation.
        Must be implemented by all concrete `NonBlockingRotator` subclasses.
        """
        pass


class FixedRotator(BlockingRotator):
    """
    A simple rotator that performs a blocking, constant-speed rotation
    over a specified angle.

    The speed and angle must be explicitly set before invoking `rotate()`.
    """
    def __init__(
        self, 
        motor: 'StepperMotor',
    ) -> None:
        super().__init__(motor)
        self._angle: float = 0.0
        self._omega: float = 0.0
        self._delays: list[float] = []

    @property
    def angle(self) -> float:
        """
        Returns the current rotation angle setting.
        """
        return self._angle

    @angle.setter
    def angle(self, value: float) -> None:
        """
        Sets the rotation angle.
        """
        if value < 0:
            raise ValueError("Angle must be non-negative.")
        self._angle = value

    @property
    def omega(self) -> float:
        """
        Returns the current angular speed setting.
        """
        return self._omega

    @omega.setter
    def omega(self, value: float) -> None:
        """
        Sets the angular speed.
        """
        if value <= 0:
            raise ValueError("Angular speed must be positive.")
        self._omega = value

    def _generate_delays(self) -> None:
        """
        Creates a list with the time delays between successive step pulses. 
        """
        steps = int(self._angle * self.motor.steps_per_degree)
        delay = 1.0 / (self._omega * self.motor.steps_per_degree) - self._step_width
        self._delays = [delay] * steps
        
    def rotate(self) -> None:
        """
        Executes the rotation by sending timed step pulse signals to the stepper
        motor driver.
        """
        if self._angle is None or self._omega is None:
            raise ValueError("Both angle and speed must be set first.")
        self._busy = True
        self._write_direction()
        self._generate_delays()
        self._start_time = time.perf_counter()
        t_ref = self._start_time
        for delay in self._delays:
            t_ref += delay
            self._pulse_step_pin()
            while time.perf_counter() < t_ref:
                pass
        self._end_time = time.perf_counter()
        self._busy = False


class FixedRotatorThreaded(NonBlockingRotator):
    """
    Executes a non-blocking rotation in a background thread using a constant 
    angular speed over a given angle.
    """
    def __init__(self, motor: 'StepperMotor') -> None:
        super().__init__(motor)
        self._angle: float | None = None
        self._omega: float | None = None
        self._thread: threading.Thread | None = None
        self._queue: deque[float] = deque()
 
    @property
    def angle(self) -> float:
        """
        Returns the current rotation angle setting.
        """
        return self._angle

    @angle.setter
    def angle(self, value: float) -> None:
        """
        Sets the rotation angle.
        """
        if value < 0:
            raise ValueError("Angle must be non-negative.")
        self._angle = value

    @property
    def omega(self) -> float:
        """
        Returns the current angular speed setting.
        """
        return self._omega

    @omega.setter
    def omega(self, value: float) -> None:
        """
        Sets the angular speed.
        """
        if value <= 0:
            raise ValueError("Angular speed must be positive.")
        self._omega = value
    
    def _generate_delays(self) -> None:
        """
        Creates a queue with the time delays between successive step pulses. 
        """
        steps = int(self._angle * self.motor.steps_per_degree)
        delay = 1.0 / (self._omega * self.motor.steps_per_degree) - self._step_width
        self._queue = deque([delay] * steps)

    def _step_loop(self) -> None:
        """
        Executes the rotation by sending timed step pulse signals to the stepper
        motor driver in a while-loop until the queue is exhausted.
        """
        self._busy = True
        self._write_direction()
        self._start_time = time.perf_counter()
        t_ref = self._start_time
        while self._queue:
            t_ref += self._queue.popleft()
            self._pulse_step_pin()
            while time.perf_counter() < t_ref:
                pass
        self._end_time = time.perf_counter()
        self._busy = False

    def start(self) -> None:
        """
        Starts the rotation in a background thread.
        """
        if self._angle is None or self._omega is None:
            raise ValueError("Angle and speed must be set before starting.")
        self._generate_delays()
        self._thread = threading.Thread(target=self._step_loop, daemon=True)
        self._thread.start()


class MotionProfileRotator(Rotator):
    """
    Executes a blocking rotation based on a motion profile.
    """
    def __init__(self, motor: 'StepperMotor') -> None:
        super().__init__(motor)
        self._motion_profile: MotionProfile | None = None
        self._delays: list[float] = []

    @property
    def profile(self) -> MotionProfile:
        """
        Returns the current motion profile setting.
        """
        return self._motion_profile

    @profile.setter
    def profile(self, value: MotionProfile) -> None:
        """
        Sets the motion profile for the rotation to be excuted.
        """
        self._motion_profile = value
    
    def _generate_delays(self) -> None:
        """
        Creates a list with the time delays between successive step pulses. 
        """
        if self._motion_profile.ds_tot > 0.0:
            step_angle = self.motor.step_angle
            final_angle = self._motion_profile.ds_tot + step_angle
            num_steps = int(final_angle / step_angle)
            try:
                angles = [self._motion_profile.s_i + i * step_angle for i in range(num_steps + 1)]
            except AttributeError:
                angles = [i * step_angle for i in range(num_steps + 1)]
            times = list(map(self._motion_profile.get_time_position_fn(), angles))
            delays = [max(0.0, t2 - t1 - self._step_width) for t1, t2 in zip(times[:-1], times[1:])]
            self._delays = delays
        else:
            self._delays = []
    
    def _step_loop(self) -> None:
        """
        Executes the rotation by sending timed step pulse signals to the stepper
        motor driver in a for-loop.
        """
        self._busy = True
        self._write_direction()
        self._start_time = time.perf_counter()
        t_ref = self._start_time
        for delay in self._delays:
            t_ref += delay
            if delay > 0.0:
                self._pulse_step_pin()
            while time.perf_counter() < t_ref:
                pass
        self._end_time = time.perf_counter()
        self._busy = False
    
    def rotate(self) -> None:
        """
        Starts the step-pulse loop and returns when the loop is finished.
        """
        if self._motion_profile is None:
            raise ValueError("Motion profile must be set first.")
        self._generate_delays()
        self._step_loop()


class MotionProfileRotatorThreaded(NonBlockingRotator):
    """
    Executes a non-blocking rotation in a background thread based on a 
    predefined motion profile.
    """
    def __init__(self, motor: 'StepperMotor') -> None:
        super().__init__(motor)
        self._motion_profile: MotionProfile | None = None
        self._thread: threading.Thread | None = None
        self._queue: deque[float] = deque()

    @property
    def profile(self) -> MotionProfile:
        """
        Returns the current motion profile setting.
        """
        return self._motion_profile

    @profile.setter
    def profile(self, value: MotionProfile) -> None:
        """
        Sets the motion profile for the rotation to be excuted.
        """
        self._motion_profile = value

    def _generate_delays(self) -> None:
        """
        Creates a queue with the time delays between successive step pulses. 
        """
        if self._motion_profile.ds_tot > 0.0:
            step_angle = self.motor.step_angle
            final_angle = self._motion_profile.ds_tot + step_angle
            num_steps = int(final_angle / step_angle)
            try:
                angles = [self._motion_profile.s_i + i * step_angle for i in range(num_steps + 1)]
            except AttributeError:
                angles = [i * step_angle for i in range(num_steps + 1)]
            times = list(map(self._motion_profile.get_time_position_fn(), angles))
            delays = [max(0.0, t2 - t1 - self._step_width) for t1, t2 in zip(times[:-1], times[1:])]
            self._queue = deque(delays)
        else:
            self._queue = deque()

    def _step_loop(self) -> None:
        """
        Executes the rotation by sending timed step pulse signals to the stepper
        motor driver in a while-loop until the queue is exhausted.
        """
        self._busy = True
        self._write_direction()
        self._start_time = time.perf_counter()
        t_ref = self._start_time
        while self._queue:
            t_ref += self._queue.popleft()
            self._pulse_step_pin()
            while time.perf_counter() < t_ref:
                pass
        self._end_time = time.perf_counter()
        self._busy = False

    def start(self) -> None:
        """
        Starts the rotation in a background thread.
        """
        if self._motion_profile is None:
            raise ValueError("Motion profile must be set before starting.")
        self._generate_delays()
        self._thread = threading.Thread(target=self._step_loop, daemon=True)
        self._thread.start()


class TwoStageMotionProfileRotator(MotionProfileRotator):
    """
    Refactored `ProfileRotator` class that splits the processing of a motion 
    profile and the commanding of a rotation movement into two separate 
    functions.
    """
    def preprocess(
        self,
        direction: RotationDirection,
        profile: MotionProfile
    ) -> None:
        """
        Calculates the delays between successive step pulses that drive the 
        stepper motor from the given motion profile..
        """
        self.direction = direction
        self.profile = profile
        self._generate_delays()

    def rotate(self) -> None:
        """
        Commands the rotation of the stepper motor.
        """
        self._step_loop()

    @property
    def delays(self) -> list[float]:
        """
        Returns the delays between successive step pulses after internal
        preprocessing of the motion profile.
        """
        return self._delays

    @delays.setter
    def delays(self, value: list[float]) -> None:
        """
        Sets the delays between successive step pulses after external 
        preprocessing of the motion profile.
        """
        self._delays = value


class DynamicRotatorThreaded(NonBlockingRotator):
    """
    Executes a non-blocking rotation of which the running time is unknown in
    advance. Acceleration and deceleration behavior can be controlled by setting
    a motion profile. The stepper motor can be started or stopped at any time.
    """
    def __init__(self, motor: 'StepperMotor') -> None:
        super().__init__(motor)
        self._motion_profile: MotionProfile | None = None
        self._generator: DynamicDelayGenerator | None = None
        self._thread: threading.Thread | None = None
        self._next_step_time = 0.0

    @property
    def profile(self) -> MotionProfile:
        """
        Returns the current motion profile setting.
        """
        return self._motion_profile

    @profile.setter
    def profile(self, value: MotionProfile) -> None:
        """
        Sets the motion profile for controlling the acceleration and 
        deceleration behavior of the motion.
        """
        self._motion_profile = value
    
    def _step_loop(self):
        """
        Executes the rotation by sending timed step pulse signals to the stepper
        motor driver in a while-loop. The continuation of the while-loop is
        controlled internally by a `DynamicDelayGenerator` object (see
        dynamic_generator.py).
        """
        self._start_time = time.perf_counter()
        while True:
            now = time.perf_counter()
            if now >= self._next_step_time:
                try:
                    self._pulse_step_pin()
                    delay = self._generator.next_delay()
                    self._next_step_time = now + delay
                except StopIteration:
                    self._busy = False
                    self._generator: DynamicDelayGenerator | None = None
                    break
            time.sleep(0.0005)
        self._end_time = time.perf_counter()
    
    def start(self) -> None:
        """
        Starts the rotation in a background thread.
        """
        self._busy = True
        self._write_direction()
        self._generator = DynamicDelayGenerator(self.motor.step_angle, self._motion_profile)
        self._next_step_time = time.perf_counter()
        self._thread = threading.Thread(target=self._step_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """
        Sends the signal to stop the motor.
        """
        self._generator.trigger_decel()
        

class RotatorType(StrEnum):
    FIXED = "fixed"
    FIXED_THREADED = "fixed_threaded"
    MOTION_PROFILE = "motion_profile"
    MOTION_PROFILE_THREADED = "profile_threaded"
    DYNAMIC_THREADED = "dynamic_threaded"
    TWO_STAGE_MOTION_PROFILE = "two_stage_motion_profile"


TRotator = typing.TypeVar("TRotator", bound=Rotator)


@dataclass
class PinConfig:
    """
    Configuration for essential GPIO pins of the stepper motor driver.

    Attributes
    ----------
    step_pin_ID : int
        GPIO pin number for STEP signal.
    dir_pin_ID : int
        GPIO pin number for DIR signal.
    en_pin_ID : int | None, optional
        GPIO pin number for ENABLE signal (active-low). Default is None.
    """
    step_pin_ID: int
    dir_pin_ID: int
    en_pin_ID: int | None = None
    use_pigpio: bool = False
    
    @property
    def step(self) -> DigitalOutput | DigitalOutputPigpio:
        if self.use_pigpio:
            return DigitalOutputPigpio(self.step_pin_ID, "STEP")
        else:
            return DigitalOutput(self.step_pin_ID, "STEP")
    
    @property
    def dir(self) -> DigitalOutput | DigitalOutputPigpio:
        if self.use_pigpio:
            return DigitalOutputPigpio(self.dir_pin_ID, "DIR")
        else:
            return DigitalOutput(self.dir_pin_ID, "DIR")
        
    @property
    def enable(self) -> DigitalOutput | DigitalOutputPigpio | None:
        if self.en_pin_ID is not None:
            if self.use_pigpio:
                return DigitalOutputPigpio(self.en_pin_ID, "EN", active_high=False)
            else:
                return DigitalOutput(self.en_pin_ID, "EN", active_high=False)
        return None
    

@dataclass
class MicrostepPinConfig:
    """
    GPIO pin configuration for microstepping control (MS1–MS3).

    These pins allow setting the microstep resolution for drivers that use 
    logic-level pin combinations to configure microstepping.

    Attributes
    ----------
    ms1_pin_number : int | None, optional
        GPIO pin number for MS1. Default is None.
    ms2_pin_number : int | None, optional
        GPIO pin number for MS2. Default is None.
    ms3_pin_number : int | None, optional
        GPIO pin number for MS3. Default is None.
    use_pigpio:
        Indicates to use the `pigpio` library to control the GPIO pins of the
        Raspberry Pi (instead of the default higher-level `gpiozero` library).
        In a multiprocessing context the use of the `pigpio` library is 
        required.
    """
    ms1_pin_number: int | None = None
    ms2_pin_number: int | None = None
    ms3_pin_number: int | None = None
    use_pigpio: bool = False
    
    @property
    def ms1(self) -> DigitalOutput | DigitalOutputPigpio | None:
        """
        Returns a GPIO pin interface (either a `DigitalOutput` or a 
        `DigitalOutputPigpio` object) to control the MS1-input of the intended
        stepper motor driver.
        """
        if self.ms1_pin_number is not None:
            if self.use_pigpio:
                return DigitalOutputPigpio(self.ms1_pin_number, "MS1")
            return DigitalOutput(self.ms1_pin_number, "MS1")
        return None
    
    @property
    def ms2(self) -> DigitalOutput | DigitalOutputPigpio | None:
        """
        Returns a GPIO pin interface (either a `DigitalOutput` or a 
        `DigitalOutputPigpio` object) to control the MS2-input of the intended
        stepper motor driver.
        """
        if self.ms2_pin_number is not None:
            if self.use_pigpio:
                return DigitalOutputPigpio(self.ms2_pin_number, "MS2")
            return DigitalOutput(self.ms2_pin_number, "MS2")
        return None

    @property
    def ms3(self) -> DigitalOutput | DigitalOutputPigpio | None:
        """
        Returns a GPIO pin interface (either a `DigitalOutput` or a 
        `DigitalOutputPigpio` object) to control the MS3-input of the intended
        stepper motor driver.
        """
        if self.ms3_pin_number is not None:
            if self.use_pigpio:
                return DigitalOutputPigpio(self.ms3_pin_number, "MS3")
            return DigitalOutput(self.ms3_pin_number, "MS3")
        return None


class MicrostepConfig:
    """
    Encapsulates resolution and pin configuration for microstepping.

    A `MicrostepConfig` instance validates allowed resolutions, stores the
    active resolution, and can optionally hold MS1–MS3 GPIO pin mapping.

    Attributes
    ----------
    supported : set[str]
        Set of supported microstep resolutions for the specific driver.
    pin_config : MicrostepPinConfig
        Optional microstepping control pins for GPIO-based configuration.
    resolution : str | None
        Selected resolution to be applied. None until explicitly set.
    factor : int | None
        Corresponding microstep factor. None until resolution is set.
    """
    SUPPORTED_FACTORS = {
        "full": 1,
        "1/2": 2,
        "1/4": 4,
        "1/8": 8,
        "1/16": 16,
        "1/32": 32,
        "1/64": 64,
        "1/128": 128,
        "1/256": 256,
    }

    def __init__(self, supported: set[str]) -> None:
        """
        Initializes a `MicrostepConfig` object.
        
        Parameters
        ----------
        supported:
            Set of microstep resolutions supported by the intended stepper 
            motor driver.
        """
        self.supported = supported
        self.pin_config = MicrostepPinConfig()
        self.resolution: str | None = None
        self.factor: int | None = None

    def set_resolution(self, resolution: str) -> None:
        """
        Sets and validates the desired microstep resolution.
        """
        if resolution not in self.supported:
            raise ValueError(f"Unsupported microstep resolution: {resolution}")
        self.resolution = resolution
        self.factor = self.SUPPORTED_FACTORS[resolution]

    def is_configured(self) -> bool:
        """
        Checks whether the microstep resolution is set or not.
        """
        return self.resolution is not None and self.factor is not None
    

class StepperMotor(typing.Generic[TRotator], ABC):
    """
    Abstract base class for a stepper motor.

    This class manages configuration, basic timing, and motion interfaces
    for concrete stepper motor implementations.
    """
    def __init__(
        self,
        pin_config: PinConfig,
        logger: logging.Logger | None = None,
        name: str = ""
    ) -> None:
        """Creates the basics of a concrete `StepperMotor` object. 
        
        Parameters
        ----------
        pin_config:
            `PinConfig` object for configuring the essential pins that control
            the intended stepper motor driver (DIR-pin, STEP-pin, EN-pin).
        logger:
            `logging.Logger` object to log messages coming from the 
            `StepperMotor` internals. If no logger is specified, a default logger
            will be created.
        name:
            Optional name to identify the motor, e.g. in a log file.
        """
        self.pin_config = pin_config
        self.logger = logger or logging.getLogger(__name__)
        self.name = name or self.__class__.__name__

        # Motor configuration (to be set separately after instantiation)
        self.full_steps_per_rev: int = 200
        self.microstep_config: MicrostepConfig | None = None

        # Placeholder for GPIO objects
        if self.pin_config:
            self.dir = self.pin_config.dir
            self.step = self.pin_config.step
            self._enable = self.pin_config.enable

        self.rotator: TRotator | None = None

    @property
    def steps_per_degree(self) -> float:
        """
        Return the number of steps corresponding to an angular displacement of
        one degree.
        """
        if not self.microstep_config or not self.microstep_config.is_configured():
            raise RuntimeError("Microstep configuration is incomplete.")
        return self.full_steps_per_rev * self.microstep_config.factor / 360.0

    @property
    def step_angle(self) -> float:
        """
        Return the angular displacement in degrees corresponding to a single 
        step.
        """
        return 1.0 / self.steps_per_degree
    
    def configure_microstepping(
        self, 
        resolution: str,
        ms_pins: MicrostepPinConfig | None = None,
        full_steps_per_rev: int = 200
    ) -> None:
        """
        Configures the microstepping resolution of the stepper motor.

        This includes:
        - validating and applying the desired microstep resolution,
        - assigning the GPIO pins used for MS1–MS3 (if applicable),
        - and defining the number of full steps per revolution for the motor.

        This method must be called before any motion logic can calculate step 
        timing.

        Parameters
        ----------
        resolution : str
            Desired microstep resolution (e.g., "full", "1/8", "1/16").
        ms_pins : MicrostepPinConfig, optional
            GPIO pin configuration for MS1, MS2, and MS3. Defaults to None.
        full_steps_per_rev : int, optional
            Full steps per revolution of the motor. If provided, this will 
            override the current setting. Defaults to 200.

        Raises
        ------
        RuntimeError
            If no MicrostepConfig has been assigned to the motor.
        ValueError
            If the provided resolution is not supported by the MicrostepConfig.
        """
        if not self.microstep_config:
            raise RuntimeError("MicrostepConfig must be set before configuring resolution.")
        self.full_steps_per_rev = full_steps_per_rev
        if ms_pins:
            self.microstep_config.pin_config = ms_pins
        self.microstep_config.set_resolution(resolution)

    def enable(self, *args, **kwargs) -> None:
        """
        Enables the stepper driver.
        """
        if self._enable:
            self._enable.write(True)
            self.logger.info(f"[{self.name}] Driver enabled")

    def disable(self) -> None:
        """
        Disables the stepper driver.
        """
        if self._enable:
            self._enable.write(False)
            self.logger.info(f"[{self.name}] Driver disabled")

    def attach_rotator(self, kind: RotatorType) -> Rotator:
        """
        Creates and attaches a concrete `Rotator` instance to the stepper motor.
        
        All rotation commands go through the concrete `Rotator` instance which
        is attached to the concrete `StepperMotor` instance.
        
        Parameters
        ----------
        kind : RotatorType
            The concrete type of `Rotator` object to attach.
        
        Returns
        -------
        Rotator
            The created concrete `Rotator` instance.
        
        Raises
        ------
        ValueError
            If the specified `Rotator` type is unknown.
        """
        # noinspection PyUnreachableCode
        match kind:
            case RotatorType.FIXED:
                rotator = FixedRotator(self)
            case RotatorType.FIXED_THREADED:
                rotator = FixedRotatorThreaded(self)
            case RotatorType.MOTION_PROFILE:
                rotator = MotionProfileRotator(self)
            case RotatorType.MOTION_PROFILE_THREADED:
                rotator = MotionProfileRotatorThreaded(self)
            case RotatorType.DYNAMIC_THREADED:
                rotator = DynamicRotatorThreaded(self)
            case _:
                raise ValueError(f"Unsupported rotator type: {kind}")
        self.rotator = rotator
        return rotator


TStepperMotor = typing.TypeVar("TStepperMotor", bound=StepperMotor)
