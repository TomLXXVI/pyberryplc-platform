import typing
from abc import ABC, abstractmethod
from enum import StrEnum
from dataclasses import dataclass
import time
import logging
import threading
from collections import deque

from pyberryplc.core import DigitalOutput, DigitalOutputPigpio
from pyberryplc.motion.profile import MotionProfile
from pyberryplc.motion.dynamic_generator import DynamicDelayGenerator


class Direction(StrEnum):
    CLOCKWISE = "clockwise"
    COUNTERCLOCKWISE = "counterclockwise"

    def to_bool(self) -> bool:
        """Returns True if direction is counterclockwise, False otherwise."""
        return self == Direction.COUNTERCLOCKWISE

    def to_int(self) -> int:
        """Returns 1 for counterclockwise, 0 for clockwise."""
        return int(self.to_bool())

    def __int__(self) -> int:
        return self.to_int()

    def __bool__(self) -> bool:
        raise TypeError("Use .to_bool() for explicit conversion.")


class Rotator(ABC):
    """
    Abstract base class for stepper motor motion strategies.
    A Rotator defines how the motor is commanded to move, e.g. fixed speed,
    acceleration profile, or dynamically adjusted trajectory.
    """
    def __init__(self, motor: 'StepperMotor') -> None:
        self.motor = motor
        self._direction: Direction = Direction.COUNTERCLOCKWISE
        self._busy = False
    
    @property
    def direction(self) -> Direction:
        return self._direction

    @direction.setter
    def direction(self, value: Direction) -> None:
        """
        Sets the rotation direction in which the stepper motor should rotate.
        Either Direction.CLOCKWISE or Direction.COUNTERCLOCKWISE.
        """
        self._direction = value

    @property
    def busy(self) -> bool:
        """Return whether the motor is currently executing a motion."""
        return self._busy
    
    def _write_direction(self) -> None:
        """
        Applies the current direction setting to the DIR pin of the motor.
        """
        self.motor.dir.write(self._direction.to_bool())
    
    def _pulse_step_pin(self) -> None:
        """
        Sends a single step pulse to the STEP pin.
        The pulse duration is controlled by self.step_width.
        """
        if self.motor.step:
            self.motor.step.write(True)
            time.sleep(self.motor.step_width)
            self.motor.step.write(False)


class BlockingRotator(Rotator):

    @abstractmethod
    def rotate(self) -> None:
        """
        Executes a blocking rotation.
        Must be implemented by all concrete BlockingRotator subclasses.
        """
        pass


class NonBlockingRotator(Rotator):
    
    def __init__(self, motor: 'StepperMotor'):
        super().__init__(motor)
    
    @abstractmethod
    def start(self) -> None:
        """
        Starts a non-blocking rotation.
        Must be implemented by all concrete NonBlockingRotator subclasses.
        """
        pass


class FixedRotator(BlockingRotator):
    """
    A simple rotator that performs a blocking, constant-speed rotation
    over a specified angle.

    The speed and angle must be explicitly set before invoking rotate().
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
        return self._angle

    @angle.setter
    def angle(self, value: float) -> None:
        if value < 0:
            raise ValueError("Angle must be non-negative.")
        self._angle = value

    @property
    def omega(self) -> float:
        return self._omega

    @omega.setter
    def omega(self, value: float) -> None:
        if value <= 0:
            raise ValueError("Angular speed must be positive.")
        self._omega = value

    def _generate_delays(self) -> None:
        steps = int(self._angle * self.motor.steps_per_degree)
        delay = 1.0 / (self._omega * self.motor.steps_per_degree) - self.motor.step_width
        self._delays = [delay] * steps
        
    def rotate(self) -> None:
        if self._angle is None or self._omega is None:
            raise ValueError("Both angle and speed must be set first.")
        self._busy = True
        self._write_direction()
        self._generate_delays()
        for delay in self._delays:
            self._pulse_step_pin()
            time.sleep(delay)
        self._busy = False


class FixedRotatorThreaded(NonBlockingRotator):
    """
    Executes a non-blocking rotation in a background thread
    using a constant angular speed over a given angle.
    """
    def __init__(self, motor: 'StepperMotor') -> None:
        super().__init__(motor)
        self._angle: float | None = None
        self._omega: float | None = None
        self._thread: threading.Thread | None = None
        self._queue: deque[float] = deque()
 
    @property
    def angle(self) -> float:
        return self._angle

    @angle.setter
    def angle(self, value: float) -> None:
        if value < 0:
            raise ValueError("Angle must be non-negative.")
        self._angle = value

    @property
    def omega(self) -> float:
        return self._omega

    @omega.setter
    def omega(self, value: float) -> None:
        if value <= 0:
            raise ValueError("Angular speed must be positive.")
        self._omega = value
    
    def _generate_delays(self) -> None:
        steps = int(self._angle * self.motor.steps_per_degree)
        delay = 1.0 / (self._omega * self.motor.steps_per_degree) - self.motor.step_width
        self._queue = deque([delay] * steps)

    def _step_loop(self) -> None:
        self._busy = True
        self._write_direction()
        while self._queue:
            delay = self._queue.popleft()
            self._pulse_step_pin()
            time.sleep(delay)
        self._busy = False

    def start(self) -> None:
        if self._angle is None or self._omega is None:
            raise ValueError("Angle and speed must be set before starting.")
        self._generate_delays()
        self._thread = threading.Thread(target=self._step_loop, daemon=True)
        self._thread.start()


class ProfileRotator(Rotator):
    """
    Executes a blocking rotation based on a predefined motion profile.

    The motion profile defines the position over time. The rotator converts this
    into a delay sequence based on discrete step positions.
    """
    def __init__(self, motor: 'StepperMotor') -> None:
        super().__init__(motor)
        self._motion_profile: MotionProfile | None = None
        self._delays: list[float] = []

    @property
    def profile(self) -> MotionProfile:
        return self._motion_profile

    @profile.setter
    def profile(self, value: MotionProfile) -> None:
        self._motion_profile = value
    
    def _generate_delays(self) -> None:
        step_angle = self.motor.step_angle
        final_angle = self._motion_profile.ds_tot + step_angle
        angles = [
            i * step_angle 
            for i in range(int(final_angle / step_angle))
        ]
        times = list(map(self.profile.get_fn_time_from_position(), angles))
        self._delays = [
            max(0.0, t2 - t1 - self.motor.step_width) 
            for t1, t2 in zip(times, times[1:])
        ]

    def _step_loop(self) -> None:
        self._busy = True
        self._write_direction()
        for delay in self._delays:
            self._pulse_step_pin()
            time.sleep(delay)
        self._busy = False
    
    def rotate(self) -> None:
        if self._motion_profile is None:
            raise ValueError("Motion profile must be set first.")
        self._generate_delays()
        self._step_loop()


class ProfileRotatorThreaded(NonBlockingRotator):
    """
    Executes a non-blocking rotation in a background thread
    based on a predefined motion profile.
    """
    def __init__(self, motor: 'StepperMotor') -> None:
        super().__init__(motor)
        self._motion_profile: MotionProfile | None = None
        self._thread: threading.Thread | None = None
        self._queue: deque[float] = deque()

    @property
    def profile(self) -> MotionProfile:
        return self._motion_profile

    @profile.setter
    def profile(self, value: MotionProfile) -> None:
        self._motion_profile = value

    def _generate_delays(self) -> None:
        step_angle = self.motor.step_angle
        final_angle = self._motion_profile.ds_tot + step_angle
        angles = [i * step_angle for i in range(int(final_angle / step_angle))]
        times = list(map(self._motion_profile.get_fn_time_from_position(), angles))
        delays = [max(0.0, t2 - t1 - self.motor.step_width) for t1, t2 in zip(times, times[1:])]
        self._queue = deque(delays)

    def _step_loop(self) -> None:
        self._busy = True
        self._write_direction()
        while self._queue:
            delay = self._queue.popleft()
            self._pulse_step_pin()
            time.sleep(delay)
        self._busy = False

    def start(self) -> None:
        if self._motion_profile is None:
            raise ValueError("Motion profile must be set before starting.")
        self._generate_delays()
        self._thread = threading.Thread(target=self._step_loop, daemon=True)
        self._thread.start()
 

class DynamicRotatorThreaded(NonBlockingRotator):
    """
    Executes a non-blocking rotation with real-time delay generation.

    The delays are generated on-the-fly by a DynamicDelayGenerator, allowing
    flexible stopping and speed control during motion.
    """
    def __init__(self, motor: 'StepperMotor') -> None:
        super().__init__(motor)
        self._motion_profile: MotionProfile | None = None
        self._generator: DynamicDelayGenerator | None = None
        self._thread: threading.Thread | None = None
        self._next_step_time = 0.0

    @property
    def profile(self) -> MotionProfile:
        return self._motion_profile

    @profile.setter
    def profile(self, value: MotionProfile) -> None:
        self._motion_profile = value
    
    def _step_loop(self):
        while True:
            now = time.perf_counter()
            if now >= self._next_step_time:
                try:
                    self._pulse_step_pin()
                    delay = self._generator.next_delay()
                    self._next_step_time = now + delay
                except StopIteration:
                    self._busy = False
                    self._generator = None
                    break
            time.sleep(0.0005)
    
    def start(self) -> None:
        self._busy = True
        self._write_direction()
        self._generator = DynamicDelayGenerator(self.motor.step_angle, self._motion_profile)
        self._next_step_time = time.perf_counter()
        self._thread = threading.Thread(target=self._step_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._generator.trigger_decel()
        

class RotatorType(StrEnum):
    FIXED = "fixed"
    FIXED_THREADED = "fixed_threaded"
    PROFILE = "profile"
    PROFILE_THREADED = "profile_threaded"
    DYNAMIC_THREADED = "dynamic_threaded"


TRotator = typing.TypeVar("TRotator", bound=Rotator)


@dataclass
class PinConfig:
    """
    Configuration for essential GPIO pins of the stepper motor driver.

    Attributes
    ----------
    step_pin_number : int
        GPIO pin number for STEP signal.
    dir_pin_number : int
        GPIO pin number for DIR signal.
    en_pin_number : int | None, optional
        GPIO pin number for ENABLE signal (active-low). Default is None.
    """
    step_pin_number: int
    dir_pin_number: int
    en_pin_number: int | None = None
    use_pigpio: bool = False
    
    @property
    def step(self) -> DigitalOutput | DigitalOutputPigpio:
        if self.use_pigpio:
            return DigitalOutputPigpio(self.step_pin_number, "STEP")
        else:
            return DigitalOutput(self.step_pin_number, "STEP")
    
    @property
    def dir(self) -> DigitalOutput | DigitalOutputPigpio:
        if self.use_pigpio:
            return DigitalOutputPigpio(self.dir_pin_number, "DIR")
        else:
            return DigitalOutput(self.dir_pin_number, "DIR")
        
    @property
    def enable(self) -> DigitalOutput | DigitalOutputPigpio | None:
        if self.en_pin_number is not None:
            if self.use_pigpio:
                return DigitalOutputPigpio(self.en_pin_number, "EN", active_high=False)
            else:
                return DigitalOutput(self.en_pin_number, "EN", active_high=False)
        return None
    

@dataclass
class MicrostepPinConfig:
    """
    GPIO pin configuration for microstepping control (MS1–MS3).

    These pins allow setting the microstep resolution for drivers
    that use logic-level pin combinations to configure microstepping.

    Attributes
    ----------
    ms1_pin_number : int | None, optional
        GPIO pin number for MS1. Default is None.
    ms2_pin_number : int | None, optional
        GPIO pin number for MS2. Default is None.
    ms3_pin_number : int | None, optional
        GPIO pin number for MS3. Default is None.
    """
    ms1_pin_number: int | None = None
    ms2_pin_number: int | None = None
    ms3_pin_number: int | None = None
    use_pigpio: bool = False
    
    @property
    def ms1(self) -> DigitalOutput | DigitalOutputPigpio | None:
        if self.ms1_pin_number is not None:
            if self.use_pigpio:
                return DigitalOutputPigpio(self.ms1_pin_number, "MS1")
            return DigitalOutput(self.ms1_pin_number, "MS1")
        return None
    
    @property
    def ms2(self) -> DigitalOutput | DigitalOutputPigpio | None:
        if self.ms2_pin_number is not None:
            if self.use_pigpio:
                return DigitalOutputPigpio(self.ms2_pin_number, "MS2")
            return DigitalOutput(self.ms2_pin_number, "MS2")
        return None

    @property
    def ms3(self) -> DigitalOutput | DigitalOutputPigpio | None:
        if self.ms3_pin_number is not None:
            if self.use_pigpio:
                return DigitalOutputPigpio(self.ms3_pin_number, "MS3")
            return DigitalOutput(self.ms3_pin_number, "MS3")
        return None


class MicrostepConfig:
    """
    Encapsulates resolution and pin configuration for microstepping.

    A MicrostepConfig instance validates allowed resolutions, stores the
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
        self.supported = supported
        self.pin_config = MicrostepPinConfig()
        self.resolution: str | None = None
        self.factor: int | None = None

    def set_resolution(self, resolution: str) -> None:
        """Set and validate the desired microstep resolution."""
        if resolution not in self.supported:
            raise ValueError(f"Unsupported microstep resolution: {resolution}")
        self.resolution = resolution
        self.factor = self.SUPPORTED_FACTORS[resolution]

    def is_configured(self) -> bool:
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
        self.pin_config = pin_config
        self.logger = logger or logging.getLogger(__name__)
        self.name = name or self.__class__.__name__

        # Motor configuration (to be set separately)
        self.full_steps_per_rev: int = 200
        self.microstep_config: MicrostepConfig | None = None

        # Placeholder for GPIO objects
        self.dir = self.pin_config.dir
        self.step = self.pin_config.step
        self._enable = self.pin_config.enable

        self.step_width = 10e-6  # default step pulse width in seconds
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
        Instantiates and attaches a rotator of the specified type to this motor.
        
        Parameters
        ----------
        kind : RotatorType
            The type of rotator to attach.
        
        Returns
        -------
        Rotator
            The created rotator instance.
        
        Raises
        ------
        ValueError
            If the specified type is unknown.
        """
        # noinspection PyUnreachableCode
        match kind:
            case RotatorType.FIXED:
                rotator = FixedRotator(self)
            case RotatorType.FIXED_THREADED:
                rotator = FixedRotatorThreaded(self)
            case RotatorType.PROFILE:
                rotator = ProfileRotator(self)
            case RotatorType.PROFILE_THREADED:
                rotator = ProfileRotatorThreaded(self)
            case RotatorType.DYNAMIC_THREADED:
                rotator = DynamicRotatorThreaded(self)
            case _:
                raise ValueError(f"Unsupported rotator type: {kind}")
        self.rotator = rotator
        return rotator
