from abc import ABC, abstractmethod
from enum import StrEnum
from dataclasses import dataclass
import time
import logging


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
    Abstract base class for all motion strategies.
    A Rotator defines how a stepper motor executes a motion.
    """

    def __init__(self, motor: 'StepperMotor') -> None:
        self.motor = motor
        self._direction: Direction = Direction.COUNTERCLOCKWISE

    def set_direction(self, direction: Direction) -> None:
        self._direction = direction

    def _write_direction(self) -> None:
        self.motor.dir.write(self._direction.to_bool())
    
    @abstractmethod
    def rotate(self) -> None:
        """Perform a blocking motion."""
        pass

    def start(self) -> None:
        """Start a non-blocking motion (optional override)."""
        raise NotImplementedError("This rotator does not support non-blocking mode.")


class FixedRotator(Rotator):
    """
    Blocking fixed-speed rotation.
    """
    def __init__(
        self, 
        motor: 'StepperMotor',
    ) -> None:
        super().__init__(motor)
        self._angle: float = 0.0
        self._omega: float = 0.0

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

    def rotate(self) -> None:
        if self._angle is None or self._omega is None:
            raise ValueError("Both angle and speed must be set before calling rotate().")

        self._write_direction()
        steps = int(self._angle * self.motor.steps_per_degree)
        delay = 1.0 / (self._omega * self.motor.steps_per_degree) - self.motor.step_width

        for _ in range(steps):
            self.motor._pulse_step_pin()
            time.sleep(delay)


@dataclass
class PinConfig:
    """
    Configuration for essential GPIO pins of the stepper motor driver.

    Attributes
    ----------
    step : int
        GPIO pin number for STEP signal.
    dir : int
        GPIO pin number for DIR signal.
    enable : int | None, optional
        GPIO pin number for ENABLE signal (active-low). Default is None.
    """
    step: int
    dir: int
    enable: int | None = None


@dataclass
class MicrostepPinConfig:
    """
    Configuration for microstepping control pins (optional, driver-dependent).

    Attributes
    ----------
    ms1 : int | None, optional
        GPIO pin number for MS1. Default is None.
    ms2 : int | None, optional
        GPIO pin number for MS2. Default is None.
    ms3 : int | None, optional
        GPIO pin number for MS3. Default is None.
    """
    ms1: int | None = None
    ms2: int | None = None
    ms3: int | None = None


class MicrostepConfig:
    """
    Encapsulates microstepping resolution support and configuration.

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

    def __init__(
        self,
        supported: set[str],
        pin_config: MicrostepPinConfig | None = None
    ) -> None:
        self.supported = supported
        self.pin_config = pin_config or MicrostepPinConfig()
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


class StepperMotor(ABC):
    """
    Abstract base class for a stepper motor.
    Provides core attributes and helper methods required by rotators.
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
        self.dir = None
        self.step = None
        self._enable = None

        # Default timing
        self.step_width = 10e-6  # seconds

    @property
    def steps_per_degree(self) -> float:
        """
        Return the number of steps corresponding to an angular displacement of
        one degree.
        """
        return self.full_steps_per_rev * self.microstep_config.factor / 360.0

    @property
    def step_angle(self) -> float:
        """
        Return the angular displacement in degrees corresponding to a single 
        step.
        """
        return 1.0 / self.steps_per_degree
    
    def configure_microstepping(self, resolution: str) -> None:
        """
        Configure the microstep resolution. This must match the supported 
        resolutions provided in the driver's MicrostepConfig.

        Parameters
        ----------
        resolution : str
            The desired microstep resolution (e.g., "1/8").

        Raises
        ------
        ValueError
            If the resolution is unsupported.
        """
        self.microstep_config.set_resolution(resolution)

    def _pulse_step_pin(self) -> None:
        if self.step:
            self.step.write(True)
            time.sleep(self.step_width)
            self.step.write(False)
