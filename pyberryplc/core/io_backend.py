from abc import ABC, abstractmethod

from gpiozero.pins.pigpio import PiFactory

from .gpio import GPIO, DigitalInput, DigitalOutput, PWMOutput


class BaseIOBackend(ABC):
    """
    Base class for PLC I/O backends.

    An I/O backend creates the concrete input and output channel objects used by
    `AbstractPLC`. The PLC runtime remains responsible for scan cycle execution
    and memory registers.
    """

    @abstractmethod
    def create_digital_input(
        self,
        pin: str | int,
        label: str,
        NC_contact: bool | None = False,
    ) -> GPIO:
        ...

    @abstractmethod
    def create_digital_output(
        self,
        pin: str | int,
        label: str,
        active_high: bool = True,
        init_value: bool = 0,  # type: ignore
    ) -> GPIO:
        ...

    @abstractmethod
    def create_pwm_output(
        self,
        pin: str | int,
        label: str,
        init_value: float = 0,
        frame_width: float = 20.0,
        min_pulse_width: float = 1.0,
        max_pulse_width: float = 2.0,
        min_value: float = 0.0,
        max_value: float = 1.0,
    ) -> GPIO:
        ...


class HardwareBackend(BaseIOBackend):
    """
    I/O backend that connects PLC channels to physical Raspberry Pi GPIO pins.
    """

    def __init__(self, pin_factory: PiFactory | None = None) -> None:
        self.pin_factory = pin_factory

    def create_digital_input(
        self,
        pin: str | int,
        label: str,
        NC_contact: bool | None = False,
    ) -> GPIO:
        active_state = False if NC_contact else True
        return DigitalInput(
            pin,
            label,
            self.pin_factory,
            pull_up=None,
            active_state=active_state,
        )

    def create_digital_output(
        self,
        pin: str | int,
        label: str,
        active_high: bool = True,
        init_value: bool = 0,  # type: ignore
    ) -> GPIO:
        return DigitalOutput(
            pin,
            label,
            active_high,
            self.pin_factory,
            init_value,
        )

    def create_pwm_output(
        self,
        pin: str | int,
        label: str,
        init_value: float = 0,
        frame_width: float = 20.0,
        min_pulse_width: float = 1.0,
        max_pulse_width: float = 2.0,
        min_value: float = 0.0,
        max_value: float = 1.0,
    ) -> GPIO:
        return PWMOutput(
            pin, label, self.pin_factory, init_value, frame_width,
            min_pulse_width, max_pulse_width, min_value, max_value
        )
