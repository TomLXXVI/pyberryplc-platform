from typing import Any, Protocol
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
import threading

from gpiozero.pins.pigpio import PiFactory

from .gpio import DigitalInput, DigitalOutput, PWMOutput


class IOChannel(Protocol):
    pin: str | int
    label: str

    def read(self) -> bool | int | float:
        ...

    def write(self, value: Any) -> None:
        ...


@dataclass
class SoftMachineState:
    """
    Thread-safe state shared between a PLC application and a soft-machine UI.

    Channels are addressed by their virtual pin. Labels and metadata are kept
    separately so a UI can present meaningful names while the backend keeps the
    same pin-based connection model as the hardware backend.
    """
    digital_inputs: dict[str, bool] = field(default_factory=dict)
    digital_outputs: dict[str, bool] = field(default_factory=dict)
    pwm_outputs: dict[str, float] = field(default_factory=dict)
    digital_input_labels: dict[str, str] = field(default_factory=dict)
    digital_output_labels: dict[str, str] = field(default_factory=dict)
    pwm_output_labels: dict[str, str] = field(default_factory=dict)
    digital_input_nc_contacts: dict[str, bool] = field(default_factory=dict)
    digital_output_active_high: dict[str, bool] = field(default_factory=dict)
    pwm_output_ranges: dict[str, tuple[float, float]] = field(default_factory=dict)
    lock: Any = field(default_factory=threading.Lock, repr=False)

    @staticmethod
    def _key(pin: str | int) -> str:
        return str(pin)

    def register_digital_input(
        self,
        pin: str | int,
        label: str,
        NC_contact: bool | None = False,
    ) -> None:
        key = self._key(pin)
        with self.lock:
            self.digital_inputs.setdefault(key, bool(NC_contact))
            self.digital_input_labels[key] = label
            self.digital_input_nc_contacts[key] = bool(NC_contact)

    def register_digital_output(
        self,
        pin: str | int,
        label: str,
        active_high: bool = True,
        init_value: bool = 0,  # type: ignore
    ) -> None:
        key = self._key(pin)
        with self.lock:
            self.digital_outputs.setdefault(key, bool(init_value))
            self.digital_output_labels[key] = label
            self.digital_output_active_high[key] = active_high

    def register_pwm_output(
        self,
        pin: str | int,
        label: str,
        init_value: float = 0,
        min_value: float = 0.0,
        max_value: float = 1.0,
    ) -> None:
        key = self._key(pin)
        with self.lock:
            self.pwm_outputs.setdefault(key, float(init_value))
            self.pwm_output_labels[key] = label
            self.pwm_output_ranges[key] = (min_value, max_value)

    def read_digital_input(self, pin: str | int) -> bool:
        with self.lock:
            return self.digital_inputs.get(self._key(pin), False)

    def set_digital_input(self, pin: str | int, value: bool) -> None:
        with self.lock:
            self.digital_inputs[self._key(pin)] = bool(value)

    def read_digital_output(self, pin: str | int) -> bool:
        with self.lock:
            return self.digital_outputs.get(self._key(pin), False)

    def write_digital_output(self, pin: str | int, value: bool | int) -> None:
        with self.lock:
            self.digital_outputs[self._key(pin)] = bool(value)

    def read_pwm_output(self, pin: str | int) -> float:
        with self.lock:
            return self.pwm_outputs.get(self._key(pin), 0.0)

    def write_pwm_output(self, pin: str | int, value: float) -> None:
        key = self._key(pin)
        with self.lock:
            min_value, max_value = self.pwm_output_ranges.get(key, (0.0, 1.0))
            value = max(min_value, min(max_value, float(value)))
            self.pwm_outputs[key] = value


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
    ) -> IOChannel:
        ...

    @abstractmethod
    def create_digital_output(
        self,
        pin: str | int,
        label: str,
        active_high: bool = True,
        init_value: bool = 0,  # type: ignore
    ) -> IOChannel:
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
    ) -> IOChannel:
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
    ) -> IOChannel:
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
    ) -> IOChannel:
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
    ) -> IOChannel:
        return PWMOutput(
            pin, label, self.pin_factory, init_value, frame_width,
            min_pulse_width, max_pulse_width, min_value, max_value
        )


class SoftDigitalInput:

    def __init__(
        self,
        state: SoftMachineState,
        pin: str | int,
        label: str,
        NC_contact: bool | None = False,
    ) -> None:
        self.state = state
        self.pin = pin
        self.label = label
        self.NC_contact = NC_contact
        self.state.register_digital_input(pin, label, NC_contact)

    def read(self) -> bool:
        return self.state.read_digital_input(self.pin)

    def write(self, value: bool | int | float) -> None:
        pass


class SoftDigitalOutput:

    def __init__(
        self,
        state: SoftMachineState,
        pin: str | int,
        label: str,
        active_high: bool = True,
        init_value: bool = 0,  # type: ignore
    ) -> None:
        self.state = state
        self.pin = pin
        self.label = label
        self.active_high = active_high
        self.state.register_digital_output(pin, label, active_high, init_value)

    def read(self) -> bool:
        return self.state.read_digital_output(self.pin)

    def write(self, value: bool | int | float) -> None:
        self.state.write_digital_output(self.pin, bool(value))


class SoftPWMOutput:

    def __init__(
        self,
        state: SoftMachineState,
        pin: str | int,
        label: str,
        init_value: float = 0,
        min_value: float = 0.0,
        max_value: float = 1.0,
    ) -> None:
        self.state = state
        self.pin = pin
        self.label = label
        self.state.register_pwm_output(
            pin, label, init_value, min_value, max_value
        )

    def read(self) -> float:
        return self.state.read_pwm_output(self.pin)

    def write(self, value: bool | int | float) -> None:
        self.state.write_pwm_output(self.pin, float(value))


class SoftwareBackend(BaseIOBackend):
    """
    In-process I/O backend for connecting a PLC to a soft-machine.
    """

    def __init__(self, state: SoftMachineState | None = None) -> None:
        self.state = state if state is not None else SoftMachineState()

    def create_digital_input(
        self,
        pin: str | int,
        label: str,
        NC_contact: bool | None = False,
    ) -> IOChannel:
        return SoftDigitalInput(self.state, pin, label, NC_contact)

    def create_digital_output(
        self,
        pin: str | int,
        label: str,
        active_high: bool = True,
        init_value: bool = 0,  # type: ignore
    ) -> IOChannel:
        return SoftDigitalOutput(
            self.state, pin, label, active_high, init_value
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
    ) -> IOChannel:
        return SoftPWMOutput(
            self.state, pin, label, init_value, min_value, max_value
        )
