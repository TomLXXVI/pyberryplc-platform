from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum, auto
import logging
import signal
import threading
import time
from statistics import mean, stdev
from typing import TYPE_CHECKING, TypeVar

from pyberryplc.utils.email_notification import EmailNotification

from .exceptions import (
    ConfigurationError,
    EmergencyException,
    InternalCommunicationError,
    RecoveryException,
)
from .io_backend import BaseIOBackend, HardwareBackend, IOChannel
from .memory import HMISharedData, MemoryVariable

if TYPE_CHECKING:
    from gpiozero.pins.pigpio import PiFactory


class PLCMode(Enum):
    """Execution mode of an :class:`AbstractPLC` instance."""

    INIT = auto()
    RUNNING = auto()
    EMERGENCY = auto()
    FAULT = auto()


@dataclass
class EmergencyConfig:
    """Configuration for the standard emergency-stop and reset behaviour.

    ``AbstractPLC`` can automatically create an emergency-stop input and a
    reset input when pins are supplied. Applications can also omit the pins and
    implement their own emergency policy by overriding ``emergency_is_active``
    and ``can_recover``.

    Parameters
    ----------
    emergency_pin:
        Optional pin of the local emergency-stop input. When omitted, no local
        emergency input is created by the framework.
    emergency_label:
        Label used for the emergency-stop input in the PLC input register.
        The default is ``"EmergencyButton"``.
    emergency_nc_contact:
        Indicates whether the emergency-stop input is a normally-closed
        contact. For industrial emergency-stop circuits this should usually be
        ``True``.
    reset_pin:
        Optional pin of the local reset input. When omitted, no local reset
        input is created by the framework.
    reset_label:
        Label used for the reset input in the PLC input register. The default
        is ``"ResetButton"``.
    global_emergency:
        Optional shared memory variable that latches a system-wide emergency
        state across multiple PLC units.
    latch_global_on_local_emergency:
        If ``True``, a local emergency-stop condition writes ``True`` to
        ``global_emergency``.
    clear_global_on_recover:
        If ``True``, successful recovery writes ``False`` to
        ``global_emergency``.
    """

    emergency_pin: str | int | None = None
    emergency_label: str = "EmergencyButton"
    emergency_nc_contact: bool = True
    reset_pin: str | int | None = None
    reset_label: str = "ResetButton"
    global_emergency: MemoryVariable | None = None
    latch_global_on_local_emergency: bool = True
    clear_global_on_recover: bool = True


class AbstractPLC(ABC):
    """Base class for PLC-like Python applications.

    ``AbstractPLC`` implements the repetitive PLC scan cycle, I/O register
    synchronisation, mode handling, emergency-stop handling, recovery handling,
    and default safety routines. A concrete application normally only needs to
    configure its I/O and implement ``control_routine``.

    The framework follows a template-method lifecycle:

    1. ``startup_routine`` is called once when ``run`` starts.
    2. ``control_routine`` is called every scan while the PLC is running.
    3. ``on_emergency_enter`` is called once when an emergency is detected.
    4. ``emergency_routine`` is called every scan while emergency mode is active.
    5. ``recover_routine`` is called once when recovery conditions are met.
    6. ``on_recovered`` is called after the PLC has returned to running mode.
    7. ``exit_routine`` is called when the PLC exits normally.
    8. ``crash_routine`` is called when an unexpected exception occurs.

    Applications may override any hook. The default emergency routine forces all
    outputs low and deactivates all markers. The default recovery routine
    deactivates all markers, activates configured initial markers, and keeps
    outputs in their safe state for the first recovered scan.
    """

    def __init__(
        self,
        scan_time: float = 0.1,
        hmi_data: HMISharedData | None = None,
        logger: logging.Logger | None = None,
        pin_factory: PiFactory | None = None,
        io_backend: BaseIOBackend | None = None,
        eml_notification: EmailNotification | None = None,
        emergency_config: EmergencyConfig | None = None,
    ) -> None:
        self.scan_time = scan_time
        self.hmi_data = hmi_data
        self.pin_factory = pin_factory
        self.io_backend = io_backend if io_backend else HardwareBackend(pin_factory)
        self.eml_notification = eml_notification
        self.logger = logger if logger else logging.getLogger(__name__)

        self.mode = PLCMode.INIT
        self._exit = False
        self._emergency_entered = False
        self._initial_marker_labels: set[str] = set()

        self._inputs: dict[str, IOChannel] = {}
        self._outputs: dict[str, IOChannel] = {}
        self.input_register: dict[str, MemoryVariable] = {}
        self.output_register: dict[str, MemoryVariable] = {}
        self.marker_register: dict[str, MemoryVariable] = {}

        self.hmi_input_register: dict[str, MemoryVariable] = {}
        self.hmi_output_register: dict[str, MemoryVariable] = {}
        if hmi_data:
            self._setup_hmi_data()

        self.emergency_config = emergency_config or EmergencyConfig()
        self.emergency_button: MemoryVariable | None = None
        self.reset_button: MemoryVariable | None = None
        self._setup_standard_emergency_inputs()

        exit_signal = getattr(signal, "SIGTSTP", None)
        if (
            exit_signal is not None
            and threading.current_thread() is threading.main_thread()
        ):
            signal.signal(exit_signal, lambda signum, frame: self._exit_handler())  # type: ignore

    def run(self, measure: bool = False) -> None | dict:
        """Run the PLC scan cycle until ``exit`` is requested or a fault occurs."""
        durations = []
        t_next = time.perf_counter()

        while not self._exit:
            if measure:
                t_start = time.perf_counter()

            self._update_previous_states()
            self._read_inputs()

            try:
                self._execute_scan()
            except EmergencyException as error:
                self._enter_emergency(str(error) or "Emergency requested.")
            except Exception as error:
                self.mode = PLCMode.FAULT
                self.crash_routine(error)
                break
            finally:
                self._write_outputs()

            t_next += self.scan_time
            self._wait_until(t_next)

            if measure:
                t_end = time.perf_counter()
                # noinspection PyUnboundLocalVariable
                durations.append(t_end - t_start)

        else:
            self.logger.info("Exiting PLC program - invoking exit routine.")
            self.exit_routine()
            self._write_outputs()

        if measure and durations:
            stats = self._scan_stats(durations)
            self.logger.info(f"[Jitter] {stats}")
            return stats
        return None

    def exit(self) -> None:
        """Request a normal PLC shutdown."""
        self._exit_handler()

    def request_emergency(self, reason: str = "Emergency requested.") -> None:
        """Latch the optional global emergency flag and enter emergency mode."""
        self._set_global_emergency(True)
        raise EmergencyException(reason)

    def recovery_failed(self, reason: str = "Recovery failed.") -> None:
        """Abort recovery and keep the PLC in emergency mode."""
        self._set_global_emergency(True)
        raise RecoveryException(reason)

    @abstractmethod
    def control_routine(self) -> None:
        """Application logic executed once per scan while the PLC is running."""
        ...

    def startup_routine(self) -> None:
        """Hook called once before the first running scan."""
        self.activate_initial_markers()

    def on_emergency_enter(self) -> None:
        """Hook called once when the PLC enters emergency mode."""
        self.force_outputs_off()

    def emergency_routine(self) -> None:
        """Hook called every scan while emergency mode is active."""
        self.force_outputs_off()
        self.deactivate_all_markers()

    def recover_routine(self) -> None:
        """Hook called once when recovery conditions are met."""
        self.force_outputs_off()
        self.deactivate_all_markers()
        self.activate_initial_markers()

    def on_recovered(self) -> None:
        """Hook called after the PLC has returned to running mode."""

    def exit_routine(self) -> None:
        """Hook called during normal PLC shutdown."""
        self.force_outputs_off()

    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        """Hook called when an unexpected exception occurs."""
        self.logger.critical(f"PLC crash: {exception}")
        self.force_outputs_off()

    def emergency_is_active(self) -> bool:
        """Return ``True`` when local or global emergency conditions are active."""
        local_active = False
        if self.emergency_button is not None:
            local_active = not self.emergency_button.active
            if local_active and self.emergency_config.latch_global_on_local_emergency:
                self._set_global_emergency(True)

        global_active = (
            self.emergency_config.global_emergency.active
            if self.emergency_config.global_emergency is not None
            else False
        )
        return local_active or global_active

    def can_recover(self) -> bool:
        """Return ``True`` when emergency conditions are clear and reset is active."""
        if self.emergency_button is not None and not self.emergency_button.active:
            return False
        if self.reset_button is None:
            return False
        return self.reset_button.active

    def force_outputs_off(self) -> None:
        """Set all output register values to ``False``."""
        for output in self.output_register.values():
            output.update(False)

    def deactivate_all_markers(self) -> None:
        """Deactivate all marker variables."""
        for marker in self.marker_register.values():
            marker.update(False)

    def activate_initial_markers(self) -> None:
        """Activate all markers registered as initial markers."""
        for label in self._initial_marker_labels:
            marker = self.marker_register.get(label)
            if marker is not None:
                marker.activate()

    def add_marker(
        self,
        label: str,
        init_value: bool | int = 0,
        initial: bool = False,
    ) -> MemoryVariable:
        """Add a marker variable to the marker register.

        ``initial=True`` marks the marker as an initial state that is activated
        during the default recovery routine.
        """
        marker = MemoryVariable(curr_state=init_value, prev_state=init_value)
        self.marker_register[label] = marker
        if initial or bool(init_value):
            self._initial_marker_labels.add(label)
        return marker

    def add_digital_input(
        self,
        pin: str | int,
        label: str,
        NC_contact: bool | None = False,
    ) -> MemoryVariable:
        init_value = 1 if NC_contact else 0
        self._inputs[label] = self.io_backend.create_digital_input(
            pin, label, NC_contact
        )
        self.input_register[label] = MemoryVariable(
            curr_state=init_value,
            prev_state=init_value,
        )
        return self.input_register[label]

    def add_digital_output(
        self,
        pin: str | int,
        label: str,
        active_high: bool = True,
        init_value: bool = 0,  # type: ignore
    ) -> tuple[MemoryVariable, MemoryVariable]:
        self._outputs[label] = self.io_backend.create_digital_output(
            pin, label, active_high, init_value
        )
        self.output_register[label] = MemoryVariable(
            curr_state=init_value,
            prev_state=init_value,
        )
        self.input_register[f"{label}_status"] = MemoryVariable()
        return self.output_register[label], self.input_register[f"{label}_status"]

    def add_pwm_output(
        self,
        pin: str | int,
        label: str,
        init_value: float = 0,
        frame_width: float = 20.0,
        min_pulse_width: float = 1.0,
        max_pulse_width: float = 2.0,
        min_value: float = 0.0,
        max_value: float = 1.0,
        decimal_precision: int = 0,
    ) -> tuple[MemoryVariable, MemoryVariable]:
        self._outputs[label] = self.io_backend.create_pwm_output(
            pin,
            label,
            init_value,
            frame_width,
            min_pulse_width,
            max_pulse_width,
            min_value,
            max_value,
        )
        self.output_register[label] = MemoryVariable(
            curr_state=init_value,
            prev_state=init_value,
            single_bit=False,
        )
        self.input_register[f"{label}_status"] = MemoryVariable(
            single_bit=False,
            decimal_precision=decimal_precision,
        )
        return self.output_register[label], self.input_register[f"{label}_status"]

    def di_read(self, label: str) -> bool:
        di = self._inputs.get(label)
        if di:
            return bool(di.read())
        raise ConfigurationError(f"unknown digital input `{label}`")

    def do_write(self, label: str, value: bool) -> None:
        do = self._outputs.get(label)
        if do:
            do.write(value)
            return
        raise ConfigurationError(f"unknown digital output `{label}`")

    def pwm_write(self, label: str, value: float) -> None:
        pwm_output = self._outputs.get(label)
        if pwm_output:
            pwm_output.write(value)
            return
        raise ConfigurationError(f"unknown PWM output `{label}`")

    def _execute_scan(self) -> None:
        if self.mode == PLCMode.INIT:
            self.startup_routine()
            self.mode = PLCMode.RUNNING

        if self.mode == PLCMode.RUNNING:
            if self.emergency_is_active():
                self.request_emergency("Emergency condition active.")
            self.control_routine()
            return

        if self.mode == PLCMode.EMERGENCY:
            self.emergency_routine()
            if self.can_recover():
                self._recover_from_emergency()

    def _enter_emergency(self, reason: str) -> None:
        if self.mode == PLCMode.EMERGENCY:
            return

        self.mode = PLCMode.EMERGENCY
        self._emergency_entered = True
        self.logger.critical(f"Switching to EMERGENCY mode: {reason}")
        self.on_emergency_enter()
        self.emergency_routine()

    def _recover_from_emergency(self) -> None:
        self.logger.info("Reset received. Recovering from Emergency...")
        try:
            self.recover_routine()
        except RecoveryException as error:
            self.mode = PLCMode.EMERGENCY
            self.logger.error(f"Recovery failed. Staying in EMERGENCY mode: {error}")
            return

        if self.emergency_config.clear_global_on_recover:
            self._set_global_emergency(False)
        self._emergency_entered = False
        self.mode = PLCMode.RUNNING
        self.on_recovered()
        self.logger.info("PLC successfully recovered to RUNNING mode.")

    def _setup_standard_emergency_inputs(self) -> None:
        cfg = self.emergency_config
        if cfg.emergency_pin is not None:
            self.emergency_button = self.add_digital_input(
                cfg.emergency_pin,
                cfg.emergency_label,
                NC_contact=cfg.emergency_nc_contact,
            )
            if cfg.emergency_label.isidentifier():
                setattr(self, cfg.emergency_label, self.emergency_button)

        if cfg.reset_pin is not None:
            self.reset_button = self.add_digital_input(cfg.reset_pin, cfg.reset_label)
            if cfg.reset_label.isidentifier():
                setattr(self, cfg.reset_label, self.reset_button)

    def _set_global_emergency(self, value: bool) -> None:
        if self.emergency_config.global_emergency is not None:
            self.emergency_config.global_emergency.update(value)

    def _setup_hmi_data(self) -> None:
        if self.hmi_data is not None:
            self.hmi_input_register = {
                name: MemoryVariable(curr_state=init_value, prev_state=init_value)
                for name, init_value in self.hmi_data.buttons.items()
            }
            self.hmi_input_register.update({
                name: MemoryVariable(curr_state=init_value, prev_state=init_value)
                for name, init_value in self.hmi_data.switches.items()
            })
            self.hmi_input_register.update({
                name: MemoryVariable(
                    curr_state=init_value,
                    prev_state=init_value,
                    single_bit=False,
                )
                for name, init_value in self.hmi_data.analog_inputs.items()
            })
            self.hmi_output_register = {
                name: MemoryVariable(curr_state=init_value, prev_state=init_value)
                for name, init_value in self.hmi_data.digital_outputs.items()
            }
            self.hmi_output_register.update({
                name: MemoryVariable(
                    curr_state=init_value,
                    prev_state=init_value,
                    single_bit=False,
                )
                for name, init_value in self.hmi_data.analog_outputs.items()
            })

    def _read_inputs(self) -> None:
        try:
            for input_ in self._inputs.values():
                self.input_register[input_.label].update(input_.read())
        except InternalCommunicationError as error:
            self._int_com_error_handler(error)

        if self.hmi_data:
            self._read_hmi_inputs()

    def _read_hmi_inputs(self) -> None:
        if self.hmi_data is not None:
            for name, value in self.hmi_data.buttons.items():
                self.hmi_input_register[name].update(value)
                self.hmi_data.buttons[name] = False
            for name, value in self.hmi_data.switches.items():
                self.hmi_input_register[name].update(value)
            for name, value in self.hmi_data.analog_inputs.items():
                self.hmi_input_register[name].update(value)

    def _write_outputs(self) -> None:
        try:
            for output in self._outputs.values():
                output.write(self.output_register[output.label].curr_state)
        except InternalCommunicationError as error:
            self._int_com_error_handler(error)

        if self.hmi_data:
            self._write_hmi_outputs()

    def _write_hmi_outputs(self) -> None:
        if self.hmi_data is not None:
            for name, mem_var in self.hmi_output_register.items():
                if mem_var.single_bit:
                    self.hmi_data.digital_outputs[name] = mem_var.curr_state
                else:
                    self.hmi_data.analog_outputs[name] = mem_var.curr_state

    def _update_previous_states(self) -> None:
        for marker in self.marker_register.values():
            marker.update(marker.curr_state)
        for output in self.output_register.values():
            output.update(output.curr_state)
        if self.hmi_data:
            self._update_hmi_previous_states()

    def _update_hmi_previous_states(self) -> None:
        for output in self.hmi_output_register.values():
            output.update(output.curr_state)

    def _int_com_error_handler(self, error: InternalCommunicationError) -> None:
        msg = f"program interrupted: {error.description}"
        self.logger.error(msg)
        if self.eml_notification:
            self.eml_notification.send(msg)
        raise InternalCommunicationError(error)

    def _exit_handler(self) -> None:
        self._exit = True

    @staticmethod
    def _wait_until(target_time: float, sleep_threshold: float = 0.002) -> None:
        while True:
            remaining = target_time - time.perf_counter()
            if remaining <= 0:
                break
            if remaining > sleep_threshold:
                time.sleep(0.001)
            else:
                while time.perf_counter() < target_time:
                    pass

    def _scan_stats(self, durations: list[float]) -> dict:
        overshoots = sum(d > self.scan_time for d in durations)
        return {
            "mean_duration_ms": round(mean(durations) * 1000, 3),
            "stdev_ms": round(stdev(durations) * 1000, 3)
            if len(durations) > 1
            else 0.0,
            "min_ms": round(min(durations) * 1000, 3),
            "max_ms": round(max(durations) * 1000, 3),
            "overshoots": overshoots,
            "overshoot_pct": round(100 * overshoots / len(durations), 3),
            "n": len(durations),
        }


TAbstractPLC = TypeVar("TAbstractPLC", bound=AbstractPLC)
