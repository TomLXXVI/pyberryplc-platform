from typing import TypeVar
from abc import ABC, abstractmethod
import signal
import logging
import threading
import time
from statistics import mean, stdev

from gpiozero.pins.pigpio import PiFactory

from pyberryplc.utils.email_notification import EmailNotification

from .gpio import GPIO, DigitalInput, DigitalOutput, PWMOutput
from .memory import MemoryVariable, HMISharedData
from .exceptions import *


class AbstractPLC(ABC):
    """
    Abstract framework class: implements all functionality common to any PLC 
    application running on Raspberry Pi.

    To write a specific PLC application, the user needs to write its own class
    inherited from this base class and implement the abstract methods of this 
    base class:
    - `control_routine()`
    - `exit_routine()`
    - `emergency_routine()`
    - `crash_routine()`
    """
    def __init__(
        self,
        scan_time: float = 0.1,
        hmi_data: HMISharedData | None = None,
        logger: logging.Logger | None = None,
        pin_factory: PiFactory | None = None,
        eml_notification: EmailNotification | None = None
    ) -> None:
        """Creates an `AbstractPLC` instance.

        Parameters
        ----------
        scan_time : float
            Minimum time duration for the PLC scan cycle. The default is 0.1 s 
            (100 ms).
        hmi_data : HMISharedData, optional
            `HMISharedData` object holding HMI inputs (buttons, switches, or 
            analog inputs) and HMI outputs (digital and analog outputs). Other 
            kind of data can also be exchanged between the HMI and PLC 
            application.
        logger : logging.Logger, optional
            Logs messages to inspect what is going on.
        pin_factory:
            Abstraction layer that allows `gpiozero` to interface with the
            hardware-specific GPIO implementation behind the scenes. If `None`,
            the default pin factory is used, which is `PiGPIOFactory`. This
            requires that `pigpio` is installed on the Raspberry Pi, and that
            the `pigpiod` daemon is running in the background.
        eml_notification: optional
            Instance of class `EmailNotification` (see module
            email_notification.py). Allows to send email messages if certain
            events have occurred (e.g. to send an alarm).
        
        Notes
        -----
        The PLC object has an attribute `logger` that can be used to write
        messages to a log file and for displaying in the terminal window. See 
        also `/utils/log_utils.py`.
        """
        self.scan_time = scan_time
        self.hmi_data = hmi_data
        self.pin_factory = pin_factory
        self._exit: bool = False
        
        # Attaches an e-mail notification service (can be None).
        self.eml_notification = eml_notification

        # Attaches a logger to the PLC application (note: the logger can be 
        # configured by calling the function `init_logger()` in module 
        # `/utils/log_utils.py` at the start of the main program).
        self.logger = logger if logger else logging.getLogger(__name__)

        # Dictionaries that hold the physical GPIO inputs/outputs used by the 
        # PLC application.
        self._inputs: dict[str, GPIO] = {}
        self._outputs: dict[str, GPIO] = {}
        
        # Dictionaries where the states of inputs/outputs are stored. These are
        # the memory registries of the PLC. The program logic reads from or 
        # writes to these registries.
        self.input_register: dict[str, MemoryVariable] = {}
        self.output_register: dict[str, MemoryVariable] = {}
        self.marker_register: dict[str, MemoryVariable] = {}
        
        # Dictionaries where the states of HMI inputs/outputs, and where other 
        # data from the HMI are stored, if an HMI is connected to the PLC 
        # application: 
        self.hmi_input_register: dict[str, MemoryVariable] = {}
        self.hmi_output_register: dict[str, MemoryVariable] = {}
        if hmi_data: self._setup_hmi_shared_data()
        
        # To terminate program: press Ctrl-Z and method `_exit_handler` will be
        # called which terminates the PLC scanning loop.
        if threading.current_thread() is threading.main_thread():
            signal.signal(signal.SIGTSTP, lambda signum, frame: self._exit_handler())
    
    def run(self, measure: bool = False) -> None | dict:
        """
        Implements the global running operation of the PLC.
        
        If `measure` is `True` variations in PLC scan cycle time are measured
        and a dict with a few stats is returned and also send to the logger.
        """
        durations = []
        t_next = time.perf_counter()
        try:
            while not self._exit:
                if measure:
                    t_start = time.perf_counter()
                
                self._update_previous_states()
                self._read_inputs()
                
                try:
                    self.control_routine()  # to be implemented in derived class
                except EmergencyException:
                    self.logger.warning(
                        "Emergency stop triggered — invoking emergency routine."
                    )
                    self.emergency_routine()  # to be implemented in derived class
                    return None
                finally:
                    self._write_outputs()
                                
                t_next += self.scan_time
                self._wait_until(t_next)
                
                if measure:
                    t_end = time.perf_counter()
                    # noinspection PyUnboundLocalVariable
                    durations.append(t_end - t_start)
            
            else:
                # `else` block is only executed when `self._exit` has become 
                # `True`, not when while-loop is interrupted by 
                # `EmergencyException`.
                self.logger.info(
                    "Exiting PLC program — invoking exit routine."
                )
                self.exit_routine()  # to be implemented in derived class
                self._write_outputs()

        except KeyboardInterrupt as e:
            self.logger.warning(
                "KeyboardInterrupt received — invoking crash routine."
            )
            self.crash_routine(e)  # to be implemented in derived class

        except Exception as e:
            self.logger.error(
                "Unexpected exception occurred — invoking crash routine."
            )
            self.crash_routine(e)
        
        if measure and durations:
            overshoots = sum(d > self.scan_time for d in durations)
            stats = {
                "mean_duration_ms": round(mean(durations) * 1000, 3),
                "stdev_ms": round(stdev(durations) * 1000, 3) if len(durations) > 1 else 0.0,
                "min_ms": round(min(durations) * 1000, 3),
                "max_ms": round(max(durations) * 1000, 3),
                "overshoots": overshoots,
                "overshoot_pct": round(100 * overshoots / len(durations), 3),
                "n": len(durations),
            }
            self.logger.info(f"[Jitter] {stats}")
            return stats
        return None
    
    def exit(self) -> None:
        """Terminates the PLC scanning loop and invokes `exit_routine()`."""
        self._exit_handler()
    
    @abstractmethod
    def control_routine(self) -> None:
        """Implements the running operation of the PLC-application.

        Must be overridden in the PLC application class derived from this class.
        """
        ...

    @abstractmethod
    def exit_routine(self) -> None:
        """Implements the routine that is called when the PLC-application is
        to be stopped, i.e. when the user has pressed the key combination
        <Ctrl-Z> on the keyboard of the PLC (Raspberry Pi).

        Must be overridden in the PLC application class derived from this class.
        """
        ...

    @abstractmethod
    def emergency_routine(self) -> None:
        """Implements the routine for when an `EmergencyException` has been 
        raised. An `EmergencyException` can be raised anywhere within the
        `control_routine` method to signal an emergency situation for which the
        PLC application must be terminated.

        Must be overridden in the PLC application class derived from this class.
        """
        ...

    @abstractmethod
    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        """Handles unexpected runtime exceptions.

        Must be overridden in the PLC application class derived from this class.
        """
        ...
    
    def add_digital_input(
        self,
        pin: str | int,
        label: str,
        NC_contact: bool | None = False,
    ) -> MemoryVariable:
        """Adds a digital input to the PLC application.

        Parameters
        ----------
        pin:
            GPIO pin the digital input is connected to.
        label:
            Meaningful name for the digital input. This will be the name used
            in the PLC application to access the input.
        NC_contact:
            Indicates if the digital input has a NC-contact. Default is `False`,
            which means by default a NO-contact is assumed.
        
        Returns
        -------
        The memory variable of the digital input in the input memory registry.
        """
        if NC_contact:
            active_state = False
            init_value = 1
        else:
            active_state = True
            init_value = 0
        self._inputs[label] = DigitalInput(
            pin, 
            label, 
            self.pin_factory, 
            pull_up=None, 
            active_state=active_state
        )
        self.input_register[label] = MemoryVariable(
            curr_state=init_value,
            prev_state=init_value
        )
        return self.input_register[label]

    def add_digital_output(
        self,
        pin: str | int,
        label: str,
        active_high: bool = True,
        init_value: bool = 0
    ) -> tuple[MemoryVariable, MemoryVariable]:
        """Adds a digital output to the PLC application.
        
        Parameters
        ----------
        pin:
            GPIO pin the digital input is connected to.
        label:
            Meaningful name for the digital input. This will be the name used
            in the PLC application to access the input.
        active_high:
            If `True`, the output will be HIGH (e.g. at 5 V or 3.3 V) when 
            triggered (i.e. when writing 1 to it).
            If `False`, the opposite happens: the output will be LOW (pulled to 
            GND) when the output is triggered.
        init_value:
            Initial value that must be written to the digital output.
        
        Returns
        -------
        The memory variable of the digital output in the output memory registry, 
        and the memory variable of its status in the input memory registry. 
        """
        self._outputs[label] = DigitalOutput(
            pin, 
            label,
            active_high,
            self.pin_factory,
            init_value
        )
        self.output_register[label] = MemoryVariable(
            curr_state=init_value,
            prev_state=init_value
        )
        self.input_register[f"{label}_status"] = MemoryVariable()
        return (
            self.output_register[label], 
            self.input_register[f"{label}_status"]
        )
    
    def add_pwm_output(
        self,
        pin: str | int,
        label: str,
        init_value: float = 0,
        frame_width: float = 20.0,  # ms
        min_pulse_width: float = 1.0,  # ms
        max_pulse_width: float = 2.0,  # ms
        min_value: float = 0.0,
        max_value: float = 1.0,
        decimal_precision: int = 0
    ) -> tuple[MemoryVariable, MemoryVariable]:
        """Adds a Pulse-Width-Modulation (PWM) output to the PLC application.
        
        Parameters
        ----------
        pin:
            GPIO pin the digital output is connected to.
        label:
            Meaningful name for the digital output. This will be the name used
            in the PLC application to access the output.
        init_value:
            Initial duty cycle that must be written to the output at the 
            start-up of the program. This must be a value between 0.0 and 1.0.
        frame_width:
            Time in milliseconds (ms) between the start of the current pulse and
            the start of the next pulse. The inverse of the frame width 
            determines the frequency of the pulses (i.e. the number of emitted 
            pulses per time unit).
        min_pulse_width:
            Minimum pulse duration in milliseconds (ms).
        max_pulse_width:
            Maximum pulse duration in milliseconds (ms).
        min_value:
            The real-world value that corresponds with `min_pulse_width`.
        max_value:
            The real-world value that corresponds with `max_pulse_width`.
        decimal_precision:
            Number of decimal places the status value is rounded to when read
            from the memory input registry. 
        
        Returns
        -------
        The memory variable of the PWM output in the output memory registry, 
        and the memory variable of its status in the input memory registry.
        """
        self._outputs[label] = PWMOutput(
            pin, label, self.pin_factory, init_value, frame_width, 
            min_pulse_width, max_pulse_width, min_value, max_value
        )
        self.output_register[label] = MemoryVariable(
            curr_state=init_value,
            prev_state=init_value,
            single_bit=False
        )
        self.input_register[f"{label}_status"] = MemoryVariable(
            single_bit=False,
            decimal_precision=decimal_precision
        )
        return (
            self.output_register[label],
            self.input_register[f"{label}_status"]
        )
    
    def add_marker(self, label: str, init_value: bool | int = 0) -> MemoryVariable:
        """Adds a marker to the marker-registry of the PLC-application and 
        returns its `MemoryVariable` object.
        """
        marker = MemoryVariable(
            curr_state=init_value,
            prev_state=init_value
        )
        self.marker_register[label] = marker
        return marker

    def di_read(self, label: str) -> bool:
        """Reads the current state of the digital input specified by the given
        label.

        Raises a `ConfigurationError` exception if the digital input with the
        given label has not been added to the PLC-application before.

        Returns the read value (integer). If the digital input has been
        configured as normally closed, the inverted value is returned.
        """
        di = self._inputs.get(label)
        if di:
            value = di.read()
            return value
        else:
            raise ConfigurationError(f"unknown digital input `{label}`")

    def do_write(self, label: str, value: bool) -> None:
        """Writes the given value (bool) to the digital output with the given
        label.

        Raises a `ConfigurationError` exception if the digital output with the
        given label has not been added to the PLC-application before.
        """
        do = self._outputs.get(label)
        if do:
            do.write(value)
        else:
            raise ConfigurationError(f"unknown digital output `{label}`")
    
    def pwm_write(self, label: str, value: float) -> None:
        """Writes the given value (float) to the PWM output with the given 
        label.
        
        Raises a `ConfigurationError` exception if the PWM output with the
        given label has not been added to the PLC-application before.
        """
        pwm_output = self._outputs.get(label)
        if pwm_output:
            pwm_output.write(value)
        else:
            raise ConfigurationError(f"unknown PWM output `{label}`")
    
    def _setup_hmi_shared_data(self) -> None:
        """If a `HMISharedData` object is passed to `AbstractPLC.__init__(), 
        creates separate HMI input and output registers.
        """
        self.hmi_input_register = {
            name: MemoryVariable(curr_state=init_value, prev_state=init_value)
            for name, init_value in self.hmi_data.buttons.items()
        }
        self.hmi_input_register.update({
            name: MemoryVariable(curr_state=init_value, prev_state=init_value)
            for name, init_value in self.hmi_data.switches.items()
        })
        self.hmi_input_register.update({
            name: MemoryVariable(curr_state=init_value, prev_state=init_value, single_bit=False)
            for name, init_value in self.hmi_data.analog_inputs.items()
        })
        self.hmi_output_register = {
            name: MemoryVariable(curr_state=init_value, prev_state=init_value)
            for name, init_value in self.hmi_data.digital_outputs.items()
        }
        self.hmi_output_register.update({
            name: MemoryVariable(curr_state=init_value, prev_state=init_value, single_bit=False)
            for name, init_value in self.hmi_data.analog_outputs.items()
        })
    
    def _read_inputs(self) -> None:
        """Reads all physical inputs (defined in the PLC application) and writes 
        their current states to the PLC input register.
        
        If an HMI is connected to the PLC application (`hmi_shared_data` is not 
        `None`) also updates the memory variables in the HMI input register
        from the shared data object.
        
        Raises
        ------
        `InternalCommunicationError` when a read operation fails.
        """
        try:
            for input_ in self._inputs.values():
                self.input_register[input_.label].update(input_.read())
        except InternalCommunicationError as error:
            self._int_com_error_handler(error)
        
        if self.hmi_data: self._read_hmi_inputs()
    
    def _read_hmi_inputs(self) -> None:
        """Reads HMI inputs from `self.hmi_shared_data` and updates the HMI input
        register.
        """
        for name, value in self.hmi_data.buttons.items():
            self.hmi_input_register[name].update(value)
            # if an HMI button state has been read into the HMI register of the
            # PLC always reset this state in `self.hmi_shared_data` (i.e. means that
            # a button press in the HMI is valid for only one PLC scan cycle).
            self.hmi_data.buttons[name] = False

        for name, value in self.hmi_data.switches.items():
            self.hmi_input_register[name].update(value)

        for name, value in self.hmi_data.analog_inputs.items():
            self.hmi_input_register[name].update(value)
    
    def _write_outputs(self) -> None:
        """Writes all current states in the PLC output register to the 
        corresponding physical outputs.
        
        If an HMI is connected to the PLC application (`hmi_shared_data` is not 
        `None`) writes the current states in the HMI output register to
        the shared data object.
        
        Raises
        ------
        `InternalCommunicationError` when a write operation fails.
        """
        try:
            for output in self._outputs.values():
                output.write(self.output_register[output.label].curr_state)
        except InternalCommunicationError as error:
            self._int_com_error_handler(error)
        
        if self.hmi_data: self._write_hmi_outputs()
    
    def _write_hmi_outputs(self) -> None:
        """Writes the state of the HMI outputs in the PLC HMI output register
        to `self.hmi_shared_data`.
        """
        for name, mem_var in self.hmi_output_register.items():
            if mem_var.single_bit:
                self.hmi_data.digital_outputs[name] = mem_var.curr_state
            else:
                self.hmi_data.analog_outputs[name] = mem_var.curr_state
    
    def _update_previous_states(self):
        """At the start of each new PLC scan cycle, the values in the "current 
        state" location of marker and output variables are moved to their 
        "previous state" location. This allows for edge detection on these 
        variables.
        """
        for marker in self.marker_register.values():
            marker.update(marker.curr_state)
        
        for output in self.output_register.values():
            output.update(output.curr_state)
        
        if self.hmi_data: self._update_hmi_previous_states()
    
    def _update_hmi_previous_states(self) -> None:
        """At the start of each new PLC scan cycle, the current states of HMI 
        outputs in the PLC's HMI output register are moved to the previous state
        location.
        """
        for output in self.hmi_output_register.values():
            output.update(output.curr_state)
    
    def _int_com_error_handler(self, error: InternalCommunicationError):
        """Handles an `InternalCommunication` exception. An error message is
        sent to the logger. If the email notification service is used, an email
        is sent with the error message. Finally, method `crash_routine()` is 
        called and the PLC application will be terminated.
        """
        msg = f"program interrupted: {error.description}"
        self.logger.error(msg)
        if self.eml_notification: self.eml_notification.send(msg)
        raise InternalCommunicationError

    def _exit_handler(self):
        """Terminates the PLC scanning loop when the user has pressed the key
        combination <Ctrl-Z> on the keyboard of the PLC (Raspberry Pi) to stop
        the PLC application.
        """
        self._exit = True
    
    @staticmethod
    def _wait_until(target_time: float, sleep_threshold: float = 0.002):
        """
        Busy-wait loop that waits until the given target time (in seconds, from 
        time.perf_counter()). It uses a hybrid strategy: sleep lightly when far 
        from the target, spin actively near the end.

        Parameters
        ----------
        target_time: 
            Absolute target time (not duration!) in seconds.
        sleep_threshold: 
            When more than this time remains, the function sleeps briefly.
        """
        while True:
            remaining = target_time - time.perf_counter()
            if remaining <= 0:
                break
            elif remaining > sleep_threshold:
                time.sleep(0.001)
            else:
                # Busy-wait in final critical period
                while time.perf_counter() < target_time:
                    pass


TAbstractPLC = TypeVar("TAbstractPLC", bound=AbstractPLC)
