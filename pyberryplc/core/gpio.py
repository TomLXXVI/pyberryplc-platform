from abc import ABC, abstractmethod
from gpiozero.pins.pigpio import PiGPIOFactory, PiFactory
from gpiozero import DigitalInputDevice, DigitalOutputDevice, PWMOutputDevice


class GPIO(ABC):
    def_pin_factory = PiGPIOFactory()
    
    def __init__(
        self, 
        pin: int | str,
        label: str,
        pin_factory: PiFactory | None = None
    ) -> None:
        self.pin = pin
        self.label = label
        if pin_factory is None:
            self.pin_factory = self.def_pin_factory
        else:
            self.pin_factory = pin_factory
    
    @abstractmethod
    def read(self) -> bool | int | float:
        pass
    
    @abstractmethod
    def write(self, value: bool | int | float) -> None:
        pass


class DigitalInput(GPIO):
        
    def __init__(
        self, 
        pin: int,
        label: str,
        pin_factory: PiFactory | None = None,
        pull_up: bool | None = None,
        active_state: bool | None = True
    ) -> None:
        """Creates a `DigitalInput` object.
        
        Parameters
        ----------
        pin:
            GPIO pin the digital input is connected to.
        label:
            Meaningful name for the digital input. This will be the name used
            in the PLC application to access the input.
        pin_factory:
            Abstraction layer that allows `gpiozero` to interface with the
            hardware-specific GPIO implementation behind the scenes. If `None`,
            the default pin factory is used, which is `PiGPIOFactory`. This
            requires that `pigpio` is installed on the Raspberry Pi, and that
            the `pigpiod` daemon is running in the background.
        pull_up:
            - If `None` (default), the hardware pin is assumed to be floating 
            and parameter `active_state` must be set.
            - If `False`, the hardware pin is assumed to be connected to ground 
            through a pull-down resistor. When the input is open, the pin reads 
            LOW (0 V). When the input is closed, the pin reads HIGH (3.3 V or 
            5 V). 
            - If `True`, the hardware pin is assumed to be connected to the 
            supply voltage through a pull-up resistor. When the input is open, 
            the pin reads HIGH (3.3 V or 5 V). When the input is closed, the pin
            reads LOW (0 V).
        active_state:
            This parameter is used in case parameter `pull_up` is `None` to set 
            the active state of the pin.
            - If `True` (default), the state of the software pin takes the 
            state of the hardware pin.
            - If `False`, the state of the software pin takes the inverse state 
            of the hardware pin.
            - If `None`, parameter `pull_up` must be set.
        """
        super().__init__(pin, label, pin_factory)
        self._device = DigitalInputDevice(
            self.pin, 
            pull_up=pull_up, 
            active_state=active_state,
            bounce_time=0,
            pin_factory=self.pin_factory
        )
    
    def read(self) -> bool:
        return self._device.value
    
    def write(self, value: bool) -> None:
        pass


class DigitalOutput(GPIO):
    
    def __init__(
        self,
        pin: int,
        label: str,
        active_high: bool = True,
        pin_factory: PiFactory | None = None,
        initial_value: bool | int | None = None
    ) -> None:
        """Creates a `DigitalOutput` object.
        
        Parameters
        ----------
        pin:
            GPIO pin the digital output is connected to.
        label:
            Meaningful name for the digital output. This will be the name used
            in the PLC application to access the output.
        active_high:
            If `True`, the output will be HIGH (e.g. at 5 V or 3.3 V) when 
            triggered (i.e. when writing 1 to it).
            If `False`, the opposite happens: the output will be LOW (pulled to 
            GND) when the output is triggered.
        pin_factory:
            Abstraction layer that allows `gpiozero` to interface with the
            hardware-specific GPIO implementation behind the scenes. If `None`,
            the default pin factory is used, which is `PiGPIOFactory`. This
            requires that `pigpio` is installed on the Raspberry Pi, and that
            the `pigpiod` daemon is running in the background.
        initial_value:
            The value that must be written to the output at the start-up of the
            program.
        """
        super().__init__(pin, label, pin_factory)
        self.initial_value = initial_value
        self._device = DigitalOutputDevice(
            self.pin, 
            active_high=active_high,
            initial_value=initial_value,
            pin_factory=self.pin_factory
        )
    
    def read(self) -> bool | int:
        return self._device.value
    
    def write(self, value: bool | int) -> None:
        try:
            value = bool(value) if isinstance(value, int) else value
            self._device.value = value
        except:
            raise ValueError("Value must be `bool` or`int`.")


class PWMOutput(GPIO):

    def __init__(
        self,
        pin: int,
        label: str,
        pin_factory: PiFactory | None = None,
        initial_value: float = 0.0,
        frame_width: float = 20.0,  # ms
        min_pulse_width: float = 1.0,  # ms
        max_pulse_width: float = 2.0,  # ms
        min_value: float = 0.0,
        max_value: float = 1.0
    ) -> None:
        """Creates a `PWMOutput` object.
        
        Parameters
        ----------
        pin:
            GPIO pin the digital output is connected to.
        label:
            Meaningful name for the digital output. This will be the name used
            in the PLC application to access the output.
        pin_factory:
            Abstraction layer that allows `gpiozero` to interface with the
            hardware-specific GPIO implementation behind the scenes. If `None`,
            the default pin factory is used, which is `PiGPIOFactory`. This
            requires that `pigpio` is installed on the Raspberry Pi, and that
            the `pigpiod` daemon is running in the background.
        initial_value:
            The initial duty cycle that must be written to the output at the 
            start-up of the program. This must be a value between 0.0 and 1.0.
        frame_width:
            Time in milliseconds (ms) between the start of the current pulse and
            the start of the next pulse. The inverse of the frame width 
            determines the frequency of the pulses (i.e. the number of emitted 
            pulses per time unit).
        min_pulse_width:
            The minimum pulse duration in milliseconds (ms).
        max_pulse_width:
            The maximum pulse duration in milliseconds (ms).
        min_value:
            The real-world value that corresponds with `min_pulse_width`.
        max_value:
            The real-world value that corresponds with `max_pulse_width`.
        """
        super().__init__(pin, label, pin_factory)
        self.initial_value = initial_value
        self.frame_width = frame_width
        self.min_pulse_width = min_pulse_width
        self.max_pulse_width = max_pulse_width
        self.min_value = min_value
        self.max_value = max_value
        self.value_range = self.max_value - self.min_value
        self.dc_range = (self.max_pulse_width - self.min_pulse_width) / self.frame_width
        self.min_dc = self.min_pulse_width / self.frame_width
        self._device = PWMOutputDevice(
            self.pin,
            active_high=True,
            initial_value=initial_value,
            frequency=int(1000.0 / self.frame_width),
            pin_factory=self.pin_factory
        )
    
    def _get_duty_cycle(self, value: float) -> float:
        """Returns the duty cycle that corresponds with `value`."""
        if value < self.min_value:
            value = self.min_value
        elif value > self.max_value:
            value = self.max_value
        dc = self.min_dc + (value - self.min_value) * self.dc_range / self.value_range
        return dc
    
    def _get_value(self, dc: float) -> float:
        """Returns the value that corresponds with duty cycle `dc`."""
        value = self.min_value + (dc - self.min_dc) * self.value_range / self.dc_range
        return value
    
    def read(self) -> float:
        value = self._get_value(self._device.value)
        return value
    
    def write(self, value: float) -> None:
        dc = self._get_duty_cycle(value)
        self._device.value = dc
    