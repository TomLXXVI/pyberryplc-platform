import logging
import time
from pyberryplc.core.gpio import DigitalOutput
from pyberryplc.stepper.driver import StepperMotor
from pyberryplc.stepper.uart.tmc2208_uart import TMC2208UART
from pyberryplc.stepper.uart.tmc2208_registers import IHOLDIRUNRegister


class TMC2208StepperMotor(StepperMotor):
    """
    Stepper motor controlled via a TMC2208 driver using either GPIO or UART.

    This class supports:
    - Full or partial UART configuration of driver registers
    - GPIO-based microstepping via MS1/MS2 pins (fallback)
    - UART-based microstepping and current control
    - Optional high sensitivity current mode (vsense = 1)

    The driver is enabled either through GPIO (enable pin) or via UART using 
    register settings.
    """
    
    MICROSTEP_CONFIG_GPIO: dict[str, tuple[int, int]] = {
        "1/2": (1, 0),
        "1/4": (0, 1),
        "1/8": (0, 0),
        "1/16": (1, 1),
    }
    
    MICROSTEP_CONFIG_UART: dict[str, int] = {
        "1/256": 0,
        "1/128": 1,
        "1/64": 2,
        "1/32": 3,
        "1/16": 4,
        "1/8": 5,
        "1/4": 6,
        "1/2": 7,
        "full": 8
    } 
    
    def __init__(
        self,
        step_pin: int,
        dir_pin: int,
        enable_pin: int | None = None,
        ms1_pin: int | None = None,
        ms2_pin: int | None = None,
        full_steps_per_rev: int = 200,
        microstep_resolution: str = "full",
        uart: TMC2208UART | None = None,
        high_sensitivity: bool = False,
        logger: logging.Logger | None = None,
    ) -> None:
        """
        Initialize a stepper motor controlled by a TMC2208 driver.

        Parameters
        ----------
        step_pin : int
            GPIO pin used for STEP signal.
        dir_pin : int
            GPIO pin used for DIRECTION signal.
        enable_pin : int | None, optional
            GPIO pin used to enable the driver (if not using UART).
        ms1_pin : int | None, optional
            GPIO pin for MS1 microstepping control (GPIO mode).
        ms2_pin : int | None, optional
            GPIO pin for MS2 microstepping control (GPIO mode).
        full_steps_per_rev : int
            Number of full steps per motor revolution (default 200).
        microstep_resolution : str, optional
            The microstep resolution to be used. Default is full-step mode.
            Valid microstep resolutions are defined in class attribute 
            MICROSTEP_FACTORS of the base class. However, it is possible that 
            the actual driver does not support all of these. This should be 
            checked in advance.
        uart : TMC2208UART | None, optional
            UART interface for register-level control of the driver.
        high_sensitivity : bool, optional
            If True, sets CHOPCONF.vsense = 1 for V_FS = 180 mV instead of 325 mV.
        logger : logging.Logger | None, optional
            Logger for debug output.
        """
        self.ms1 = (
            DigitalOutput(ms1_pin, label="MS1")
            if ms1_pin is not None
            else None
        )
        self.ms2 = (
            DigitalOutput(ms2_pin, label="MS2")
            if ms2_pin is not None
            else None
        )
        self.uart = uart
        self.high_sensitivity = high_sensitivity
        super().__init__(
            step_pin, dir_pin, enable_pin, 
            full_steps_per_rev, microstep_resolution, 
            logger
        )

    def enable(self) -> None:
        """
        Enables the stepper driver.

        If UART is configured, this method opens the UART connection and
        configures the driver to accept software microstepping settings
        and activate current output. Otherwise, it falls back to GPIO-based
        enabling.
        """
        if self.uart is not None:
            self.uart.open()
            self.uart.update_register(
                reg_name="GCONF", 
                fields={
                    "pdn_disable": True,        # PDN_UART input function disabled. 
                    "mstep_reg_select": True    # Microstep resolution selected by MSTEP register
                }
            )
            time.sleep(0.005)
            self.uart.update_register(
                reg_name="CHOPCONF", 
                fields={
                    "toff": 3,  # enable driver - Off time setting controls duration of slow decay phase
                    "vsense": self.high_sensitivity,
                }
            )
            self.logger.info("Driver enabled")
        else:
            super().enable()

    def disable(self) -> None:
        """
        Disables the stepper driver.

        If UART is configured, this method turns off the driver's current
        output (via CHOPCONF) and closes the UART connection. Otherwise,
        it disables the driver through the GPIO enable pin.
        """
        if self.uart is not None:
            self.uart.update_register(
                reg_name="CHOPCONF", 
                fields={"toff": 0}  # Driver disable, all bridges off
            )
            self.uart.close()
            self.logger.info("Driver disabled")
        else:
            super().disable()
    
    def _validate_microstepping(self, microstep_resolution: str) -> tuple[str, int]:
        """
        Checks whether the microstep resolution is valid for the TMC2208 driver. 
        
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
            If the microstep resolution is unavailable on the TMC2208 driver.
        """
        valid_resolutions = (
            list(self.MICROSTEP_CONFIG_UART.keys()) 
            if self.uart is not None 
            else 
            list(self.MICROSTEP_CONFIG_GPIO.keys())
        )
        if microstep_resolution in valid_resolutions:
            microstep_factor = self.MICROSTEP_FACTORS[microstep_resolution]
            return microstep_resolution, microstep_factor
        else:
            raise ValueError(
                f"Microstep resolution '{microstep_resolution}' is "
                f"unavailable on this driver. "
                f"Available: {valid_resolutions}"
            )
    
    def set_microstepping(self) -> None:
        """Configures microstepping on the TMC2208 driver."""
        if self.uart is not None:
            self._set_microstepping_uart()
        else:
            self._set_microstepping_gpio()
    
    def _set_microstepping_gpio(self) -> None:
        """Configures microstepping on the TMC2208 driver via GPIO."""
        if self.ms1 and self.ms2:
            ms1_val, ms2_val = self.MICROSTEP_CONFIG_GPIO[self.microstep_resolution]
            self.ms1.write(ms1_val)
            self.ms2.write(ms2_val)
            self.logger.info(
                f"Microstepping set to {self.microstep_resolution} "
                f"(MS1={ms1_val}, MS2={ms2_val})"
            )
        else:
            self.logger.warning(
                "MS1/MS2 pins not configured, skipping microstepping setup"
            )

    def _set_microstepping_uart(self) -> None:
        """Configures microstepping on the TMC2208 driver via UART."""
        mres = self.MICROSTEP_CONFIG_UART[self.microstep_resolution]
        self.uart.update_register("CHOPCONF", {"mres": mres})
        self.logger.info(
            f"Setting microstepping via UART: {self.microstep_resolution} "
            f"(mres = {mres})"
        )

    def set_current_via_uart(
        self,
        run_current_pct: float,
        hold_current_pct: float,
        ihold_delay: int = 8
    ) -> None:
        """
        Sets motor current digitally via UART using percentages.

        Parameters
        ----------
        run_current_pct : float
            Run current as a percentage (0–100).
        hold_current_pct : float
            Hold current as a percentage (0–100).
        ihold_delay : int
            Delay before switching to hold current (0–15).
        
        Raises
        ------
        ValueError
            If any of the input parameters are outside their allowed range.
        RuntimeError
            If no UART interface is configured.
        """
        if self.uart is None:
            raise RuntimeError(
                "UART interface not available on this stepper motor."
            )

        if not (0 <= run_current_pct <= 100):
            raise ValueError("run_current_pct must be between 0 and 100.")
        if not (0 <= hold_current_pct <= 100):
            raise ValueError("hold_current_pct must be between 0 and 100.")
        if not (0 <= ihold_delay <= 15):
            raise ValueError("ihold_delay must be between 0 and 15.")

        irun = round(run_current_pct / 100 * 31)
        ihold = round(hold_current_pct / 100 * 31)

        if 0 < irun <= 1:
            irun = 1
        if 0 < ihold <= 1:
            ihold = 1

        self.uart.update_register("GCONF", {"i_scale_analog": False})
        self.uart.write_register("IHOLD_IRUN", IHOLDIRUNRegister(
            ihold=ihold,
            irun=irun,
            ihold_delay=ihold_delay
        ))
        self.logger.info(
            f"UART current config set: IRUN={irun}/31, IHOLD={ihold}/31, "
            f"DELAY={ihold_delay}"
        )
