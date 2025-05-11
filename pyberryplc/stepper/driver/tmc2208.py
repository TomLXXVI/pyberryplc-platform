import logging
import time

from pyberryplc.stepper.driver.base import (
    StepperMotor, 
    PinConfig, 
    MicrostepConfig, 
    MicrostepPinConfig,
)
from pyberryplc.stepper.uart.tmc2208_uart import TMC2208UART
from pyberryplc.stepper.uart.tmc2208_registers import IHOLDIRUNRegister


class TMC2208StepperMotor(StepperMotor):
    """
    Stepper motor controlled via a TMC2208 driver using either GPIO or UART.
    """
    MICROSTEP_GPIO_CFG: dict[str, tuple[int, int]] = {
        "1/2": (1, 0),
        "1/4": (0, 1),
        "1/8": (0, 0),
        "1/16": (1, 1),
    }
    
    MICROSTEP_UART_CFG: dict[str, int] = {
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
        pin_config: PinConfig,
        logger: logging.Logger | None = None,
        name: str = "",
        uart: TMC2208UART | None = None
    ) -> None:
        """
        Initialize a stepper motor controlled by a TMC2208 driver.
        
        Parameters
        ----------
        pin_config : PinConfig
            Configuration for essential GPIO pins of the stepper motor driver.
        logger : logging.Logger
            Logger for analysis and diagnosis of the stepper motor driver 
            operation.
        name : str, optional
            Name to identify the stepper motor driver.
        uart : TMC2208UART, optional
            UART interface for register-level control of the driver.
        """
        super().__init__(pin_config, logger, name)
        self.uart = uart
    
    def configure_microstepping(
        self, 
        resolution: str,
        ms_pins: MicrostepPinConfig | None = None,
        full_steps_per_rev: int = 200
    ) -> None:
        """Configures microstepping on the TMC2208 driver.
        
        Parameters
        ----------
        resolution : str
            Desired microstep resolution (e.g., "full", "1/8", "1/16").
        ms_pins : MicrostepPinConfig, optional
            GPIO pin configuration for MS1, MS2, and MS3. Defaults to None.
            If None, microstepping is configured via UART. If UART is enabled,
            configuration of microstepping via UART takes precedence over GPIO. 
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
        if ms_pins is not None and self.uart is None:
            self.microstep_config = MicrostepConfig(supported=set(self.MICROSTEP_GPIO_CFG.keys()))
            self._configure_gpio_microstepping(resolution, ms_pins, full_steps_per_rev)
        elif self.uart is not None:
            self.microstep_config = MicrostepConfig(supported=set(self.MICROSTEP_UART_CFG.keys()))
            self._configure_uart_microstepping(resolution, full_steps_per_rev)
        else:
            raise RuntimeError("Microstep pin configuration is missing.")
    
    def _configure_gpio_microstepping(
        self,
        resolution: str,
        ms_pins: MicrostepPinConfig,
        full_steps_per_rev: int
    ) -> None:
        """Configures microstepping on the TMC2208 driver via GPIO."""
        super().configure_microstepping(resolution, ms_pins, full_steps_per_rev)
        ms1 = self.microstep_config.pin_config.ms1
        ms2 = self.microstep_config.pin_config.ms2
        mres = self.microstep_config.resolution
        if ms1 and ms2:
            ms1_val, ms2_val = self.MICROSTEP_GPIO_CFG[mres]
            ms1.write(ms1_val)
            ms2.write(ms2_val)
            self.logger.info(
                f"[{self.name}] Microstepping set to {mres} "
                f"(MS1={ms1_val}, MS2={ms2_val})"
            )
        else:
            self.logger.warning(
                f"[{self.name}] MS1/MS2 pins not configured, "
                f"skipping microstepping setup"
            )
    
    def _configure_uart_microstepping(
        self,
        resolution: str,
        full_steps_per_rev: int
    ) -> None:
        """Configures microstepping on the TMC2208 driver via UART."""
        super().configure_microstepping(resolution, None, full_steps_per_rev)
        mres = self.MICROSTEP_UART_CFG[self.microstep_config.resolution]
        self.uart.update_register("CHOPCONF", {"mres": mres})
        self.logger.info(
            f"[{self.name}] Setting microstepping via UART: "
            f"{self.microstep_config.resolution} (mres = {mres})"
        )
    
    def enable(self, high_sensitivity: bool = False) -> None:
        """
        Enables the stepper driver.

        If UART is configured, this method opens the UART connection and
        configures the driver to accept software microstepping settings
        and activate current output. Otherwise, it falls back to GPIO-based
        enabling.
        """
        if self.uart is not None:
            self._enable_via_uart(high_sensitivity)
            self.logger.info(f"[{self.name}] Driver enabled")
        else:
            super().enable()
    
    def _enable_via_uart(self, high_sensitivity) -> None:
        self.uart.open()
        time.sleep(0.005)
        self.uart.update_register(
            reg_name="GCONF",
            fields={
                "pdn_disable": True,      # PDN_UART input function disabled. 
                "mstep_reg_select": True  # microstep resolution selected by MSTEP register
            }
        )
        time.sleep(0.005)
        self.uart.update_register(
            reg_name="CHOPCONF",
            fields={
                "toff": 3,  # Enable driver - off time setting controls duration of slow decay phase
                "vsense": high_sensitivity,
            }
        )
    
    def disable(self) -> None:
        """
        Disables the stepper driver.

        If UART is configured, this method turns off the driver's current
        output (via CHOPCONF) and closes the UART connection. Otherwise,
        it disables the driver through the GPIO enable pin.
        """
        if self.uart is not None:
            self._disable_via_uart()
            self.logger.info(f"[{self.name}] Driver disabled")
        else:
            super().disable()
    
    def _disable_via_uart(self) -> None:
        self.uart.update_register(
            reg_name="CHOPCONF",
            fields={"toff": 0}  # Driver disable, all bridges off, motor is freewheeling.
        )
        self.uart.close()
        
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
            To determine a safe `run_current_pct` value in order to limit the 
            motor run current to its rated value, there is a utility-function 
            `calculate_run_current_pct()` in `/utils/tmc_utils.py`.
        hold_current_pct : float
            Hold current as a percentage (0–100). This could be a smaller 
            percentage than `run_current_pct`, but will also depend on the
            holding torque that may be required in the specific application.
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
            raise RuntimeError("UART interface not available on this stepper motor.")

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
            f"[{self.name}] UART current config set: IRUN={irun}/31, "
            f"IHOLD={ihold}/31, DELAY={ihold_delay}"
        )
