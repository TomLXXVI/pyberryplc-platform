import logging
from pyberryplc.core.gpio import DigitalOutput
from pyberryplc.stepper.driver.base import StepperMotor


class A4988StepperMotor(StepperMotor):
    """
    Stepper motor controlled via an A4988 driver and GPIO.
    Supports microstepping configuration via MS1, MS2, and MS3 pins.
    """

    MICROSTEP_CONFIG_GPIO: dict[str, tuple[int, int, int]] = {
        "full":  (0, 0, 0),
        "1/2":   (1, 0, 0),
        "1/4":   (0, 1, 0),
        "1/8":   (1, 1, 0),
        "1/16":  (1, 1, 1),
    }
    
    def __init__(
        self,
        step_pin: int,
        dir_pin: int,
        enable_pin: int | None = None,
        ms1_pin: int | None = None,
        ms2_pin: int | None = None,
        ms3_pin: int | None = None,
        full_steps_per_rev: int = 200,
        microstep_resolution: str = "full",
        logger: logging.Logger | None = None
    ) -> None:
        """
        Initialize an A4988 stepper motor driver instance.

        Parameters
        ----------
        step_pin : int
            GPIO pin connected to the STEP input of the driver.
        dir_pin : int
            GPIO pin connected to the DIR input of the driver.
        enable_pin : int | None, optional
            GPIO pin connected to the EN input of the driver (active low).
        ms1_pin : int | None, optional
            GPIO pin connected to MS1 for microstepping control.
        ms2_pin : int | None, optional
            GPIO pin connected to MS2 for microstepping control.
        ms3_pin : int | None, optional
            GPIO pin connected to MS3 for microstepping control.
        full_steps_per_rev : int, optional
            Number of full steps per motor revolution. Default is 200.
        microstep_resolution : str, optional
            The microstep resolution to be used. Default is full-step mode.
            Valid microstep resolutions are defined in class attribute 
            MICROSTEP_FACTORS of the base class. However, it is possible that 
            the actual driver does not support all of these. This should be 
            checked in advance.
        logger : logging.Logger | None, optional
            Logger instance for debug/info output.
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
        self.ms3 = (
            DigitalOutput(ms3_pin, label="MS3")
            if ms3_pin is not None
            else None
        )
        super().__init__(
            step_pin, dir_pin, enable_pin, 
            full_steps_per_rev, microstep_resolution, 
            logger
        )
    
    def _validate_microstepping(self, microstep_resolution: str) -> int:
        """
        Checks whether the microstep resolution is valid for the A4988 driver. 
        
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
            If the microstep resolution is unavailable on the A4988 driver.
        """
        valid_resolutions = list(self.MICROSTEP_CONFIG_GPIO.keys())
        if microstep_resolution in valid_resolutions:
            return self.MICROSTEP_FACTORS[microstep_resolution]
        else:
            raise ValueError(
                f"Microstep resolution '{microstep_resolution}' is "
                f"unavailable on this driver. "
                f"Available: {valid_resolutions}"
            )
    
    def set_microstepping(self) -> None:
        """
        Configure microstepping on the A4988 driver.
        """
        if self.ms1 and self.ms2 and self.ms3:
            ms1_val, ms2_val, ms3_val = self.MICROSTEP_CONFIG_GPIO[self.microstep_resolution]
            self.ms1.write(ms1_val)
            self.ms2.write(ms2_val)
            self.ms3.write(ms3_val)
            self.logger.info(
                f"Microstepping set to {self.microstep_resolution} "
                f"(MS1={ms1_val}, MS2={ms2_val}, MS3={ms3_val})"
            )
        else:
            self.logger.warning(
                f"MS1/MS2/MS3 pins not configured, skipping microstepping setup"
            )
