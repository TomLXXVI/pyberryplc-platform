import logging

from pyberryplc.stepper.driver.base import (
    StepperMotor, 
    PinConfig, 
    MicrostepPinConfig, 
    MicrostepConfig
)


class A4988StepperMotor(StepperMotor):
    """
    Stepper motor driver class for the A4988 chip.

    This driver supports GPIO-based microstepping using MS1, MS2, and MS3 pins.
    """
    MICROSTEP_GPIO_CFG: dict[str, tuple[int, int, int]] = {
        "full": (0, 0, 0),
        "1/2": (1, 0, 0),
        "1/4": (0, 1, 0),
        "1/8": (1, 1, 0),
        "1/16": (1, 1, 1),
    }
    
    def __init__(
        self,
        pin_config: PinConfig,
        logger: logging.Logger | None = None,
        name: str = ""
    ) -> None:
        super().__init__(pin_config, logger, name)
        self.microstep_config = MicrostepConfig(supported=set(self.MICROSTEP_GPIO_CFG.keys()))

    def configure_microstepping(
        self, 
        resolution: str = "1/16",
        ms_pins: MicrostepPinConfig | None = None,
        full_steps_per_rev: int = 200
    ) -> None:
        """
        Configure microstepping on the A4988 driver.
        """
        super().configure_microstepping(resolution, ms_pins, full_steps_per_rev)
        ms1 = self.microstep_config.pin_config.ms1
        ms2 = self.microstep_config.pin_config.ms2
        ms3 = self.microstep_config.pin_config.ms3
        mres = self.microstep_config.resolution
        if ms1 and ms2 and ms3:
            ms1_val, ms2_val, ms3_val = self.MICROSTEP_GPIO_CFG[mres]
            ms1.write(ms1_val)
            ms2.write(ms2_val)
            ms3.write(ms3_val)
            self.logger.info(
                f"[{self.name}] Microstepping set to {mres} "
                f"(MS1={ms1_val}, MS2={ms2_val}, MS3={ms3_val})"
            )
        else:
            self.logger.warning(
                f"[{self.name}] "
                f"MS1/MS2/MS3 pins not configured, skipping microstepping setup"
            )
