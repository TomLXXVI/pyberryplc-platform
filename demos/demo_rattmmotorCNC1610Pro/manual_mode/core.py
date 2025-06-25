from typing import Any
import logging
from pathlib import Path
import tomllib

from pyberryplc.stepper import TMC2208StepperMotor, PinConfig, TMC2208UART, RotatorType
from pyberryplc.motion import RotationDirection, TrapezoidalProfile


def create_dynamic_stepper_motor(
    name: str, 
    cfg: dict[str, Any],
    logger: logging.Logger
) -> TMC2208StepperMotor:
    """
    Creates a `TMC2208StepperMotor` instance. The `TMC2208StepperMotor` instance
    is equipped with a `DynamicRotatorThreaded` rotator
    (`RotatorType.DYNAMIC_THREADED`). This allows to start the stepper motor
    by pressing a start button, the motor will keep rotating until a stop
    button is pressed.
    """
    stepper = TMC2208StepperMotor(
        pin_config=PinConfig(
            step_pin_ID=cfg["step_pin_ID"],
            dir_pin_ID=cfg["dir_pin_ID"]
        ),
        logger=logger,
        name=name,
        uart=TMC2208UART(port=cfg["comm_port"])
    )
    stepper.attach_rotator(RotatorType.DYNAMIC_THREADED)
    stepper.rotator.direction = RotationDirection.COUNTERCLOCKWISE
    stepper.rotator.profile = TrapezoidalProfile(
        ds_tot=720.0,  # deg
        a_m=1000.0,    # deg/s2
        v_m=360.0,     # deg/s
        v_i=0.0,
        v_f=0.0
    )
    return stepper


def configure_stepper_motor(
    stepper: TMC2208StepperMotor, 
    cfg: dict[str, Any]
) -> None:
    """
    Enables the stepper motor driver, configures microstepping, and sets the
    allowable motor run and hold current.
    """
    stepper.enable()
    stepper.configure_microstepping(
        resolution=cfg["microstepping"]["resolution"], 
        full_steps_per_rev=cfg["microstepping"]["full_steps_per_rev"]
    )
    stepper.set_current_via_uart(
        run_current_pct=cfg["current"]["run_current_pct"], 
        hold_current_pct=cfg["current"]["hold_current_pct"]
    )


def load_motor_configurations(
    filepath: str
) -> tuple[Any | None, ...]:
    """
    Loads the motor configuration TOML file and assigns the configuration 
    of each motor to its own configuration-dict. If a motor is missing, its
    configuration-dict will be set to `None`. Returns the configuration-dicts
    in the order of x-, y-, and z-motor.
    """
    with Path(filepath).open("rb") as f:
        config = tomllib.load(f)
        x_motor_cfg = config.get("x_motor")
        y_motor_cfg = config.get("y_motor")
        z_motor_cfg = config.get("z_motor")
    return x_motor_cfg, y_motor_cfg, z_motor_cfg
