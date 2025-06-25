"""
PLC program for the RATTMMOTOR CNC 1610 Pro using MKS TMC2208 V2.0 drivers for
the stepper motors with UART-interface.
-------------------------------------------------------------------------------
Only the stepper motors of the RATTMMOTOR CNC 1610 Pro are used (and only X and 
Y in this demo). These are connected to external MKS TMC2208 V2.0 drivers which
are controlled by the Raspberry Pi using the `pyberryplc` package. 

The PLC program has two operating modes: either 'manual' or 'auto'.
The operating mode is passed to the constructor of the PLC app `XYPlcApp`.
If the operating mode is 'manual', the subroutine in package `manual_mode` is
loaded. If the operating mode is 'auto', the subroutine in package 
`automatic_mode` is loaded.
The configuration of the stepper motors is done with a TOML configuration file
(`motor_config.toml` in the same directory as `main.py`), which is passed to
the constructor of the PLC app.
In 'auto' mode, the path to a JSON trajectory file must be given 
(`trajectory01.json` in the same directory as `main.py`).

The PLC app uses third-party package `keyboard` to detect key presses on the
keyboard of the Raspberry Pi. This package requires the user to be a 
"sudo user". For this a bash script is provided (`run_with_keyboard.sh`). To
launch the PLC app from the command line of the console, enter 
`source run_with_keyboard.sh`, and to be supplemented with the necessary 
arguments. For example, to run the PLC app in 'manual' mode, one can enter:
```
source run_with_keyboard.sh manual
```
This will launch the PLC app in 'manual' mode, and it will use the default 
`motor_config.toml` to configure the stepper motor drivers. 
To run the PLC app in 'auto' mode, one can enter for example:
```
source run_with_keyboard.sh auto --prg trajectory01.json
```
This will launch the PLC app in 'auto' mode, use the default `motor_config.toml` 
to configure the stepper motor drivers, and load `trajectory01.json` to be
executed within the automatic-mode subroutine (see module `subroutine.py` in
package `automatic_mode`).
Should a different motor configuration file be used (i.e. different than 
`motor_config.toml` in the same directory as `main.py`), one must also add 
`--cfg <path to the TOML config file>` to the command line.

Notes
-----
Trajectory JSON files are created with a `TrajectoryPlanner` instance 
(see `pyberryplc.motion.trajectory.py`). An example of this can be found in
`demos/demo_stepper_motion/motion_planner.py`.
"""
import argparse
import os

from pyberryplc.core import AbstractPLC
from pyberryplc.utils.log_utils import init_logger

from manual_mode import ManualModeSubRoutine
from automatic_mode import AutomaticModeSubRoutine


class XYPlcApp(AbstractPLC):
    
    def __init__(
        self, 
        cfg_filepath: str = "motor_config.toml", 
        prg_filepath: str | None = None,
        mode: str = "manual"
    ) -> None:
        if mode not in ("manual", "auto"):
            raise ValueError(
                f"Unknown operation mode: {mode}. "
                "Must be either 'manual' or 'auto'."
            )
        if mode == "auto" and prg_filepath is None:
            raise ValueError(
                "`prg_filepath` cannot be `None` when `mode` is 'auto'."
            )
        super().__init__(logger=init_logger("XYPlcApp"))
        self._subroutine = None
        if mode == "auto":
            self._subroutine = AutomaticModeSubRoutine(
                main_routine=self,
                cfg_filepath=cfg_filepath,
                prg_filepath=prg_filepath
            )
        elif mode == "manual":
            self._subroutine = ManualModeSubRoutine(
                main_routine=self,
                cfg_filepath=cfg_filepath
            )
    
    def control_routine(self) -> None:
        self._subroutine.call()
    
    def exit_routine(self) -> None:
        self._subroutine.exit()
    
    def emergency_routine(self) -> None:
        self.exit_routine()
    
    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        self.exit_routine()
        raise exception


def parse_args():
    parser = argparse.ArgumentParser(description="Start the XY PLC application.")

    parser.add_argument(
        "mode", 
        choices=["manual", "auto"],
        help="Operating mode: 'manual' or 'auto'"
    )
    parser.add_argument(
        "--cfg", 
        dest="cfg_filepath", 
        default="motor_config.toml",
        help="Path to the motor configuration file (default: motor_config.toml)"
    )
    parser.add_argument(
        "--prg", 
        dest="prg_filepath",
        help="Path to the motion program file (required in 'auto' mode)"
    )

    args = parser.parse_args()

    if args.mode == "auto" and not args.prg_filepath:
        parser.error("The --prg argument is required when mode is 'auto'.")

    return args


if __name__ == "__main__":
    os.system("clear")

    args = parse_args()

    plc = XYPlcApp(
        cfg_filepath=args.cfg_filepath,
        prg_filepath=args.prg_filepath,
        mode=args.mode
    )
    plc.run()
