import time
from typing import Any, Callable, TypeVar
import multiprocessing
from multiprocessing.connection import Connection

from pyberryplc.motion import MotionProfile
from .base import StepperMotor


TStepperMotor = TypeVar("TStepperMotor", bound=StepperMotor)


class StepperMotorProcess(multiprocessing.Process):
    """
    A motor control process that receives motion profile commands from a master
    process and executes them using a `StepperMotor` instance.

    This allows each motor to run in a dedicated OS-level process, enabling
    parallel execution of blocking motor movements such as `rotate_profile()`
    without interference from the main PLC thread or other motor processes.

    Parameters
    ----------
    conn : multiprocessing.Connection
        Duplex connection to receive commands and send status updates.
    motor_class : type[StepperMotor]
        Concrete subclass of StepperMotor to instantiate.
    motor_kwargs : dict[str, Any]
        Keyword arguments passed to the motor_class constructor.
    config_callback : Callable[[StepperMotor], None], optional
        Function to apply configuration on the `StepperMotor` instance after 
        instantiation (e.g., set microstepping, current limits).
    name : str, optional
        Identifier for logging and debugging purposes.
    """

    def __init__(
        self,
        conn: Connection,
        motor_class: type[StepperMotor],
        motor_kwargs: dict[str, Any],
        config_callback: Callable[[TStepperMotor], None] | None = None,
        name: str = "StepperProcess"
    ) -> None:
        super().__init__(daemon=True)
        self.conn = conn
        self.motor_class = motor_class
        self.motor_kwargs = motor_kwargs
        self.config_callback = config_callback
        self.name = name

    def run(self) -> None:
        """Main loop of the motor process. Waits for commands and executes them."""
        motor = self.motor_class(**self.motor_kwargs)
        motor.enable()
        
        if self.config_callback:
            self.config_callback(motor)
        
        prepared = False
        
        while True:
            try:
                msg = self.conn.recv()
            except EOFError:
                break

            if not isinstance(msg, dict):
                continue

            cmd = msg.get("cmd")

            if cmd == "prepare_profile":
                profile: MotionProfile = msg["profile"]
                direction: str = msg.get("direction", "forward")

                motor.calculate_rotation_profile(profile, direction)
                self.conn.send({"status": "ready", "name": self.name})
                prepared = True

            elif cmd == "start" and prepared:
                start_time: float | None = msg.get("start_time")
                if start_time is not None:
                    now = time.perf_counter()
                    wait_time = start_time - now
                    if wait_time > 0:
                        time.sleep(wait_time)
                
                motor.start_rotation_profile()
                
                while motor.busy:
                    time.sleep(0.005)

                self.conn.send({"status": "done", "name": self.name})
                prepared = False

            elif cmd == "shutdown":
                motor.disable()
                break

            else:
                self.conn.send({"status": "error", "message": f"Unknown command: {cmd}"})
