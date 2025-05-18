from typing import Any, Callable, TypeVar
import multiprocessing
# noinspection PyProtectedMember
from multiprocessing.connection import Connection

from pyberryplc.motion.multi_axis import MotionProfile
from .base import StepperMotor, ProfileRotator, Direction


TStepperMotor = TypeVar("TStepperMotor", bound=StepperMotor)


class TwoStageProfileRotator(ProfileRotator):
    """
    Refactored `ProfileRotator` class that splits the processing of a motion 
    profile and the commanding of a rotation movement into two separate 
    functions.
    """
    def preprocess(self, direction: Direction, profile: MotionProfile) -> None:
        """Calculates the delays between successive step pulses that drive the 
        stepper motor from the given motion profile..
        """
        self.direction = direction
        self.profile = profile
        self._generate_delays()

    def rotate(self) -> None:
        """Commands the rotation of the stepper motor."""
        self._step_loop()
    
    @property
    def delays(self) -> list[float]:
        """Returns the delays between successive step pulses after internal
        preprocessing of the motion profile.
        """
        return self._delays
    
    @delays.setter
    def delays(self, value: list[float]) -> None:
        """Sets the delays between successive step pulses after external 
        preprocessing of the motion profile.
        """
        self._delays = value


class SingleMotionProcess(multiprocessing.Process):
    """
    Motor control process that receives a motion profile from a master process, 
    processes the motion profile and executes it using a `StepperMotor` 
    instance.

    Using a separate process to drive a stepper motor, allows each motor to run 
    in a dedicated OS-level process, enabling parallel execution of blocking 
    motor movements without interference from the main PLC thread or other motor
    processes.

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
        """
        Main loop of the motor process. Waits for commands and executes them.
        """
        # Instantiate stepper motor and rotator.
        motor = self.motor_class(**self.motor_kwargs)
        motor.rotator = TwoStageProfileRotator(motor)
        
        # Enable motor driver and set motor driver settings on the driver 
        # through a callback function.
        try:
            if self.config_callback is not None:
                self.config_callback(motor)
            else:
                motor.enable()
        except Exception as e:
            error_msg = f"{type(e).__name__}: {e}"
            motor.logger.error(error_msg)
            self.conn.send({
                "status": "startup_error",
                "name": self.name,
                "message": error_msg
            })
            return
        
        prepared = False
        
        # Message loop
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
                if direction == "forward":
                    motor.rotator.preprocess(Direction.COUNTERCLOCKWISE, profile)
                else:
                    motor.rotator.preprocess(Direction.CLOCKWISE, profile)
                self.conn.send({
                    "status": "ready", 
                    "name": self.name
                })
                prepared = True

            elif cmd == "start" and prepared:
                motor.rotator.rotate()
                self.conn.send({
                    "status": "done", 
                    "name": self.name, 
                    "travel_time": motor.rotator.moving_time
                })
                prepared = False

            elif cmd == "shutdown":
                motor.disable()
                break

            else:
                self.conn.send({
                    "status": "error", 
                    "message": f"Unknown command: {cmd}"
                })


class MultiMotionProcess(multiprocessing.Process):
    """
    Motor control process that directly receives a step pulse train to drive 
    the stepper motor. The preprocessing of the motion profile must have been 
    done in advance.

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
        """
        Main loop of the motor process. Waits for commands and executes them.
        """
        # Instantiate stepper motor and rotator.
        motor = self.motor_class(**self.motor_kwargs)
        motor.rotator = TwoStageProfileRotator(motor)

        # Enable motor driver and set motor driver settings on the driver 
        # through a callback function.
        try:
            if self.config_callback is not None:
                self.config_callback(motor)
            else:
                motor.enable()
        except Exception as e:
            error_msg = f"{type(e).__name__}: {e}"
            motor.logger.error(error_msg)
            self.conn.send({
                "status": "startup_error",
                "name": self.name,
                "message": error_msg
            })
            return
        
        # Message loop.
        while True:
            try:
                msg = self.conn.recv()
            except EOFError:
                break

            if not isinstance(msg, dict):
                continue

            cmd = msg.get("cmd")

            if cmd == "start":
                motor.rotator.delays = msg.get("delays", None)
                motor.rotator.direction = msg.get("direction", Direction.COUNTERCLOCKWISE)
                if motor.rotator.delays is not None:
                    motor.rotator.rotate()
                    self.conn.send({
                        "status": "done", 
                        "name": self.name, 
                        "travel_time": motor.rotator.moving_time
                    })

            elif cmd == "shutdown":
                motor.disable()
                break

            else:
                self.conn.send({
                    "status": "error", 
                    "message": f"Unknown command: {cmd}"
                })
