from typing import Any, Callable
import multiprocessing
# noinspection PyProtectedMember
from multiprocessing.connection import Connection

from pyberryplc.motion import MotionProfile, RotationDirection
from pyberryplc.stepper.driver.base import (
    StepperMotor, 
    TwoStageMotionProfileRotator,
    DynamicRotatorThreaded,
    TStepperMotor
)


class MPMCProcess(multiprocessing.Process):
    """
    MPMC stands for "Motion Profile Motor Control".
    
    Motor control process that receives a single motion profile from a master 
    process, processes that motion profile internally (i.e. creating the step 
    pulse time delays to drive the stepper motor) and executes the rotation by 
    creating a concrete `StepperMotor` instance inside the process.

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
        Function to configure the `StepperMotor` instance after its 
        instantiation (e.g., set microstepping, current limits).
    motor_name : str, optional
        Identifier for logging and debugging purposes.
    """
    def __init__(
        self,
        conn: Connection,
        motor_class: type[StepperMotor],
        motor_kwargs: dict[str, Any],
        config_callback: Callable[[TStepperMotor], None] | None = None,
        motor_name: str = "MPMC"
    ) -> None:
        super().__init__(daemon=True)
        self.conn = conn
        self.motor_class = motor_class
        self.motor_kwargs = motor_kwargs
        self.config_callback = config_callback
        self.motor_name = motor_name

    def run(self) -> None:
        """
        Main loop of the motor process. Waits for commands and executes them.
        """
        # Instantiate the stepper motor and its rotator.
        motor = self.motor_class(**self.motor_kwargs)
        motor.rotator = TwoStageMotionProfileRotator(motor)
        
        # Enable the motor driver and set the motor driver configuration 
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
                "name": self.motor_name,
                "message": error_msg
            })
            return
        
        prepared = False
        msg: str = ""
        
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
                direction: str = msg.get("direction", "ccw")
                if direction == "ccw":
                    motor.rotator.preprocess(RotationDirection.CCW, profile)
                else:
                    motor.rotator.preprocess(RotationDirection.CW, profile)
                self.conn.send({
                    "status": "ready", 
                    "name": self.motor_name
                })
                prepared = True

            elif cmd == "start" and prepared:
                motor.rotator.rotate()
                self.conn.send({
                    "status": "done", 
                    "name": self.motor_name,
                    "travel_time": motor.rotator.travel_time
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


class SPMCProcess(multiprocessing.Process):
    """
    SPMC stands for "Step Pulse Motor Control".
    
    This motor control process receives the step pulse time delays to drive the 
    stepper motor from an external process. This requires or allows that the 
    processing of motion profiles must have been done or can be done in advance.
    
    A concrete `StepperMotor` instance is only created when the process is 
    started. The specific configuration settings of the `StepperMotor` instance 
    are passed using a callback-function which must be passed to the constructor
    of the `SPMCProcess` and which will be called by the `SPMCProcess` when the 
    process is started.
    
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
    motor_name : str, optional
        Process identifier for logging and debugging purposes.
    
    Communication messages between the external process and the `SPMCProcess` 
    are dictionaries. The external process sends commands to the `SPMCProcess`
    and receives status feedback messages back from the `SPMCProcess`. 
    
    The `SPMCProcess` understands the following commands:
    
    1.  Command to execute a movement of the axis.
        ```
        {"cmd": "start_motion", "delays": <list[float]>, "direction": <RotationDirection>}
        ```
        - Key "delays" must carry a list with the time delays between the step pulses.
        - Key "direction" must indicate the rotation direction of the axis.

    2.  Command to turn jog mode on.
        ```
        {"cmd": "start_jog", "direction": <RotationDirection>}
        ```

    3.  Command to turn jog mode off.
        ```
        {"cmd": "stop_jog"}
        ```

    4.  Command to shutdown the process.
        ```
        {"cmd": "shutdown"}
        ```
        
    The `SPMCProcess` returns the following status feedback messages back to the
    external process:
    
    1.  In response to command "start_motion" it returns a status feedback
        message to signal that the axis movement is finished.
        ```
        {"status": "motion_done", "name": <str>, "travel_time": <float>}
        ```
        - Key "name" contains the motor name given to the `SPMCProcess`.
        - Key "travel_time" returns the travel time of the executed movement.

    2.  In response to command "start_jog":
        ```
        {"status": "jog_started", "name": <str>, "message": <str>}
        ```

    3.  In response to command "stop_jog":
        ```
        {"status": "jog_stopped", "name": <str>, "message": <str>}
        ```

    4.  If an exception is raised when instantiating or configuring the
        `StepperMotor` instance, the following status feedback message is sent:
        ```
        {"status": "startup_error", "name": <str>, "message": <str>}
        ```
        - Key "message" contains the type of exception and its description.
    
    5.  If an unknown command is send, the `SPMCProcess` responds with:
        ```
        {"status": "error", "message": f"Unknown command: {cmd}"}
        ```
    """
    def __init__(
        self,
        conn: Connection,
        motor_class: type[StepperMotor],
        motor_kwargs: dict[str, Any],
        config_callback: Callable[[TStepperMotor], None] | None = None,
        motor_name: str = "_",
        jog_mode_profile: MotionProfile | None = None
    ) -> None:
        super().__init__(daemon=True)
        self.conn = conn
        self.motor_class = motor_class
        self.motor_kwargs = motor_kwargs
        self.config_callback = config_callback
        self.motor_name = motor_name
        self.jog_mode_profile = jog_mode_profile
        self._jog_mode_active: bool = False

    def run(self) -> None:
        """
        Runs the motor process. First, a motor is instantiated and configured, 
        then the message loop is entered, waiting for commands to execute.
        """
        # Instantiate the stepper motor and its rotators.
        motor = self.motor_class(**self.motor_kwargs)
        motion_profile_rotator = TwoStageMotionProfileRotator(motor)
        dynamic_rotator = DynamicRotatorThreaded(motor)

        # Enable motor driver and set motor driver settings on the driver 
        # through a callback function.
        try:
            if self.config_callback is not None:
                self.config_callback(motor)
            else:
                motor.enable()
        except Exception as e:
            error_msg = f"[{self.motor_name}] {type(e).__name__}: {e}"
            motor.logger.error(error_msg)
            self.conn.send({
                "status": "startup_error",
                "name": self.motor_name,
                "message": error_msg
            })
            return
        
        msg: str = ""

        # Enter the message loop.
        while True:
            try:
                msg = self.conn.recv()
            except EOFError:
                break

            if not isinstance(msg, dict):
                continue

            cmd = msg.get("cmd")

            if cmd == "start_motion":
                self._jog_mode_active = False
                motor.rotator = motion_profile_rotator
                motor.rotator.delays = msg.get("delays", None)
                motor.rotator.direction = msg.get("direction", RotationDirection.CCW)
                if motor.rotator.delays is not None:
                    motor.rotator.rotate()
                    self.conn.send({
                        "status": "motion_done",
                        "name": self.motor_name,
                        "travel_time": motor.rotator.travel_time
                    })

            elif cmd == "start_jog":
                if self.jog_mode_profile is not None:
                    self._jog_mode_active = True
                    motor.rotator = dynamic_rotator
                    motor.rotator.profile = self.jog_mode_profile
                    motor.rotator.direction = msg.get("direction", RotationDirection.CCW)
                    motor.rotator.start()
                    self.conn.send({
                        "status": "jog_started",
                        "name": self.motor_name,
                        "message": "jog mode turned on"
                    })

            elif cmd == "stop_jog":
                if isinstance(motor.rotator, DynamicRotatorThreaded) and self._jog_mode_active:
                    motor.rotator.stop()
                    self._jog_mode_active = False
                    self.conn.send({
                        "status": "jog_stopped",
                        "name": self.motor_name,
                        "message": "jog mode turned off"
                    })

            elif cmd == "set_jog_mode_profile_args":
                args = msg.get("args")
                if args is not None and self.jog_mode_profile is not None:
                    self.jog_mode_profile = type(self.jog_mode_profile)(**args)

            elif cmd == "shutdown":
                motor.disable()
                break

            else:
                self.conn.send({
                    "status": "error",
                    "name": self.motor_name,
                    "message": f"Unknown command: {cmd}"
                })
