from typing import Callable, Type, Any, TypeVar
from logging import Logger
from pyberryplc.core import AbstractPLC, SharedData, CounterUp
from pyberryplc.motion.multi_axis import ProfileType
from pyberryplc.motion.trajectory import TrajectoryPlanner, Trajectory

from multiprocessing import Pipe
from pyberryplc.stepper import (
    StepperMotor,
    TMC2208StepperMotor,
    TMC2208UART,
    PinConfig,
    TrajectoryProcess
)


TStepperMotor = TypeVar("TStepperMotor", bound=StepperMotor)


class MotorController:
    
    def __init__(
        self,
        motor_name: str,
        motor_class: Type[StepperMotor],
        pin_config: PinConfig,
        cfg_callback: Callable[[TStepperMotor], None] | None = None,
        comm_port: str = "",
        logger: Logger | None = None,
    ) -> None:
        self.motor_name = motor_name
        self.motor_class = motor_class
        self.pin_config = pin_config
        self.cfg_callback = cfg_callback
        self.comm_port = comm_port
        self.logger = logger
        self.interface, _motor_proc = Pipe()

        motor_kwargs: dict[str, Any] = {
            "pin_config": self.pin_config,
            "logger": logger,
            "name": self.motor_name,
        }
        if issubclass(self.motor_class, TMC2208StepperMotor) and self.comm_port:
            motor_kwargs["uart"] = TMC2208UART(port=self.comm_port)
                
        self._motor_proc = TrajectoryProcess(
            conn=_motor_proc,
            motor_class=self.motor_class,
            motor_kwargs=motor_kwargs,
            config_callback=self.cfg_callback,
            name=self.motor_name
        )
    
    def enable(self) -> None:
        self._motor_proc.start()
    
    def disable(self) -> None:
        self._motor_proc.join()
    
    def update_status(self, callback: Callable[[dict[str, Any]], None]) -> None:
        if self.interface.poll():
            msg = self.interface.recv()
            callback(msg)
    

class XYMotionPLC(AbstractPLC):
    
    def __init__(self, shared_data: SharedData, logger: Logger):
        super().__init__(shared_data=shared_data, logger=logger)
        
        # Setup of X-motor controller
        def _config_x_motor(motor: TMC2208StepperMotor) -> None:
            motor.enable(high_sensitivity=True)
            motor.configure_microstepping(
                resolution="full",
                ms_pins=None,
                full_steps_per_rev=200
            )
            motor.set_current_via_uart(
                run_current_pct=35.0,
                hold_current_pct=10.0
            )
        
        self.x_motor_controller = MotorController(
            motor_name="Motor X",
            motor_class=TMC2208StepperMotor,
            pin_config=PinConfig(step_pin_ID=21, dir_pin_ID=27, use_pigpio=True),
            cfg_callback=_config_x_motor,
            comm_port="/dev/ttyUSB1",
            logger=self.logger
        )
        self.x_interface = self.x_motor_controller.interface
        
        # Setup of Y-motor controller
        def _config_y_motor(motor: TMC2208StepperMotor) -> None:
            motor.enable(high_sensitivity=True)
            motor.configure_microstepping(
                resolution="full",
                ms_pins=None,
                full_steps_per_rev=200
            )
            motor.set_current_via_uart(
                run_current_pct=35.0,
                hold_current_pct=10.0
            )

        self.y_motor_controller = MotorController(
            motor_name="Motor Y",
            motor_class=TMC2208StepperMotor,
            pin_config=PinConfig(step_pin_ID=19, dir_pin_ID=20, use_pigpio=True),
            cfg_callback=_config_y_motor,
            comm_port="/dev/ttyUSB0",
            logger=self.logger
        )
        self.y_interface = self.y_motor_controller.interface
        
        # Set up trajectory planner
        self.trajectory_planner = TrajectoryPlanner(
            pitch=100,
            motor_speed=180.0,
            motor_accel=360.0,
            full_steps_per_rev=200,
            microstep_factor=1,
            profile_type=ProfileType.S_CURVED
        )
        self.trajectory: Trajectory | None = None
      
        # Internal markers
        self.X0 = self.add_marker("X0")
        self.X1 = self.add_marker("X1")
        self.X2 = self.add_marker("X2")
        
        self.trajectory_ready = self.add_marker("ready", init_value=False)
        
        # HMI-inputs
        self.start_motion = self.hmi_input_register["start_motion"]
        
        # HMI-outputs (status feedback)
        self.x_motor_busy = self.hmi_output_register["x_motor_busy"]
        self.y_motor_busy = self.hmi_output_register["y_motor_busy"]
        self.travel_time_x = self.hmi_output_register["travel_time_x"]
        self.travel_time_y = self.hmi_output_register["travel_time_y"]
        self.x_motor_ready = self.hmi_output_register["x_motor_ready"]
        self.y_motor_ready = self.hmi_output_register["y_motor_ready"]
        
        # Counters
        self.segment_counter = CounterUp(preset_val=1)
        
        # Internal data variables
        self._init_flag = False
        self.num_segments = 0

    def _init_control(self):
        if not self._init_flag:
            self._init_flag = True

            self.x_motor_controller.enable()
            self.y_motor_controller.enable()

            self.X0.activate()
    
    def _sequence_control(self):
        # Get feedback from motor controllers
        self.x_motor_controller.update_status(self._process_status_x_motor)
        self.y_motor_controller.update_status(self._process_status_y_motor)
        
        # Transition logic
        if self.X0.active and self.start_motion.active:
            self.logger.info("Prepare trajectory")
            self.X0.deactivate()
            self.X1.activate()  # prepare trajectory

        if self.X1.active and self.trajectory_ready.active:
            self.logger.info("Execute trajectory.")
            self.trajectory_ready.update(False)
            self.X1.deactivate()
            self.X2.activate()  # execute trajectory

        if self.X2.active and self._trajectory_finished():
            self.logger.info("Trajectory finished.")
            self.X2.deactivate()
            self.X0.activate()

    def _execute_actions(self):
        if self.X0.rising_edge:
            self.trajectory = None
            self.num_segments = 0
            self.segment_counter.reset()
        
        if self.X1.rising_edge:
            # Prepare trajectory
            segments = self.shared_data.hmi_data["segments"]
            
            for segment in segments:
                self.logger.info(f"received segment {segment}")
                        
            self.trajectory = self.trajectory_planner.create_trajectory(*segments)
            self.num_segments = len(segments)
            
            self.trajectory_ready.update(True)
            self.x_motor_busy.update(True)
            self.y_motor_busy.update(True)
            
            self.x_motor_ready.update(True)
            self.y_motor_ready.update(True)
        
        if self.X2.active:
            # Execute trajectory segment per segment
            if self.x_motor_ready.active and self.y_motor_ready.active:
                
                self.logger.info(f"Execute segment {self.segment_counter.value}")
                
                self.x_motor_ready.update(False)
                self.y_motor_ready.update(False)
                
                segment = self.trajectory[self.segment_counter.value - 1]
                
                self.logger.info(f"Segment has {len(segment.x_delays)} delays in x.")
                self.logger.info(f"Segment has {len(segment.y_delays)} delays in y.")
                self.logger.info(f"Rotation direction of X-axis: {segment.x_direction}")
                self.logger.info(f"Rotation direction of Y-axis: {segment.y_direction}")

                self.x_interface.send({
                    "cmd": "start_segment",
                    "delays": segment.x_delays,
                    "direction": segment.x_direction,
                })
                self.y_interface.send({
                    "cmd": "start_segment",
                    "delays": segment.y_delays,
                    "direction": segment.y_direction,
                })
                
                self.segment_counter.count_up()
    
    def _process_status_x_motor(self, msg: dict[str, Any]) -> None:
        status = msg.get("status")
        self.logger.info(f"Feedback from X-motor: {status}")
        if status == "segment_done":
            self.travel_time_x.update(msg.get("travel_time", float('nan')))
            if self.segment_counter.value <= self.num_segments:
                self.x_motor_ready.update(True)
            else:
                self.x_motor_ready.update(False)
                self.x_motor_busy.update(False)
        elif status == "startup_error":
            self.logger.error(
                f"[motor X] Startup error: "
                f"{msg.get('message', 'No message')}"
            )
            raise RuntimeError("Startup error in motor X")
    
    def _process_status_y_motor(self, msg: dict[str, Any]) -> None:
        status = msg.get("status")
        self.logger.info(f"Feedback from Y-motor: {status}")
        if status == "segment_done":
            self.travel_time_y.update(msg.get("travel_time", float('nan')))
            if self.segment_counter.value <= self.num_segments:
                self.y_motor_ready.update(True)
            else:
                self.y_motor_ready.update(False)
                self.y_motor_busy.update(False)
        elif status == "startup_error":
            self.logger.error(
                f"[motor Y] Startup error: "
                f"{msg.get('message', 'No message')}"
            )
            raise RuntimeError("Startup error in motor Y")
        
    def _trajectory_finished(self):
        c1 = self.segment_counter.value > self.num_segments
        c2 = not self.x_motor_busy.active
        c3 = not self.y_motor_busy.active
        if c1 and c2 and c3:
            return True
        return False
    
    def control_routine(self):
        self._init_control()
        self._sequence_control()
        self._execute_actions()

    def exit_routine(self):
        self.logger.info("Exit. Shutting down...")

        self.x_interface.send({"cmd": "shutdown"})
        self.y_interface.send({"cmd": "shutdown"})

        self.x_motor_controller.disable()
        self.y_motor_controller.disable()

        self.logger.info("Motorcontrollers stopped.")

    def emergency_routine(self):
        self.logger.warning("Emergency stop. Shutting down...")
        self.exit_routine()

    def crash_routine(self, exception):
        self.logger.error("PLC crashed. Shutting down...")
        self.exit_routine()
        raise exception
