"""
Stepper driver. The stepper driver controls a `TMC2208StepperMotor` with 
UART-interface.
"""
import os
import socket
import json
import threading
import logging
from datetime import datetime

from pyberryplc.stepper import TMC2208StepperMotor, TMC2208UART, PinConfig, RotatorType
from pyberryplc.motion.single_axis import TrapezoidalProfile
from pyberryplc.motion import RotationDirection


# Configure logging to a file to log the operation of the stepper driver.
log_dir = "/shared/python-projects/pyberryplc-platform/demos/demo_stepper_remote/logs"
os.makedirs(log_dir, exist_ok=True)

timestamp = datetime.now().strftime("%Y-%m-%d")
logfile_path = os.path.join(log_dir, f"stepper_drive_{timestamp}.log")

logging.basicConfig(
    filename=logfile_path,
    level=logging.INFO,
    format="[%(name)s %(asctime)s | %(levelname)s] %(message)s",
    datefmt="%d-%m-%Y %H:%M:%S",
    filemode='a'
)


class StepperDriver:
    """
    Controls a `TMC2208StepperMotor`. Communicates with a master PLC program 
    through a TCP connection with JSON messages. The master sends commands to
    the driver which responds by sending status messages back to the master.
    """
    def __init__(self, host: str = '0.0.0.0', port: int = 65432):
        self.host = host
        self.port = port
        self.conn_to_master = None
        self.movement_in_progress = False
        self.logger = logging.getLogger("STEPPER-DRIVER")
        self.stepper_motor = self._create_stepper_motor()

    def _create_stepper_motor(self) -> TMC2208StepperMotor:
        """
        Instantiates a `TMC2208StepperMotor` object and configures the rotation
        of the motor (namely blocking rotation with a motion profile). 
        """
        # Instantiate a `TMC2208StepperMotor`
        stepper_motor = TMC2208StepperMotor(
            pin_config=PinConfig(
                step_pin_ID=21,
                dir_pin_ID=27
            ),
            logger=self.logger,
            name="STEPPER-DRIVER",
            uart=TMC2208UART(port="/dev/ttyUSB1")
        )
        # Set blocking rotation with a motion profile.
        stepper_motor.attach_rotator(RotatorType.MOTION_PROFILE)
        stepper_motor.rotator.profile = TrapezoidalProfile(
            ds_tot=180,
            dt_tot=1,
            dt_i=0.25
        )
        return stepper_motor
    
    def _initialize_stepper_motor(self) -> None:
        """
        Enables the stepper motor driver and configures the stepper motor driver
        (microstepping is set to "full-step mode", and run and hold current are
        set to safe values for the specific stepper motor that is being used).
        """
        # Enable the stepper motor driver.
        self.stepper_motor.enable(high_sensitivity=True)
        
        # Configure microstepping of the stepper motor.
        self.stepper_motor.configure_microstepping(
            resolution="full",
            ms_pins=None,
            full_steps_per_rev=200
        )
        
        # Set run current and hold current of the stepper motor.
        self.stepper_motor.set_current_via_uart(
            run_current_pct=35.0,
            hold_current_pct=10.0
        )
    
    def _notify_master(self, message_dict) -> None:
        """
        Converts response `message_dict` to a JSON string-line, which is then 
        encoded to bytes and sent to the master.
        
        This method is only used in method `_move_stepper()`, which runs in a 
        separate thread, to notify the master as soon as when a rotation is 
        completed.   
        """
        if self.conn_to_master:
            try:
                self.conn_to_master.sendall((json.dumps(message_dict) + "\n").encode())  # `\n` indicates the end of a message.
            except:
                self.logger.error("Failed to send message to master.")

    def _move_stepper(self) -> None:
        """
        Executes the rotation of the stepper motor, and notifies the master
        when the rotation is completed.
        """
        # Movement from start position to end position.
        self.movement_in_progress = True
        self.stepper_motor.rotator.direction = RotationDirection.COUNTERCLOCKWISE
        self.stepper_motor.rotator.rotate()  # blocking rotation
        
        # Movement back from end position to start position.
        self.stepper_motor.rotator.direction = RotationDirection.CLOCKWISE
        self.stepper_motor.rotator.rotate()
        self.movement_in_progress = False
        
        self._notify_master({"status": "done", "message": "Movement complete"})

    def _handle_command(self, command) -> dict[str, str]:
        """
        Handles a command coming from the master and returns a status response. 
        
        The master can send two commands:
        -   "move" to execute a rotation.
        -   "shutdown" to shutdown the stepper driver.
        
        When a "move" command is received, method `_move_stepper()` is called 
        from within a separate thread in order not to block the communication 
        connection between the master and the stepper driver.  
        """
        if command == "move":
            if self.movement_in_progress:
                return {"status": "busy", "message": "Already moving"}
            threading.Thread(target=self._move_stepper, daemon=True).start()
            return {"status": "started", "message": "Movement started"}
        elif command == "shutdown":
            return {"status": "ok", "message": "Shutting down"}
        else:
            return {"status": "error", "message": "Unknown command"}
    
    def run(self):
        """
        Establishes a TCP connection between master and stepper driver. Starts
        a while-loop for exhanging messages between master and stepper driver, 
        so the master can send commands (requests) to the stepper driver and 
        receive status responses back from the stepper driver. Messages between 
        master and stepper driver are sent through the TCP connection using 
        JSON encoding. 
        """
        # Initialize the stepper motor.
        self._initialize_stepper_motor()
        
        # Master - stepper driver communication.
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            # Establish TCP communication with the master.
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port))
            s.listen(1)
            self.logger.info(f"Slave listening on {self.host}:{self.port}...")

            # Wait for the master to connect.
            conn, addr = s.accept()
            
            with conn:
                self.conn_to_master = conn
                self.logger.info(f"Connected to master: {addr}")
                
                # Enter the master-slave message loop.
                while True:
                    data = conn.recv(1024)  # Check receipt of new data.
                    if not data:
                        break
                    
                    # Handle a command received from the master and send
                    # a status response back to the master.
                    try:
                        # Convert bytes to string, parse the JSON string and 
                        # convert it to a dictionary.
                        command = json.loads(data.decode()) 
                        response = self._handle_command(command.get("command", ""))
                    except Exception as e:
                        response = {"status": "error", "message": str(e)}
        
                    conn.sendall((json.dumps(response) + "\n").encode())  # `\n` indicates the end of a message.
                    
                    # If the master sent a shutdown command, the message loop 
                    # and, consequently, the TCP connection with the master are 
                    # broken.
                    if command.get("command") == "shutdown":
                        self.logger.warning("Shutdown received. Closing connection.")
                        break

if __name__ == '__main__':
    # This will be executed when `stepper_driver.py` is opened from `main()`
    # in the main script `main.py`
    
    def main():
        stepper_driver = StepperDriver(host="localhost", port=65432)
        stepper_driver.run()
    
    main()
