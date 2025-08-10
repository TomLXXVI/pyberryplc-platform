"""
Main PLC-program.
"""
import os
import subprocess
import threading

from pyberryplc.core import AbstractPLC, SharedMemoryBlock, MemoryVariable
from pyberryplc.utils.log_utils import init_logger
from pyberryplc.utils.keyboard_input import KeyInput

from loading_station import LoadingStation
from multi_operation_machine import MultiOperationMachine


class MainPLC(AbstractPLC):
    """
    Overarching, central PLC program from which the PLC programs of the 
    loading station and of the multi-operation machine are started and stopped.
    """
    def __init__(self) -> None:
        super().__init__(logger=init_logger("MAIN"))

        self.init_flag = True
        self.key_input = KeyInput()
        
        # The PLC programs of different machine-units exhange data with each 
        # other through a `SharedMemoryBlock`.
        self.datablock = SharedMemoryBlock(
            name="DB1",
            data={
                "main_started": MemoryVariable(False),
                "main_exited": MemoryVariable(False),
                "rotarydisk_ready": MemoryVariable(False),
            }
        )
        
        # Instantiate the PLC program of the loading station and the
        # multi-operation machine, passing the `SharedMemoryBlock` to them.
        self.loadingstation = LoadingStation(
            init_logger("LOADING-STATION"),
            self.datablock)
        self.loadingstation_thread: threading.Thread | None = None
        
        self.multi_machine = MultiOperationMachine(
            init_logger("multi-operation"),
            self.datablock
        )
        self.multistation_thread: threading.Thread | None = None
        
        # Create step markers for the program sequence.
        self.X0 = self.add_marker("X0")
        self.X1 = self.add_marker("X1")
        self.X2 = self.add_marker("X2")
        
    def _init_control(self) -> None:
        if self.init_flag:
            self.logger.info("Main program is starting...")
            self.init_flag = False
            
            # Start PLC scan loop of loading station in a thread.
            self.loadingstation_thread = threading.Thread(target=self.loadingstation.run)
            self.loadingstation_thread.start()
            
            # Start PLC scan loop of multi-operation machine in a thread.
            self.multistation_thread = threading.Thread(target=self.multi_machine.run)
            self.multistation_thread.start()
            
            # Note: we need threads otherwise the PLC scan loop of the loading
            # station would block the startup of the PLC scan loop of the 
            # multi-operation machine.
            
            self.X0.activate()
            
    def _sequence_control(self) -> None:
        if self.X0.active and self.key_input.rising_edge("s"):
            # The machine-units are started when the 's' key has been pressed.
            self.X0.deactivate()
            self.X1.activate()

        if self.X1.active and self.key_input.rising_edge("e"):
            # The machine-units are shutdown when the 'e' key has been pressed.
            self.X1.deactivate()
            self.X2.activate()
    
    def _execute_actions(self) -> None:
        if self.X0.rising_edge:
            # Do nothing, just wait for the 's' key to be pressed.
            self.logger.info("Press key 's' to start.")
        
        if self.X1.rising_edge:
            # Signal through the shared data memory to the machine-units that 
            # the main program has been started.
            self.datablock.write("main_started", True)
            self.datablock.write("main_exited", False)
            self.logger.info("Press key 'e' to exit.")
        
        if self.X2.rising_edge:
            # Signal through the shared data memory to the machine-units that 
            # the main program will be closed.
            self.logger.info("Calling main exit routine.")
            self.datablock.write("main_exited", True)
            self.datablock.write("main_started", False)
            self.exit()
    
    def control_routine(self) -> None:
        self.key_input.update()
        self._init_control()
        self._sequence_control()
        self._execute_actions()
    
    def exit_routine(self) -> None:
        self.logger.info("Exit main program.")
        # Wait here until the PLC programs of the other units have exited.
        self.loadingstation_thread.join()
        self.multistation_thread.join()
    
    def emergency_routine(self) -> None:
        self.exit_routine()
    
    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        self.exit_routine()
        raise exception


def main():
    # Clear the console screen.
    os.system("clear")

    # A stepper motor driver is used in the loading station to transfer items
    # from location POS2 in the loading station to location POS3. The driver
    # runs in a completely separate process and communicates with the
    # PLC program of the loading station through a TCP connection.
    # Below the script `stepper_driver.py` is launched in a new OS subprocess.
    project_path = "/shared/python-projects/pyberryplc-platform/demos/demo_09"
    venv_activate = "/shared/python-projects/pyberryplc-platform/.venv/bin/activate"
    stepper_driver_script = os.path.join(project_path, "stepper_driver.py")
    subprocess.Popen([
        'bash', '-c',
        f'source {venv_activate} && python {stepper_driver_script}'
    ])
    
    # Instantiate the main PLC program and start it.
    main_plc = MainPLC()
    main_plc.run(measure=True)  # indicate that PLC scan time should be measured


if __name__ == '__main__':
    main()
