"""
Demo of how the situation of multiple machine-units, each with their own PLC 
program, can be handled by using threading and shared data memory.

A loading station transfers items to the rotary disk of a multi-station machine. 
The loading machine and the multi-station machine each run their own PLC program.
The multi-station machine must signal to the loading station when it is ready to
accept a new item.
"""
import logging
import os
import subprocess
import threading

from pyberryplc.core import AbstractPLC, SharedMemoryBlock, MemoryVariable
from pyberryplc.utils.log_utils import init_logger
from pyberryplc.utils.keyboard_input import KeyInput

from loading_station import LoadingStation
from multi_station_machine import MultiStationMachine


class MainPLC(AbstractPLC):
    """
    Overarching, central PLC program from which the PLC programs of the 
    loading station and multi-station machine are started and stopped.
    """
    def __init__(self, logger: logging.Logger) -> None:
        super().__init__(logger=logger)
        self.init_flag = True
        self.key_input = KeyInput()
        
        # The PLC programs of the different machine-units exhange data with each 
        # other through a `SharedMemoryBlock`.
        self.datablock = SharedMemoryBlock(
            name="DB1",
            data={
                "main_started": MemoryVariable(False),
                "main_exited": MemoryVariable(False),
                "rotarydisk_ready": MemoryVariable(False),
            }
        )
        
        # Instantiate the PLC program of the loading station and of the 
        # multi-station machine, and pass them the `SharedMemoryBlock`. 
        self.loadingstation = LoadingStation(init_logger("LOADING-STATION"), self.datablock)
        self.multistation = MultiStationMachine(init_logger("MULTI-STATION"), self.datablock)
        self.thr_loadingstation: threading.Thread | None = None
        self.thr_multistation: threading.Thread | None = None
        
        # Create step markers for the program sequence.
        self.X0 = self.add_marker("X0")
        self.X1 = self.add_marker("X1")
        self.X2 = self.add_marker("X2")
        
    def _init_control(self) -> None:
        if self.init_flag:
            self.logger.info("Main program starting up...")
            self.init_flag = False
            
            # Start PLC scan loop of loading station in a thread.
            self.thr_loadingstation = threading.Thread(target=self.loadingstation.run)
            self.thr_loadingstation.start()
            
            # Start PLC scan loop of multi-station machine in a thread.
            self.thr_multistation = threading.Thread(target=self.multistation.run)
            self.thr_multistation.start()
            
            # Note: we need threads otherwise the PLC scan loop of the loading
            # station would block the startup of the PLC scan loop of the 
            # multi-station machine.
            
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
            # Do nothing, wait for the 's' key to be pressed.
            self.logger.info("Press 's' to start.")
        
        if self.X1.rising_edge:
            # Signal through the shared data memory to the machine-units that 
            # the main program has been started.
            self.datablock.write("main_started", True)
            self.datablock.write("main_exited", False)
            self.logger.info("Press 'e' to exit.")
        
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
        # Wait here until the PLC programs of the machine-units have exited. 
        self.thr_loadingstation.join()
        self.thr_multistation.join()
    
    def emergency_routine(self) -> None:
        self.exit_routine()
    
    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        self.exit_routine()
        raise exception


def main():
    # Clear the console screen.
    os.system("clear")

    # A stepper driver is used by the loading station to transfer items from a
    # location POS2 in the loading station to a location POS3. The stepper 
    # driver runs in a completely separate process and communicates with the
    # PLC program of the loading station through a TCP connection.
    # Below the stepper driver script `stepper_driver.py` is launched in a new 
    # subprocess.
    project_path = "/shared/python-projects/pyberryplc-platform/demos/demo_stepper_remote"
    venv_activate = "/shared/python-projects/pyberryplc-platform/.venv/bin/activate"
    stepper_driver_script = os.path.join(project_path, "stepper_driver.py")

    subprocess.Popen(
        ['bash', '-c', f'source {venv_activate} && python {stepper_driver_script}']
    )
    
    # Instantiate the main PLC program and start it.
    main_plc = MainPLC(logger=init_logger("MAIN"))
    main_plc.run()


if __name__ == '__main__':
    main()
