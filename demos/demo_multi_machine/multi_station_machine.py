"""
Simulation of a multi-station machine.

This is actually a small, unfinished class that only serves to demonstrate 
communication between two machine-units in a larger whole. The 
`MultiStationMachine` sends random `True` or `False` to the `LoadingStation` 
every few seconds to indicate whether the carousel is ready or not. 
"""
import logging
import random

from pyberryplc.core import AbstractPLC, SharedMemoryBlock, TimerOnDelay


class MultiStationMachine(AbstractPLC):
    
    def __init__(
        self, 
        logger: logging.Logger, 
        datablock: SharedMemoryBlock
    ) -> None:
        super().__init__(logger=logger)
        self.datablock = datablock
        self._init_flag = True

        self.X0 = self.add_marker("X0")
        self.X1 = self.add_marker("X1")
        self.X2 = self.add_marker("X2")
        
        self.T0 = TimerOnDelay(3)
    
    def _init_control(self) -> None:
        if self._init_flag:
            self._init_flag = False
            self.X0.activate()
            self.logger.info("Multi-station machine ready.")
    
    def _sequence_control(self) -> None:
        # Check main PLC is running or exiting.
        main_started = self.datablock.read("main_started")
        main_exited = self.datablock.read("main_exited")
        
        # PLC sequence.
        if self.X0.active and main_started.active:
            self.X0.deactivate()
            self.X1.activate()
                
        if self.X1.active and main_exited.active:
            self.X1.deactivate()
            self.X2.activate()
        
    def _execute_actions(self) -> None:
        if self.X1.active:
            if self.T0.has_elapsed:
                r = random.choice([True, False])
                self.datablock.write("rotarydisk_ready", r)
                self.logger.info(f"Rotary disk ready: {r}")
                self.T0.reset()
        
        if self.X2.active :
            self.exit()
                
    def control_routine(self) -> None:
        self._init_control()
        self._sequence_control()
        self._execute_actions()
    
    def exit_routine(self) -> None:
        self.logger.info("Exit multi-station machine.")
    
    def emergency_routine(self) -> None:
        self.exit_routine()
    
    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        self.exit_routine()
  