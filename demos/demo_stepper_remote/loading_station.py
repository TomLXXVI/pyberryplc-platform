"""
Simulation of a loading station. Demo on how to implement simultaneous tasks
that run side by side using an SFC ("Sequential Function Chart") approach.

In the loading station, a batch of items is transported from the entrance to 
the exit of the loading station. At the exit an item is attached to the rotary
disk of multi-station machine. Each item in the batch is moved through three 
positions in the loading station (indicated by POS1, POS2, and POS3). 

To move an item from POS2 to POS3 a stepper driver is used. The stepper driver
runs in a separate application and communicates with the master PLC over a
TCP connection with messages encoded in JSON format. 
"""
import logging

from pyberryplc.core import AbstractPLC, SharedMemoryBlock
from pyberryplc.core.timers import TimerOnDelay
from pyberryplc.core.counters import CounterUp
from pyberryplc.utils.remote_interface import TCPRemoteDeviceClient


class LoadingStation(AbstractPLC):
    
    def __init__(
        self, 
        logger: logging.Logger,
        datablock: SharedMemoryBlock
    ) -> None:
        super().__init__(logger=logger)
        self.datablock = datablock
        
        # Remote stepper driver
        self.stepper_driver = TCPRemoteDeviceClient(logger=self.logger)
                        
        # SFC Step markers
        self.X0 = self.add_marker('X0')
        
        self.X10 = self.add_marker('X10')
        self.X11 = self.add_marker('X11')
        self.X12 = self.add_marker('X12')

        self.X20 = self.add_marker('X20')
        self.X21 = self.add_marker('X21')
        self.X22 = self.add_marker('X22')

        self.X30 = self.add_marker('X30')
        self.X31 = self.add_marker('X31')
        self.X32 = self.add_marker('X32')

        self.X40 = self.add_marker('X40')
        self.X41 = self.add_marker('X41')
        self.X42 = self.add_marker('X42')
        
        # Markers that simulate detection sensors at three positions.
        self.pos1_free = self.add_marker('pos1_free')
        self.pos2_free = self.add_marker('pos2_free')
        self.pos3_free = self.add_marker('pos3_free')
    
        # Flags & Other Stuff
        self.init_flag = True
        self.batch_size: int = 5
        
        # Timers
        self.T1 = TimerOnDelay(1)
        self.T2 = TimerOnDelay(2)
        self.T4 = TimerOnDelay(1)
        
        # Counters
        self.item_counter = CounterUp()
        
    def _init_control(self):
        if self.init_flag:
            # Lock `_init_control()` in subsequent PLC scan cycles.
            self.init_flag = False
            
            # Connect to the stepper driver.
            self.logger.info("Connect to stepper driver.")
            self.stepper_driver.connect()
            
            # At the start of the PLC program all loading positions are free
            # (no items in the loading station).
            self.pos1_free.update(1)           
            self.pos2_free.update(1)
            self.pos3_free.update(1)
                        
            # Step into the PLC sequence.
            self.X0.activate()
            self.logger.info("Loading station ready.")
    
    def _sequence_control(self):
        # Check main PLC is still running or closing.
        main_started = self.datablock.read("main_started")
        main_exited = self.datablock.read("main_exited")
        
        # Check rotary disk of multi-station machine is ready to accept a new 
        # item.
        rotarydisk_ready = self.datablock.read("rotarydisk_ready")
        
        # PLC sequence. 
        if self.X0.active and main_exited.active:
            # Main program is closing.
            self.exit()
        
        if self.X0.active and main_started.active:
            self.logger.info("Start batch.")
            self.X0.deactivate()
            # Activate 4 steps simultaneously (i.e. in the same PLC scan cycle).
            self.X10.activate()
            self.X20.activate() 
            self.X30.activate()
            self.X40.activate()
        
        if self.X10.active and self.pos1_free.active:
            # Items are available and no item at POS1.
            self.X10.deactivate()
            self.X11.activate()  # Transfer an item to POS1.
        
        if self.X11.active and not self.pos1_free.active:
            # An item is at POS1.
            self.X11.deactivate()
            self.X12.activate()
        
        if self.X12.active and not self.item_counter.value == self.batch_size:
            self.X12.deactivate()
            self.X10.activate()  # Go back to wait step X10.
        
        if self.X20.active and not self.pos1_free.active and self.pos2_free.active:
            # An item is at POS1 but no item is at POS2.
            self.X20.deactivate()
            self.X21.activate()  # Transfer item from POS1 to POS2.
        
        if self.X21.active and not self.pos2_free.active:
            # An item is at POS2.
            self.X21.deactivate()
            self.X22.activate()
        
        if self.X22.active and not self.item_counter.value == self.batch_size:
            self.X22.deactivate()
            self.X20.activate()  # Go back to wait step X20.
        
        if self.X30.active and not self.pos2_free.active and self.pos3_free.active:
            # An item is at POS2 but no item is at POS3.
            self.X30.deactivate()
            self.X31.activate()  # Transfer item from POS2 to POS3.
        
        if self.X31.active and not self.pos3_free.active:
            # An item is at POS3.
            self.X31.deactivate()
            self.X32.activate()  
        
        if self.X32.active and not self.item_counter.value == self.batch_size:
            self.X32.deactivate()
            self.X30.activate()  # Go back to wait step X30.
        
        if self.X40.active and main_exited.active:
            # If main program is closing, multi-station machine may already be
            # closed and the sequence will get stuck on step X40. 
            self.X40.deactivate()
            self.exit()
        
        if self.X40.active and not self.pos3_free.active and rotarydisk_ready.active:
            # An item is at POS3 and the rotary disk from the multi-station 
            # machine is ready.
            self.X40.deactivate()
            self.X41.activate()  # Transfer item from POS3 to rotary disk.
                
        if self.X41.active and self.pos3_free.active:
            # No item at POS3: transfer is finished.
            self.X41.deactivate()
            self.X42.activate()
        
        if self.X42.active and not self.item_counter.value == self.batch_size:
            self.X42.deactivate()
            self.X40.activate()  # Go back to wait step X40.
        
        if (
            self.X12.active 
            and self.X22.active 
            and self.X32.active 
            and self.X42.active 
            and self.item_counter.value == self.batch_size
        ):
            # All transfer operations are done and batch is completed.
            self.logger.info("Batch completed.")
            self.X12.deactivate()
            self.X22.deactivate()
            self.X32.deactivate()
            self.X42.deactivate()
            self.X0.activate()
        
    def _execute_actions(self):
        if self.X0.rising_edge:
            self.item_counter.reset()
        
        if self.X10.active:
            pass  
            # Wait for the associated step transition condition to become `True` 
            # before continuing with the next step in the sequence.
        
        if self.X11.active:
            if self.X11.rising_edge:
                self.logger.info("Transfer item to POS1.")
                self.T1.reset()
            # Simulate transfer of item to POS1 by running an on-delay timer.
            if self.T1.has_elapsed:
                self.pos1_free.update(False)  # Item is now in POS1.
                if self.pos1_free.falling_edge:
                    self.item_counter.count_up()  # Count items that enter the loading station.
                    self.logger.info(f"Items count: {self.item_counter.value}")
                
        if self.X12.active:
            pass
        
        if self.X20.active:
            pass
        
        if self.X21.active:
            if self.X21.rising_edge:
                self.logger.info("Transfer item from POS1 to POS2.")
                self.T2.reset()
            # Simulate transfer of item from POS1 to POS2.
            if self.T2.has_elapsed:
                self.pos2_free.update(False)  # Item is now at POS2,
                self.pos1_free.update(True)   # while POS1 is free again.
        
        if self.X22.active:
            pass
        
        if self.X30.active:
            pass
        
        if self.X31.active:
            if self.X31.rising_edge:
                try:
                    # Send command to stepper driver to execute its movement.
                    self.logger.info("Transfer item from POS2 to POS3.")
                    self.stepper_driver.send_command({"command": "move"})
                    self.logger.info("Command 'move' sent to stepper.")
                except Exception as e:
                    self.logger.error(f"Sending of 'move' failed: {e}")
            
            try:
                self.stepper_driver.wait_for_done()  # Pauses the PLC scan cycle.
                self.logger.info("Servo confirmed: movement finished.")
                self.pos3_free.update(False)  # Item is now at POS3,
                self.pos2_free.update(True)   # while POS2 is free again.
            except Exception as e:
                self.logger.error(f"Error while waiting for 'done': {e}")
         
        if self.X32.active:
            pass
        
        if self.X40.active:
            pass
        
        if self.X41.active:
            if self.X41.rising_edge:
                self.logger.info("Transfer item from POS3 to rotary disk")
                self.T4.reset()
            # Simulate transfer of item to rotary disk.  
            if self.T4.has_elapsed:
                self.pos3_free.update(True)  # POS3 is free again. 

    def control_routine(self):
        self._init_control()
        self._sequence_control()
        self._execute_actions()
    
    def exit_routine(self):
        self.logger.info("Exit loading station.")
        self.logger.info("Shutdown stepper driver.")
        try:
            self.stepper_driver.shutdown()
        except Exception:
            self.logger.warning("Failure to send shutdown to stepper drive.")
        self.stepper_driver.close()
    
    def emergency_routine(self):
        self.exit_routine()
    
    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        self.exit_routine()
