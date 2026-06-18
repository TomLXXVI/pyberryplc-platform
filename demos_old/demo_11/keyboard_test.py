"""
Demonstrates how to use the keyboard to toggle a soft switch.
"""
from pyberryplc.core import AbstractPLC, MemoryVariable, ToggleSwitch
from pyberryplc.utils.keyboard_input import KeyInput


class MyPLC(AbstractPLC):

    def __init__(self, logger):
        super().__init__(logger=logger)
        self.key_input = KeyInput()
        self.init_flag: bool = True

        self.l_key = MemoryVariable()  # Create a MemoryVariable `l_key`.
        self.key_input.keys["l"] = self.l_key  # Bind MemoryVariable `l_key` to keyboard key "l".
        self.L_switch = ToggleSwitch(self.l_key)  # Bind MemoryVariable `l_key` to ToggleSwitch `L_switch`.

        # Result:
        # A press on key "l" turns MemoryVariable `l_key` one scan (or a few scans) HIGH.
        # If `L_switch` is LOW, it will be turned HIGH and stay HIGH until key "l" is pressed again.
        # If `L_switch` is HIGH, it will be turned LOW and stay LOW until key "l" is pressed again.

        self.X0 = self.add_marker("X0")
        self.X1 = self.add_marker("X1")

    def _init_control(self) -> None:
        if self.init_flag:
            self.init_flag = False
            self.X0.activate()

    def _sequence_control(self) -> None:
        self.key_input.update()
        # At the start of a new scan cycle, update the state of memory variable
        # `l_key`. If key "l" is pressed at the start of the scan cycle,
        # `l_key` is turned HIGH, otherwise `l_key` is turned LOW (or stays LOW).

        self.L_switch.update()
        # At the start of a new scan cycle, the internal switch state must also
        # be updated by the state of its associated `l_key`.

        if self.X0.active and self.L_switch.active:
            self.X0.deactivate()
            self.X1.activate()

        if self.X1.active and not self.L_switch.active:
            self.X1.deactivate()
            self.X0.activate()

        if self.X1.active and self.key_input.rising_edge("e"):
            self.exit()

    def _execute_actions(self) -> None:

        if self.X0.rising_edge:
            self.logger.info("State X0 is active.")

        if self.X1.rising_edge:
            self.logger.info("State X1 is active.")

    def control_routine(self) -> None:
        self._init_control()
        self._sequence_control()
        self._execute_actions()

    def exit_routine(self) -> None:
        self.logger.info("Bye, bye")

    def emergency_routine(self) -> None:
        self.logger.info("Auwch")

    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        self.logger.info("What the hell has happened?")
        raise exception


if __name__ == '__main__':

    import os
    from pyberryplc.utils.log_utils import init_logger

    os.system("clear")
    logger = init_logger(name="MyPLC", log_file=None)
    logger.info("Starting keyboard test...")
    plc = MyPLC(logger)
    plc.run()
