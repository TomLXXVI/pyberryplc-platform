"""
Small test script to test operation of the UART interface of the TMC2208
stepper motor driver. 
"""
import logging

from pyberryplc.utils.log_utils import init_logger
from pyberryplc.stepper.uart import TMC2208UART


logger = init_logger(level=logging.DEBUG)


def test():
    with TMC2208UART(port="/dev/ttyUSB1", logger=logger) as uart:
        print("Read CHOPCONF register: ")
        print(uart.read_register("CHOPCONF"))


if __name__ == '__main__':
    test()
