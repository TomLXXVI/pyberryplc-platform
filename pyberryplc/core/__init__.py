"""
Core components for building a Python-based PLC.

This module exposes base classes and utilities for digital I/O, timers,
counters, switches, and the main PLC execution engine.
"""

from pyberryplc.core.plc import AbstractPLC, TAbstractPLC
from pyberryplc.core.memory import MemoryVariable, HMISharedData, SharedMemoryBlock
from pyberryplc.core.gpio import DigitalInput, DigitalOutput, PWMOutput, DigitalOutputPigpio
from pyberryplc.core.timers import TimerSingleScan, TimerOnDelay, TimerOffDelay
from pyberryplc.core.counters import CounterUp, CounterDown, CounterUpDown
from pyberryplc.core.switches import ToggleSwitch
from pyberryplc.core.exceptions import InternalCommunicationError, ConfigurationError, EmergencyException


__all__ = [
    "AbstractPLC",
    "TAbstractPLC",
    "MemoryVariable",
    "HMISharedData",
    "SharedMemoryBlock",
    "DigitalInput",
    "DigitalOutput",
    "PWMOutput",
    "DigitalOutputPigpio",
    "TimerSingleScan",
    "TimerOnDelay",
    "TimerOffDelay",
    "CounterUp",
    "CounterDown",
    "CounterUpDown",
    "ToggleSwitch",
    "InternalCommunicationError",
    "ConfigurationError",
    "EmergencyException"
]
