"""
Core components for building a Python-based PLC.

This module exposes base classes and utilities for digital I/O, timers,
counters, switches, and the main PLC execution engine.
"""

from pyberryplc.core.plc import AbstractPLC, MemoryVariable
from pyberryplc.core.gpio import DigitalInput, DigitalOutput, PWMOutput
from pyberryplc.core.timers import TimerSingleScan, TimerOnDelay, TimerOffDelay
from pyberryplc.core.counters import CounterUp, CounterDown, CounterUpDown
from pyberryplc.core.switches import ToggleSwitch
from pyberryplc.core.exceptions import InternalCommunicationError, ConfigurationError, EmergencyException

__all__ = [
    "AbstractPLC",
    "MemoryVariable",
    "DigitalInput",
    "DigitalOutput",
    "PWMOutput",
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
