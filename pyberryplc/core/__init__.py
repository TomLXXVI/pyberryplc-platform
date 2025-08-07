"""
Core components for building a Python-based PLC.

This module exposes base classes and utilities for digital I/O, timers,
counters, switches, and the main PLC execution engine.
"""

from .plc import AbstractPLC, TAbstractPLC
from .memory import MemoryVariable, HMISharedData, SharedMemoryBlock
from .gpio import DigitalInput, DigitalOutput, PWMOutput, DigitalOutputPigpio
from .timers import TimerSingleScan, TimerOnDelay, TimerOffDelay
from .counters import CounterUp, CounterDown, CounterUpDown
from .switches import ToggleSwitch
from .exceptions import InternalCommunicationError, ConfigurationError, EmergencyException


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
