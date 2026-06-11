"""
Core components for building a Python-based PLC.

This module exposes base classes and utilities for digital I/O, timers,
counters, switches, and the main PLC execution engine.
"""

from .plc_new import AbstractPLC, EmergencyConfig, PLCMode, TAbstractPLC
from .memory import MemoryVariable, HMISharedData, SharedMemoryBlock
from .io_backend import (
    BaseIOBackend, HardwareBackend, SoftwareBackend, SoftMachineState,
    IOChannel
)
from .timers import TimerSingleScan, TimerOnDelay, TimerOffDelay
from .counters import CounterUp, CounterDown, CounterUpDown
from .switches import ToggleSwitch
from .exceptions import (
    InternalCommunicationError,
    ConfigurationError,
    EmergencyException,
    RecoveryException,
)


__all__ = [
    "AbstractPLC",
    "EmergencyConfig",
    "PLCMode",
    "TAbstractPLC",
    "MemoryVariable",
    "HMISharedData",
    "SharedMemoryBlock",
    "BaseIOBackend",
    "HardwareBackend",
    "SoftwareBackend",
    "SoftMachineState",
    "IOChannel",
    "TimerSingleScan",
    "TimerOnDelay",
    "TimerOffDelay",
    "CounterUp",
    "CounterDown",
    "CounterUpDown",
    "ToggleSwitch",
    "InternalCommunicationError",
    "ConfigurationError",
    "EmergencyException",
    "RecoveryException",
]
