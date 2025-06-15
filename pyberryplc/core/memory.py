from typing import Any
from dataclasses import dataclass, field


@dataclass
class MemoryVariable:
    """
    Represents a variable with a memory: the variable holds its current state,
    but also remembers its previous state (from the previous PLC scan cycle). 
    This allows for edge detection. 

    Attributes
    ----------
    curr_state:
        Current state of the variable, i.e., its state in the current PLC
        scan cycle.
    prev_state:
        Previous state of the variable, i.e., its state in the previous PLC
        scan cycle.
    single_bit: bool
        Indicates that the memory variable should be treated as a single bit 
        variable (its value can be either 0 or 1). Default value is `True`.
    decimal_precision: int
        Sets the decimal precision for floating point values in case the memory
        variable state is represented by a float. The default precision is 3.
    """
    curr_state: Any = None
    prev_state: Any = None
    single_bit: bool = False
    decimal_precision: int = 3

    def __post_init__(self):
        if self.curr_state is None: self.curr_state = False
        c1 = isinstance(self.curr_state, bool)
        c2 = isinstance(self.curr_state, int) and self.curr_state in (0, 1)
        if c1 or c2: self.single_bit = True
        if self.prev_state is None: self.prev_state = self.curr_state

    def update(self, value: Any) -> None:
        """
        Updates the current state of the variable with parameter `value`.
        Before `value` is assigned to the current state of the variable, the
        preceding current state is stored in attribute `prev_state`.
        """
        self.prev_state = self.curr_state
        self.curr_state = value

    @property
    def active(self) -> bool:
        """
        Returns `True` if the current state evaluates to `True`, else returns
        `False`.
        """
        if self.curr_state:
            return True
        return False

    def activate(self) -> None:
        """
        Sets the current state of the variable to `True` or 1. Only valid for 
        single bit variables.
        """
        if self.single_bit:
            if isinstance(self.curr_state, int):
                self.update(1)
            else:
                self.update(True)
        else:
            raise ValueError("Memory variable is not single bit.")

    def deactivate(self) -> None:
        """
        Sets the current state of the variable to `False` or 0. Only valid for 
        single bit variables.
        """
        if self.single_bit:
            if isinstance(self.curr_state, int):
                self.update(0)
            else:
                self.update(False)
        else:
            raise ValueError("Memory variable is not single bit.")

    @property
    def rising_edge(self) -> bool:
        """
        Returns `True` if `prev_state` evaluates to `False` and `curr_state` 
        evaluates to `True`. Only valid for single bit variables.
        """
        if self.single_bit:
            if self.curr_state and not self.prev_state:
                return True
            return False
        else:
            raise ValueError("Memory variable is not single bit.")

    @property
    def falling_edge(self) -> bool:
        """Returns `True` if `prev_state` evaluates to `False` and `curr_state` 
        evaluates to `True`. Only valid for single bit variables.
        """
        if self.single_bit:
            if self.prev_state and not self.curr_state:
                return True
            return False
        else:
            raise ValueError("Memory variable is not single bit.")

    @property
    def state(self) -> Any:
        """
        Returns the current state (value) of the memory variable, i.e. the 
        state (value) in the current PLC scan cycle. If the state is represented
        by a float, it will be rounded to the default decimal precision of 3 or
        the decimal precision specified when the memory variable was first 
        instantiated.
        """
        if isinstance(self.curr_state, float):
            return round(self.curr_state, self.decimal_precision)
        else:
            return self.curr_state


@dataclass
class HMISharedData:
    """
    Dataclass for sharing data between a PLC application and a HMI application.
    """
    buttons: dict[str, bool] = field(default_factory=dict)
    switches: dict[str, bool] = field(default_factory=dict)
    analog_inputs: dict[str, float] = field(default_factory=dict)
    digital_outputs: dict[str, bool] = field(default_factory=dict)
    analog_outputs: dict[str, float] = field(default_factory=dict)
    data: dict[str, Any] = field(default_factory=dict)


@dataclass
class SharedMemoryBlock:
    """
    Dataclass for data exchange between PLC application units.
    
    It has an attribute `data` which is a dictionary with string keys for the
    names of the variables and the corresponding values are `MemoryVariable`
    objects.
    """
    name: str = ""
    data: dict[str, MemoryVariable] = field(default_factory=dict)
    
    def read(self, name: str) -> MemoryVariable:
        """
        Returns the memory variable with the given name from the shared memory 
        block.
        """
        try:
            return self.data[name]
        except KeyError:
            raise KeyError(f"Memory variable {name} does not exist.")
    
    def write(self, name: str, value: Any) -> None:
        """
        Writes the given value to the memory variable with the given name in
        the shared memory block.
        """
        try:
            self.data[name].update(value)
        except KeyError:
            raise KeyError(f"Memory variable {name} does not exist.")
