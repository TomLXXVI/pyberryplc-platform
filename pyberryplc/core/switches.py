from .plc import MemoryVariable


class ToggleSwitch:
    """Represents a virtual toggle switch (soft switch) which is linked to a 
    pushbutton. The switch is toggled ON or OFF at the raising edge of the 
    pushbutton.  
    
    This can be used to make a monostable pushbutton behave like a bistable
    switch.
    """
    def __init__(self, button: MemoryVariable) -> None:
        """Creates a `ToggleSoftSwitch` object.
        
        Parameters
        ----------
        button:
            Memory variable that is linked to the pushbutton that toggles the 
            soft switch. The switch will be toggled on every raising edge of the 
            pushbutton, i.e., when the state of the pushbutton was low (0) in 
            the previous PLC scan cycle and is high (1) in the current PLC scan 
            cycle.
        """
        self._button = button
        self._switch = MemoryVariable(curr_state=0, prev_state=0)
    
    def update(self) -> None:
        """Updates the state of the switch.  
        
        If the switch is OFF, turns the switch ON at the raising edge of the 
        pushbutton, i.e., if the state of the pushbutton was low (0) in the 
        previous PLC scan cycle and is high (1) in the current PLC scan cycle.
        If the switch was ON, the switch will be turned OFF.
        
        Call this function only once at the beginning of the PLC scan loop.

        Returns
        -------
        The state of the switch in the current PLC scan cycle.
        """
        c1 = self._button.curr_state != self._button.prev_state
        c2 = self._button.curr_state
        if c1 and c2:
            self._switch.update(not self._switch.curr_state)
        else:
            self._switch.update(self._switch.curr_state)
    
    @property
    def curr_state(self) -> bool:
        """Returns the current state of the switch (i.e. the state in the 
        current PLC scan cycle).
        """
        return self._switch.curr_state
    
    @property
    def prev_state(self) -> bool:
        """Returns the previous state of the switch (i.e. the state in the 
        previous PLC scan cycle).
        """
        return self._switch.prev_state
    
    @property
    def active(self) -> bool:
        return self._switch.curr_state

    @property
    def rising_edge(self) -> bool:
        """Returns `True` if `prev_state` is 0 and `curr_state` is 1."""
        return self.curr_state and not self.prev_state

    @property
    def falling_edge(self) -> bool:
        """Returns `True` if `prev_state` is 1 and `curr_state` is 0."""
        return self.prev_state and not self.curr_state
    
    def force(self, value: bool | int) -> None:
        """Programmatically forces the current state of the switch to the given 
        value.
        """
        self._switch.update(value)
