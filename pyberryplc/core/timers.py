import time


class TimerSingleScan:

    def __init__(self, dt_secs: int | float) -> None:
        """Creates a `SingleScanTimer` object.
        
        Parameters
        ----------
        dt_secs:
            Time span in seconds.
        """
        self.dt = dt_secs
        self._t_start = None

    @property
    def has_elapsed(self) -> bool:
        """The first time when the method is called, the timer is initialized 
        with the current time (in seconds). On every next call, the difference 
        between the current and initial time is taken. If this difference is 
        greater than or equal to the time span `dt_secs`, which was set when 
        instantiating the timer object, `True` is returned. Otherwise, `False` 
        is returned. When the set time span has elapsed (and `True` is returned)
        the timer is also automatically reset. This implies that when 
        `has_elapsed` is called in subsequent PLC scans, it will only return 
        `True` in the PLC scan where the passage of the set time span was
        just detected. It will return `False` again until the set time span has
        elapsed again. 
        """
        if self._t_start is None:
            self._t_start = time.time()
        t_curr = time.time()
        dt = t_curr - self._t_start
        if dt >= self.dt:
            self._t_start = None
            return True
        return False


class TimerOnDelay:

    def __init__(self, dt_secs: int | float) -> None:
        """Creates an `OnDelayTimer` object.

        Parameters
        ----------
        dt_secs:
            Time span in seconds.
        """
        self.dt = dt_secs
        self._t_start = None

    @property
    def has_elapsed(self) -> bool:
        """Returns `False` as long as the set time span has not elapsed, and 
        returns `True` once the time span of the timer has elapsed. 
        To reset the timer, method `reset` must be called.
        """
        if self._t_start is None:
            self._t_start = time.time()
        t_curr = time.time()
        dt = t_curr - self._t_start
        if dt >= self.dt:
            return True
        return False
    
    def reset(self):
        """Resets the timer to the set time span."""
        self._t_start = None


class TimerOffDelay:

    def __init__(self, dt_secs: int | float) -> None:
        """Creates an `OffDelayTimer` object.

        Parameters
        ----------
        dt_secs:
            Time span in seconds.
        """
        self.dt = dt_secs
        self._t_start = None

    @property
    def running(self) -> bool:
        """Returns `True` as long as the set time span has not elapsed, and 
        returns `False` once the time span of the timer has elapsed.
        To reset the timer, method `reset` must be called.
        """
        if self._t_start is None:
            self._t_start = time.time()
        t_curr = time.time()
        dt = t_curr - self._t_start
        if dt >= self.dt:
            return False
        return True

    def reset(self):
        """Resets the timer to the set time span."""
        self._t_start = None
