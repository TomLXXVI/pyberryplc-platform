

class CounterUp:
    
    def __init__(self, preset_val: int = 0) -> None:
        self.preset_value = preset_val
        self.value = preset_val
    
    def count_up(self) -> None:
        self.value += 1
    
    def reset(self) -> None:
        self.value = self.preset_value
    
    def set(self, preset_val: int) -> None:
        self.preset_value = preset_val
        self.value = preset_val


class CounterDown:
    
    def __init__(self, preset_val: int) -> None:
        self.preset_value = preset_val
        self.value = preset_val
    
    def count_down(self) -> None:
        if self.value > 0:
            self.value -= 1
    
    def reset(self) -> None:
        self.value = self.preset_value
    
    def set(self, preset_val: int) -> None:
        self.preset_value = preset_val
        self.value = preset_val


class CounterUpDown:
    
    def __init__(self, preset_value: int = 0) -> None:
        self.preset_value = preset_value
        self.value = preset_value
    
    def count_up(self) -> None:
        self.value += 1
    
    def count_down(self) -> None:
        if self.value > 0:
            self.value -= 1
    
    def reset(self) -> None:
        self.value = self.preset_value
    
    def set(self, preset_val: int) -> None:
        self.preset_value = preset_val
        self.value = preset_val
