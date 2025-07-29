import keyboard

from pyberryplc.core import MemoryVariable


class KeyInput:

    def __init__(self):
        self.keys: dict[str, MemoryVariable] = {}

    def update(self) -> None:
        """
        Update the remembered key states from the previous scan.
        """
        for key in self.keys:
            self.keys[key].update(keyboard.is_pressed(key))

    def is_pressed(self, key: str) -> bool:
        """
        Check if a key is currently pressed.
        """
        if key not in self.keys:
            self.keys[key] = MemoryVariable(keyboard.is_pressed(key), False)
        return self.keys[key].state

    def rising_edge(self, key: str) -> bool:
        """
        Detect if the key has just been pressed (rising edge).
        """
        if key not in self.keys:
            self.keys[key] = MemoryVariable(keyboard.is_pressed(key), False)
            return self.keys[key].state
        else:
            return self.keys[key].rising_edge
