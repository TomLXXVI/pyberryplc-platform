import keyboard

class KeyInput:
    def __init__(self):
        self.prev_state = {}

    def update(self) -> None:
        """
        Update the remembered key states from the previous scan.
        """
        for key in self.prev_state:
            self.prev_state[key] = keyboard.is_pressed(key)

    def is_pressed(self, key: str) -> bool:
        """
        Check if a key is currently pressed.
        """
        if key not in self.prev_state:
            self.prev_state[key] = False
        return keyboard.is_pressed(key)

    def rising_edge(self, key: str) -> bool:
        """
        Detect if the key has just been pressed (rising edge).

        Returns
        -------
        bool
            True if the key was not pressed last scan, but is pressed now.
        """
        current = keyboard.is_pressed(key)
        previous = self.prev_state.get(key, False)
        return current and not previous
