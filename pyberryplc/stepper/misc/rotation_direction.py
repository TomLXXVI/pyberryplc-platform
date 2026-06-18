from enum import StrEnum


class RotationDirection(StrEnum):
    CW = "clockwise"
    CCW = "counterclockwise"

    def to_bool(self) -> bool:
        """Returns True for counterclockwise, False for clockwise."""
        return self == RotationDirection.CCW

    def to_int(self) -> int:
        """Returns 1 for counterclockwise, -1 for clockwise."""
        if self == RotationDirection.CW:
            return -1
        return 1

    def __int__(self) -> int:
        return self.to_int()

    def __bool__(self) -> bool:
        raise TypeError("Use .to_bool() for explicit conversion.")

    def __invert__(self) -> 'RotationDirection':
        if self == RotationDirection.CCW:
            return RotationDirection.CW
        return RotationDirection.CCW

    def toggle(self) -> 'RotationDirection':
        return ~self
