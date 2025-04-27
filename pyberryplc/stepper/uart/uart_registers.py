from typing import TypeVar, Type, cast
from abc import ABC, abstractmethod


T = TypeVar("T", bound="Register")


class Register(ABC):
    """
    Abstract base class for all UART-accessible stepper driver registers.
    """

    @classmethod
    @abstractmethod
    def field_layout(cls) -> dict[str, tuple[int, int]]:
        """
        Returns the bit layout of the register fields.
        Returns a dictionary where the key is the field name and the value is a
        (bit-position, bit-width) tuple.
        """
        pass

    @abstractmethod
    def as_dict(self) -> dict[str, int | bool]:
        """
        Returns the field values as a dictionary for introspection or updates.
        """
        pass

    def to_int(self) -> int:
        """
        Converts the register's fields into a single 32-bit integer 
        representation.

        Returns
        -------
        int
            Encoded 32-bit register value.

        Raises
        ------
        ValueError
            If a field value is too large for its bit-width.
        """
        value = 0
        layout = self.field_layout()

        for name, (pos, width) in layout.items():
            field_value = getattr(self, name)
            if isinstance(field_value, bool):
                field_value = int(field_value)
            if field_value >= (1 << width):
                raise ValueError(
                    f"Value {field_value} too large for "
                    f"field '{name}' (max {2 ** width - 1})"
                )
            value |= field_value << pos

        return value

    @classmethod
    def from_int(cls: Type[T], value: int) -> T:
        """Creates a `Register` instance from a 32-bit integer."""
        field_values = {}
        for name, (pos, width) in cls.field_layout().items():
            mask = (1 << width) - 1
            raw = (value >> pos) & mask
            field_values[name] = bool(raw) if width == 1 else raw
        return cast(T, cls(**field_values))
