from typing import Any
from dataclasses import dataclass, field


@dataclass
class SharedData:
    hmi_buttons: dict[str, Any] = field(default_factory=dict)
    hmi_switches: dict[str, Any] = field(default_factory=dict)
    hmi_analog_inputs: dict[str, Any] = field(default_factory=dict)
    hmi_outputs: dict[str, Any] = field(default_factory=dict)
    hmi_data: dict[str, Any] = field(default_factory=dict)
