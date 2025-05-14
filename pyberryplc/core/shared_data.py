from typing import Any
from dataclasses import dataclass, field


@dataclass
class SharedData:
    hmi_buttons: dict[str, bool] = field(default_factory=dict)
    hmi_switches: dict[str, bool] = field(default_factory=dict)
    hmi_analog_inputs: dict[str, float] = field(default_factory=dict)
    hmi_digital_outputs: dict[str, bool] = field(default_factory=dict)
    hmi_analog_outputs: dict[str, float] = field(default_factory=dict)
    hmi_data: dict[str, Any] = field(default_factory=dict)
