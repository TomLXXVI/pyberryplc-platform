"""
Small script to convert the rated current of the stepper motor into a percentage
that is needed to configure the current limit of the TMC2208 stepper motor 
driver.
"""
from pyberryplc.utils.tmc2208_utils import (
    calculate_run_current_pct, 
    calculate_rms_current
)

i_run_pct = calculate_run_current_pct(target_irms=1.2, vfs=0.325)
print(i_run_pct)

i_rms_max = calculate_rms_current(22, vfs=0.325)
print(i_rms_max)
