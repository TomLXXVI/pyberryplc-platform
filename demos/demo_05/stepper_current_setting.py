from pyberryplc.utils.tmc2208_utils import calculate_run_current_pct


run_current_pct = calculate_run_current_pct(target_irms=1.3)
print(run_current_pct)
