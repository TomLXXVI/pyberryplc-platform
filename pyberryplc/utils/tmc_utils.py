import math

def calculate_run_current_pct(
    target_irms: float,
    rsense: float = 0.11,
    vfs: float = 0.325,
    offset: float = 0.030
) -> int:
    """
    Calculates the `run_current_pct` to achieve a desired RMS motor current.

    Parameters
    ----------
    target_irms : float
        Rated RMS motor current or desired motor RMS current in amperes 
        (e.g. 0.350 for 350 mA).
    rsense : float, optional
        Value of the external sense resistors BRA en BRB in ohms. Default is 
        0.11 ohm. On MKS TMC2208 V2.0 BRA is R7 and BRB is R6; both R0805/0R11
        resistors.
    vfs : float, optional
        Full-scale voltage (V_FS) used by the driver, typically 0.325 V if low
        sensitivity (`vsense = 0`) or 0.180 V if high sensitivity is selected.
    offset : float, optional
        Internal offset resistance (in ohms), typically 0.030.

    Returns
    -------
    int
        Recommended run_current_pct value (0–100) for `set_current_via_uart()`.
    """
    cs = ((target_irms * math.sqrt(2) * (rsense + offset)) / vfs) * 32 - 1
    cs_rounded = round(cs)
    cs_rounded = max(0, min(cs_rounded, 31))  # Clamp to valid range
    pct = round((cs_rounded / 31) * 100)
    return pct
