"""
Utility functions and classes for the TMC2208 stepper motor driver.
"""
import math
import csv
import time
from typing import TextIO
from pathlib import Path
import typing
import matplotlib.pyplot as plt

from pyberryplc.stepper import TMC2208UART
from pyberryplc.stepper.uart import DRVSTATUSRegister


def calculate_run_current_pct(
    target_irms: float,
    rsense: float = 0.11,
    vfs: float = 0.325,
    offset: float = 0.030
) -> float:
    """
    Intended as a helper function for setting a safe motor current limit via 
    UART using the `set_current_via_uart()` method of class 
    `TMC2208StepperMotor`. It calculates the `run_current_pct` value that 
    corresponds with the given motor current `target_irms`.

    Parameters
    ----------
    target_irms : float
        Rated RMS motor current or desired motor RMS current in amperes 
        (e.g. 0.350 for 350 mA).
    rsense : float, optional
        Value of the external sense resistors BRA en BRB in ohms. Default is 
        0.11 Ohm. On driver MKS TMC2208 V2.0 BRA is R7 and BRB is R6; both are
        R0805/0R11 resistors.
    vfs : float, optional
        Full-scale voltage in volts across the sense resistors applied by the 
        driver, typically 0.325 V if low sensitivity (`vsense = 0`) or 0.180 V 
        if high sensitivity is selected.
    offset : float, optional
        Internal resistance of MOSFETs in H-bridge (in ohms), typically 0.030 
        Ohm (cf. TMC2208 datasheet - 9. Motor current control).

    Returns
    -------
    float: 
        `run_current_pct` value (0–100).
    """
    # Calculate current scale.
    cs = target_irms * math.sqrt(2) * (rsense + offset) / vfs * 32 - 1
    cs_rounded = round(cs)
    cs_rounded = max(0, min(cs_rounded, 31))  # Clamp to valid range.
    pct = round((cs_rounded / 31) * 100)
    return pct


def calculate_rms_current(
    cs_actual: int,
    rsense: float = 0.11,
    vfs: float = 0.325,
    offset: float = 0.030
) -> float:
    """
    Calculates the approximate RMS current that corresponds with the actual 
    current scale `cs_actual`.
    
    Parameters
    ----------
    cs_actual:
       The actual current scale read via UART from the DRV_STATUS register of
       the TMC2208 driver.
    rsense : float, optional
        Value of the external sense resistors BRA en BRB in ohms. Default is 
        0.11 Ohm. On driver MKS TMC2208 V2.0 BRA is R7 and BRB is R6; both are
        R0805/0R11 resistors.
    vfs : float, optional
        Full-scale voltage in volts across the sense resistors applied by the 
        driver, typically 0.325 V if low sensitivity (`vsense = 0`) or 0.180 V 
        if high sensitivity is selected.
    offset : float, optional
        Internal resistance of MOSFETs in H-bridge (in ohms), typically 0.030 
        Ohm (cf. TMC2208 datasheet - 9. Motor current control).
    """
    i_rms = (cs_actual + 1) / 32 * vfs / (rsense + offset) * 1 / math.sqrt(2)
    return i_rms


class RMSCurrentLogger:
    """
    Retrieves stepper motor RMS currents from a TMC2208 driver via UART and 
    writes them to a CSV file.
    """
    def __init__(
        self,
        uart: TMC2208UART | None,
        filename: str = "rms_current.csv"
    ) -> None:
        """
        Creates a `RMSCurrentLogger` instance.

        Parameters
        ----------
        uart:
            UART interface of `TMC2208StepperMotor` instance.
        filename:
            Name for the CSV file where current values will be written to.
        """
        self.uart = uart
        self.filename = filename
        self.file: TextIO | None = None
        self.writer: csv.writer = None

    def open(self) -> None:
        """
        Opens the CSV log file.
        """
        self.file = open(self.filename, mode="w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["timestamp", "i_rms"])

    def log(self) -> None:
        """
        Requests the actual motor RMS current from the TMC2208 driver through 
        the UART interface and writes the returned value into the CSV file. 
        """
        if not self.file or not self.writer:
            raise RuntimeError("RMSCurrentLogger not opened. Call .open() first.")
        drv_status = typing.cast(DRVSTATUSRegister, self.uart.read_register("DRV_STATUS"))
        i_rms = calculate_rms_current(drv_status.cs_actual)
        timestamp = time.time()
        self.writer.writerow([timestamp, i_rms])
        self.file.flush()

    def close(self) -> None:
        """
        Closes the CSV log file.
        """
        if self.file:
            self.file.close()
            self.file = None
            self.writer = None

    def plot(self) -> None:
        """
        Reads the CSV log file and plots the motor RMS currents over time.
        """
        path = Path(self.filename)
        if not path.exists():
            raise FileNotFoundError(f"Log file {self.filename} not found.")

        timestamps: list[float] = []
        currents: list[float] = []

        with path.open(mode="r", newline="") as f:
            reader = csv.reader(f)
            next(reader)  # skip header
            for row in reader:
                timestamps.append(float(row[0]))
                currents.append(float(row[1]))

        if not timestamps:
            raise ValueError("Log file is empty or malformed.")

        t0 = timestamps[0]
        time_rel = [t - t0 for t in timestamps]

        plt.figure()
        plt.plot(time_rel, currents)
        plt.xlabel("Time (s)")
        plt.ylabel("RMS Current (A)")
        plt.title("Stepper Motor RMS Current")
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    def save_plot(self, output_path: str = "rms_current.png") -> None:
        """
        Reads the log file and saves a graph of i_rms vs time to a PNG file.
        """
        path = Path(self.filename)
        if not path.exists():
            raise FileNotFoundError(f"Log file {self.filename} not found.")

        timestamps: list[float] = []
        currents: list[float] = []

        with path.open(mode="r", newline="") as f:
            reader = csv.reader(f)
            next(reader)  # skip header
            for row in reader:
                timestamps.append(float(row[0]))
                currents.append(float(row[1]))

        if not timestamps:
            raise ValueError("Log file is empty or malformed.")

        t0 = timestamps[0]
        time_rel = [t - t0 for t in timestamps]

        plt.figure()
        plt.plot(time_rel, currents)
        plt.xlabel("Time (s)")
        plt.ylabel("RMS Current (A)")
        plt.title("Stepper Motor RMS Current")
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(output_path)
        plt.close()
