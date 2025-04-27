from dataclasses import dataclass
from .uart_registers import Register


@dataclass
class GCONFRegister(Register):
    """
    Parsed representation of the GCONF register (0x00).
    See TMC2208 datasheet Rev. 1.13, Table 5.1.
    """

    i_scale_analog: bool         # 1 = VREF pin active, 0 = internal current control (IHOLD/IRUN)
    internal_rsense: bool        # 1 = internal Rsense, 0 = external sense resistors
    en_spreadcycle: bool         # 1 = SpreadCycle, 0 = StealthChop
    shaft: bool                  # 1 = inverse motor direction
    index_otpw: bool             # 1 = INDEX = overtemperature warning
    index_step: bool             # 1 = INDEX = step signal
    pdn_disable: bool            # 1 = PDN/UART pin is ignored
    mstep_reg_select: bool       # 1 = microstep resolution set via MSTEP register
    multistep_filt: bool         # 1 = step pulse filter enabled
    test_mode: bool              # 1 = test mode enabled (do not use!)

    def as_dict(self) -> dict[str, bool]:
        return {
            "i_scale_analog": self.i_scale_analog,
            "internal_rsense": self.internal_rsense,
            "en_spreadcycle": self.en_spreadcycle,
            "shaft": self.shaft,
            "index_otpw": self.index_otpw,
            "index_step": self.index_step,
            "pdn_disable": self.pdn_disable,
            "mstep_reg_select": self.mstep_reg_select,
            "multistep_filt": self.multistep_filt,
            "test_mode": self.test_mode,
        }
    
    @classmethod
    def field_layout(cls) -> dict[str, tuple[int, int]]:
        """
        Returns the bit-layout of the GCONF register as a mapping from
        field names to (bit-position, bit-width).
        """
        return {
            "i_scale_analog":     (0, 1),
            "internal_rsense":    (1, 1),
            "en_spreadcycle":     (2, 1),
            "shaft":              (3, 1),
            "index_otpw":         (4, 1),
            "index_step":         (5, 1),
            "pdn_disable":        (6, 1),
            "mstep_reg_select":   (7, 1),
            "multistep_filt":     (8, 1),
            "test_mode":          (9, 1)
        }


@dataclass
class GSTATRegister(Register):
    """
    Parsed representation of the GSTAT register (0x01).
    See TMC2208 datasheet Rev. 1.13, Table 5.1.
    """

    reset: bool                  # 1 = Reset occurred
    drv_err: bool                # 1 = Driver error (overtemperature, short circuit, etc.)
    uv_cp: bool                  # 1 = Charge pump undervoltage

    def as_dict(self) -> dict[str, bool]:
        return {
            "reset": self.reset,
            "drv_err": self.drv_err,
            "uv_cp": self.uv_cp,
        }
    
    @classmethod
    def field_layout(cls) -> dict[str, tuple[int, int]]:
        pass


@dataclass
class IOINRegister(Register):
    """
    Parsed representation of the IOIN register (0x06).
    See TMC2208 datasheet Rev. 1.13, Table 5.1.
    """

    enn: bool                   # 0 = enabled, 1 = disabled
    ms1: bool                   # MS1 pin state
    ms2: bool                   # MS2 pin state
    diag: bool                  # Diagnostic output
    pdn_uart: bool              # PDN_UART pin state
    step: bool                  # STEP input level
    sel_a: bool                 # Current step resolution A (combined with sel_b)
    dir: bool                   # Direction input
    version: int                # Bits [31:24] indicate IC version (should be 0x21 for TMC2208)

    def as_dict(self) -> dict:
        return {
            "enn": self.enn,
            "ms1": self.ms1,
            "ms2": self.ms2,
            "diag": self.diag,
            "pdn_uart": self.pdn_uart,
            "step": self.step,
            "sel_a": self.sel_a,
            "dir": self.dir,
            "version": self.version,
        }
    
    @classmethod
    def field_layout(cls) -> dict[str, tuple[int, int]]:
        pass


@dataclass
class CHOPCONFRegister(Register):
    """
    Parsed representation of the CHOPCONF register (0x6C).
    See TMC2208 datasheet Rev. 1.13, Table 5.4.1
    """

    toff: int
    hstrt: int
    hend: int
    tbl: int
    vsense: bool
    mres: int
    intpol: bool
    dedge: bool
    diss2g: bool
    diss2vs: bool

    def as_dict(self) -> dict:
        return {
            "toff": self.toff,
            "hstrt": self.hstrt,
            "hend": self.hend,
            "tbl": self.tbl,
            "vsense": self.vsense,
            "mres": self.mres,
            "intpol": self.intpol,
            "dedge": self.dedge,
            "diss2g": self.diss2g,
            "diss2vs": self.diss2vs,
        }
    
    @classmethod
    def field_layout(cls) -> dict[str, tuple[int, int]]:
        """
        Returns the bit-layout of the CHOPCONF register as a mapping from
        field names to (bit-position, bit-width).
        """
        return {
            "toff":     (0, 4),
            "hstrt":    (4, 3),
            "hend":     (7, 4),
            "tbl":      (15, 2),
            "vsense":   (17, 1),
            "mres":     (24, 4),
            "intpol":   (28, 1),
            "dedge":    (29, 1),
            "diss2g":   (30, 1),
            "diss2vs":  (31, 1),
        }


@dataclass
class DRVSTATUSRegister(Register):
    """
    Parsed representation of the DRV_STATUS register (0x6F).
    See TMC2208 datasheet Rev. 1.13, Table 39 (p. 35).
    """

    stst: bool         # Standstill indicator
    olb: bool          # Open load on phase B
    ola: bool          # Open load on phase A
    s2gb: bool         # Short to GND on phase B
    s2ga: bool         # Short to GND on phase A
    s2vsb: bool        # Short to supply on phase B
    s2vsa: bool        # Short to supply on phase A
    otpw: bool         # Overtemperature prewarning
    ot: bool           # Overtemperature shutdown
    cs_actual: int     # Actual current control scaler (5 bits)
    stealth: bool      # StealthChop running indicator

    def as_dict(self) -> dict:
        return {
            "stst": self.stst,
            "olb": self.olb,
            "ola": self.ola,
            "s2gb": self.s2gb,
            "s2ga": self.s2ga,
            "s2vsb": self.s2vsb,
            "s2vsa": self.s2vsa,
            "otpw": self.otpw,
            "ot": self.ot,
            "cs_actual": self.cs_actual,
            "stealth": self.stealth,
        }
    
    @classmethod
    def field_layout(cls) -> dict[str, tuple[int, int]]:
        pass


@dataclass
class IHOLDIRUNRegister(Register):
    """
    Parsed representation of the IHOLD_IRUN register (0x10).
    See TMC2208 datasheet Rev. 1.13, Table 5.2.
    """

    ihold: int
    irun: int
    ihold_delay: int

    def as_dict(self) -> dict[str, int]:
        return {
            "ihold": self.ihold,
            "irun": self.irun,
            "ihold_delay": self.ihold_delay,
        }

    @classmethod
    def field_layout(cls) -> dict[str, tuple[int, int]]:
        return {
            "ihold":       (0, 5),   # bits 4..0
            "irun":        (8, 5),   # bits 12..8
            "ihold_delay": (16, 4),  # bits 19..16
        }
