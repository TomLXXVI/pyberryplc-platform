from typing import Optional
import serial
import time
from .uart_registers import Register
from .tmc2208_registers import (
    CHOPCONFRegister, 
    GCONFRegister, 
    GSTATRegister,
    IOINRegister,
    DRVSTATUSRegister,
    IHOLDIRUNRegister
)


class TMC2208UART:
    """
    UART communication helper for Trinamic TMC2208 driver.

    Supports register-level read and write access using the Trinamic UART protocol.

    This class is a context manager, ensuring the serial port is safely closed
    after use, even when exceptions occur.

    Parameters
    ----------
    port : str
        Serial port name (e.g., '/dev/ttyAMA0').
    baudrate : int, optional
        Baudrate for UART communication (default is 115200).
    timeout : float, optional
        Timeout in seconds for UART reads (default is 0.5).
    slave_address : int, optional
        Address of the TMC2208 slave device (default is 0x00).
    """
    REGISTER_CLASS_MAP = {
        "CHOPCONF": (0x6C, CHOPCONFRegister, "RW"),
        "GCONF": (0x00, GCONFRegister, "RW"),
        "GSTAT": (0x01, GSTATRegister, "RW"),
        "IOIN": (0x06, IOINRegister, "R"),
        "DRV_STATUS": (0x6F, DRVSTATUSRegister, "R"),
        "IHOLD_IRUN": (0x10, IHOLDIRUNRegister, "W")
    }
    
    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 0.5,
        slave_address: int = 0x00
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.slave_address = slave_address
        self.serial: Optional[serial.Serial] = None

    def open(self) -> None:
        """
        Opens the serial port if it is not already open.
        """
        if not self.serial or not self.serial.is_open:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )

    def close(self) -> None:
        """
        Closes the serial port if it is open.
        """
        if self.serial and self.serial.is_open:
            self.serial.close()

    def __enter__(self) -> "TMC2208UART":
        self.serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout
        )
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        if self.serial and self.serial.is_open:
            self.serial.close()
    
    @staticmethod
    def _calculate_crc(data: list[int]) -> int:
        crc = 0
        for byte in data:
            for _ in range(8):
                if (crc >> 7) ^ (byte & 0x01):
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
                byte >>= 1
        return crc

    def read_register_addr(self, reg_addr: int) -> int:
        """
        Sends a read request for the specified register and returns its value.
        
        This is a low-level function. For easier use, `read_register()`
        is recommended. 

        Parameters
        ----------
        reg_addr : int
            Address of the register to read.

        Returns
        -------
        int
            32-bit register value.

        Raises
        ------
        IOError
            If communication fails or an invalid response is received.
        """
        if not self.serial or not self.serial.is_open:
            raise IOError("Serial port is not open.")

        request = [0x05, self.slave_address, reg_addr & 0x7F]
        request.append(self._calculate_crc(request))

        self.serial.reset_input_buffer()   # clear RX-buffer
        self.serial.write(bytes(request))  # write request
        self.serial.flush()                # send full request
        time.sleep(0.005)
        response = self.serial.read(12)    # wait for answer 
        
        if len(response) < 12:
            raise IOError("Incomplete response received from driver.")

        response = response[4:]  # skip echo (first 4 bytes)
                
        if response[0] != 0x05:
            raise IOError(f"Invalid sync byte in response: 0x{response[0]:02X}")
        if response[1] != 0xFF:
            raise IOError(f"Invalid master address: 0x{response[1]:02X}")
        if response[2] != (reg_addr & 0x7F):
            raise IOError(f"Unexpected register address in response: 0x{response[2]:02X}")
        if self._calculate_crc(list(response[:7])) != response[7]:
            raise IOError("CRC check failed for received response.")

        value = (
            (response[3] << 24) |
            (response[4] << 16) |
            (response[5] << 8) |
            response[6]
        )
        return value

    def write_register_addr(self, reg_addr: int, value: int) -> None:
        """
        Writes a 32-bit value to the specified register.
        
        This is a low-level function. For easier use, `write_register()`
        is recommended. 
        
        Parameters
        ----------
        reg_addr : int
            Address of the register to write.
        value : int
            32-bit value to write into the register.

        Raises
        ------
        IOError
            If the serial port is not open.
        """
        if not self.serial or not self.serial.is_open:
            raise IOError("Serial port is not open.")

        datagram = [
            0x05,                    # Sync
            self.slave_address,      # Slave address
            reg_addr | 0x80,         # Write bit (MSB set)
            (value >> 24) & 0xFF,    # Data byte 1 (MSB)
            (value >> 16) & 0xFF,    # Data byte 2
            (value >> 8) & 0xFF,     # Data byte 3
            value & 0xFF             # Data byte 4 (LSB)
        ]
        crc = self._calculate_crc(datagram)
        datagram.append(crc)

        self.serial.write(bytes(datagram))
        self.serial.flush()

    def update_register_addr(self, reg_addr: int, mask: int, value: int) -> None:
        """
        Updates selected bits in a register using a read–modify–write operation.
        
        This is a low-level function. For easier use, `update_register()`
        is recommended. 
        
        Parameters
        ----------
        reg_addr : int
            Address of the register to update.
        mask : int
            Bitmask that defines which bits should be updated (1 = modify).
        value : int
            New bit values to apply (only bits under the mask are used).
    
        Raises
        ------
        IOError
            If reading or writing the register fails.
        """
        current = self.read_register_addr(reg_addr)
        new_value = (current & ~mask) | (value & mask)
        self.write_register_addr(reg_addr, new_value)

    def update_register(self, reg_name: str, fields: dict[str, int]) -> None:
        """
        Updates specific fields in a register by key, based on a layout
        defined in the associated dataclass.

        Parameters
        ----------
        reg_name : str
            Name of the register (e.g., "CHOPCONF").
        fields : dict[str, int]
            Mapping of field names to values (e.g., {"toff": 3, "mres": 5}).

        Raises
        ------
        ValueError
            If register or field key is unknown, or value is too large.
        IOError
            If reading or writing the register fails.
        """
        if reg_name not in self.REGISTER_CLASS_MAP:
            raise ValueError(f"Unknown register '{reg_name}'")

        addr, reg_class, access = self.REGISTER_CLASS_MAP[reg_name]
        if "W" not in access:
            raise IOError(f"Register '{reg_name}' is not writeable.")
        if "R" not in access:
            raise IOError(
                f"Register '{reg_name}' is not readable, "
                f"so it cannot be updated partially."
            )
        
        current = self.read_register(reg_name)
        layout = reg_class.field_layout()

        for key, new_value in fields.items():
            if key not in layout:
                raise ValueError(
                    f"Invalid field '{key}' for register '{reg_name}'"
                )
            width = layout[key][1]
            if isinstance(new_value, bool):
                new_value = int(new_value)

            if not (0 <= new_value < (1 << width)):
                raise ValueError(f"Value {new_value} is out of range for field '{key}'")

            setattr(current, key, new_value)

        # Write the modified `Register` object back to the driver's register
        self.write_register(reg_name, current)

    def read_register(self, reg_name: str) -> Register:
        """
        Reads the given register and returns an instance of the associated dataclass.

        Parameters
        ----------
        reg_name : str
            Name of the register (e.g., "GCONF", "CHOPCONF").

        Returns
        -------
        Register
            An instance of the register dataclass, e.g., GCONFRegister.

        Raises
        ------
        ValueError
            If the register or parser is unknown.
        IOError
            If the register cannot be read.
        """
        if reg_name not in self.REGISTER_CLASS_MAP:
            raise ValueError(f"Unknown register '{reg_name}'")

        addr, reg_class, access = self.REGISTER_CLASS_MAP[reg_name]
        if "R" not in access:
            raise IOError(f"Register '{reg_name}' is not readable")
        
        value = self.read_register_addr(addr)
        return reg_class.from_int(value)

    def write_register(self, reg_name: str, reg_obj: Register) -> None:
        """
        Writes the full contents of a register using the field values in a 
        dataclass.

        Parameters
        ----------
        reg_name : str
            Name of the register (e.g., "GCONF", "CHOPCONF").
        reg_obj : Register
            Instance of the corresponding register containing all field values.

        Raises
        ------
        ValueError
            If the register is unknown or the object doesn't match.
        IOError
            If writing to the register fails.
        """
        if reg_name not in self.REGISTER_CLASS_MAP:
            raise ValueError(f"Unknown register '{reg_name}'")

        addr, reg_class, access = self.REGISTER_CLASS_MAP[reg_name]
        if "W" not in access:
            raise IOError(f"Register '{reg_name}' is not writeable.")
        
        if not isinstance(reg_obj, reg_class):
            raise ValueError(
                f"Expected instance of {reg_class.__name__}, "
                f"got {type(reg_obj).__name__}"
            )

        self.write_register_addr(addr, reg_obj.to_int())
