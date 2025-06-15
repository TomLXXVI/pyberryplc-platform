from abc import ABC, abstractmethod
import time
import logging
import socket
import json
import serial


class AbstractRemoteDeviceClient(ABC):
    """
    Abstract base class for remote device clients.

    Defines a unified interface for sending commands, receiving responses,
    and managing connections to remote devices, regardless of the underlying
    communication protocol (e.g., TCP/IP or serial).
    """
    @abstractmethod
    def connect(self) -> None:
        """Establishes a connection to the remote device."""
        pass

    @abstractmethod
    def send_command(self, command_dict: dict) -> None:
        """Sends a command dictionary (as JSON) to the remote device."""
        pass

    @abstractmethod
    def wait_for_done(self) -> None:
        """Waits for a response from the remote device indicating completion."""
        pass

    @abstractmethod
    def shutdown(self) -> None:
        """Sends a shutdown command to the remote device."""
        pass

    @abstractmethod
    def close(self) -> None:
        """Closes the connection to the remote device."""
        pass


class TCPRemoteDeviceClient(AbstractRemoteDeviceClient):
    """
    Remote device client for TCP/IP socket communication.

    Provides connection management, command sending, and response handling
    over a TCP/IP socket with JSON-formatted messages.
    """
    def __init__(
        self,
        host: str = 'localhost',
        port: int = 65432,
        logger: logging.Logger | None = None,
        timeout: float = 5,
        max_retries: int = 3,
        retry_delay: float = 2
    ) -> None:
        """
        Initializes the TCP remote device client.

        Parameters
        ----------
        host : str
            IP address or hostname of the remote device.
        port : int
            TCP port on which the remote device is listening.
        logger : logging.Logger | None
            Logger or None to use print.
        timeout : float
            Timeout in seconds for socket operations.
        max_retries : int
            Number of retries if connection fails.
        retry_delay : float
            Delay in seconds between retry attempts.
        """
        self.host: str = host
        self.port: int = port
        self.logger: logging.Logger | None = logger
        self.timeout: float = timeout
        self.max_retries: int = max_retries
        self.retry_delay: float = retry_delay
        self.socket:socket.socket | None = None
        self.stream: socket.SocketIO | None = None

    def connect(self) -> None:
        """
        Establishes a TCP connection to the remote device with retry logic.
        """
        attempt = 0
        while attempt < self.max_retries:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(self.timeout)
                self.socket.connect((self.host, self.port))
                self.stream = self.socket.makefile('r')  
                # File-like object that can be read. Used for receiving the
                # responses from the remote device.
                self.socket.settimeout(None)
                self._log(f"Connected to device at {self.host}:{self.port}")
                return
            except Exception as e:
                attempt += 1
                self._log(f"Connection attempt {attempt} failed: {e}")
                time.sleep(self.retry_delay)
        raise ConnectionError("Failed to connect to remote device after multiple attempts.")

    def send_command(self, command_dict: dict) -> None:
        """
        Sends a JSON-encoded command to the remote device.
        """
        msg = json.dumps(command_dict) + "\n"  # `\n` indicates the end of a message.
        self.socket.sendall(msg.encode())  # Convert JSON string to bytes and send it to the remote device.

    def wait_for_done(self) -> None:
        """
        Waits for the response from the remote device after an execution command
        was sent. 

        Expects a JSON message with 'status': 'done' to confirm completion.
        Raises an error if 'status': 'error' or timeout occurs.
        """
        start_time = time.time()
        while True:
            if time.time() - start_time > self.timeout:
                raise TimeoutError(
                    "Timed out waiting for response from remote device."
                )
            # Read a text line from the stream (i.e a string terminated with `\n`).
            line = self.stream.readline()
            if not line:
                raise ConnectionError(
                    "Connection to remote device was closed unexpectedly."
                )
            # Convert the text line to a dictionary.
            response = json.loads(line) 
            if response.get("status") == "done":
                return
            elif response.get("status") == "error":
                raise RuntimeError(f"Error from remote device: {response.get('message')}")

    def shutdown(self) -> None:
        """
        Sends a shutdown command to the remote device.
        """
        try:
            self.send_command({"command": "shutdown"})
        except:
            self._log("Failed to send shutdown command.", level=logging.ERROR)

    def close(self) -> None:
        """
        Closes the TCP socket connection.
        """
        try:
            if self.socket:
                self.socket.close()
        except:
            pass
        self._log("Connection to remote device closed.")

    def _log(self, msg: str, level: int = logging.INFO) -> None:
        """
        Logs a message using the configured logger.
        """
        if isinstance(self.logger, logging.Logger):
            # noinspection PyUnreachableCode
            match level:
                case logging.DEBUG:
                    self.logger.debug(msg)
                case logging.WARNING:
                    self.logger.warning(msg)
                case logging.ERROR:
                    self.logger.error(msg)
                case logging.CRITICAL:
                    self.logger.critical(msg)
                case _:
                    self.logger.info(msg)
        else:
            print(msg)


class SerialRemoteDeviceClient(AbstractRemoteDeviceClient):
    """
    Remote device client for serial communication using `pyserial`.

    Provides connection management, command sending, and response handling
    over a serial interface using JSON-formatted messages.
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 9600,
        logger: logging.Logger | None = None,
        timeout: float = 5
    ) -> None:
        """
        Initializes the serial remote device client.

        Parameters
        ----------
        port : str
            Serial port (e.g., '/dev/ttyUSB0' or 'COM3').
        baudrate : int
            Communication speed in baud.
        logger : logging.Logger | None
            Logger or None to use print.
        timeout : float
            Timeout in seconds for response waiting.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.logger: logging.Logger | None = logger
        self.ser: serial.Serial | None = None

    def connect(self) -> None:
        """
        Opens the serial port connection.
        """
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self._log(f"Connected to serial device at {self.port} (baudrate {self.baudrate})")
        except Exception as e:
            raise ConnectionError(f"Failed to open serial port: {e}")

    def send_command(self, command_dict: dict) -> None:
        """
        Sends a JSON-encoded command to the serial device.
        """
        msg = json.dumps(command_dict) + "\n"
        self.ser.write(msg.encode())

    def wait_for_done(self) -> None:
        """
        Waits for a response from the serial device.

        Expects a JSON message with 'status': 'done' to confirm completion.
        Raises an error if 'status': 'error' or timeout occurs.
        """
        start_time = time.time()
        while True:
            if time.time() - start_time > self.timeout:
                raise TimeoutError("Timed out waiting for response from serial device.")
            line = self.ser.readline().decode().strip()
            if not line:
                continue
            response = json.loads(line)
            if response.get("status") == "done":
                return
            if response.get("status") == "error":
                raise RuntimeError(f"Error from serial device: {response.get('message')}")

    def shutdown(self) -> None:
        """
        Sends a shutdown command to the serial device.
        """
        try:
            self.send_command({"command": "shutdown"})
        except:
            self._log("Failed to send shutdown command.", level=logging.ERROR)

    def close(self) -> None:
        """
        Closes the serial port connection.
        """
        try:
            if self.ser:
                self.ser.close()
        except:
            pass
        self._log("Serial connection closed.")

    def _log(self, msg: str, level: int = logging.INFO, *args, **kwargs) -> None:
        """
        Logs a message using the configured logger.
        """
        if isinstance(self.logger, logging.Logger):
            # noinspection PyUnreachableCode
            match level:
                case logging.DEBUG:
                    self.logger.debug(msg, *args, **kwargs)
                case logging.WARNING:
                    self.logger.warning(msg, *args, **kwargs)
                case logging.ERROR:
                    self.logger.error(msg, *args, **kwargs)
                case logging.CRITICAL:
                    self.logger.critical(msg, *args, **kwargs)
                case _:
                    self.logger.info(msg, *args, **kwargs)
        else:
            print(msg % args if args else msg)
