from typing import Any

import os
import logging
from pathlib import Path
from datetime import datetime
import json
import socket
from abc import ABC, abstractmethod


def init_file_logger(name: str, log_dir: str, file_name: str) -> logging.Logger:
    os.makedirs(log_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d")
    logfile_path = Path(log_dir) / f"{file_name}_{timestamp}.log"
    logging.basicConfig(
        filename=logfile_path,
        level=logging.INFO,
        format="[%(name)s %(asctime)s | %(levelname)s] %(message)s",
        datefmt="%d-%m-%Y %H:%M:%S",
        filemode='a'
    )
    return logging.getLogger(name)


class TCPRemoteDeviceServer(ABC):

    def __init__(
        self,
        logger: logging.Logger,
        host: str = "0.0.0.0",
        port: int = 65432,
    ) -> None:
        self.host = host
        self.port = port
        self.logger = logger

    @abstractmethod
    def initialize(self) -> None:
        ...

    @abstractmethod
    def handle_command(self, command: dict[str, str]) -> dict[str, Any]:
        ...

    def run(self) -> None:
        self._message_loop()

    def _message_loop(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            # Establish TCP communication with the master.
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port))
            s.listen(1)
            self.logger.info(f"Slave listening on {self.host}:{self.port}...")

            # Wait for the master to connect.
            conn, addr = s.accept()

            with conn:
                self.logger.info(f"Connected to master: {addr}")

                # Enter the master-slave message loop.
                stream = conn.makefile("r")
                while True:
                    line = stream.readline()
                    if not line:
                        break
                    command = {}

                    # Handle a command received from the master and send
                    # a status response back to the master.
                    try:
                        # Convert bytes to string, parse the JSON string and
                        # convert it to a dictionary.
                        command = json.loads(line)
                        response = self.handle_command(command)
                    except Exception as e:
                        response = {"status": "error", "message": str(e)}

                    conn.sendall((json.dumps(response) + "\n").encode())  # `\n` indicates the end of a message.

                    # If the master sent a shutdown command, the message loop
                    # and, consequently, the TCP connection with the master are
                    # broken.
                    if command.get("command") == "shutdown":
                        self.logger.warning("Shutdown received. Closing connection.")
                        break
