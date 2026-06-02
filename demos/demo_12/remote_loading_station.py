import logging
import threading
import time
from enum import StrEnum
from typing import Any

from pyberryplc.utils.remote_interface_server import TCPRemoteDeviceServer
from pyberryplc.core import TimerOffDelay


class Status(StrEnum):
    OFF = "off"
    READY = "ready"
    BUSY = "busy"
    DONE = "done"
    ERROR = "error"


class Command(StrEnum):
    GET_STATUS = "get_status"
    START_LOADING = "start_loading"
    SHUTDOWN = "shutdown"


class LoadingStation(TCPRemoteDeviceServer):

    def __init__(
        self,
        logger: logging.Logger,
        host: str = "0.0.0.0",
        port: int = 65432,
    ) -> None:
        super().__init__(logger, host, port)
        self._status = Status.OFF
        self.timer_loading = TimerOffDelay(10)

    def initialize(self) -> None:
        self._status = Status.READY

    def handle_command(self, command: dict[str, str]) -> dict[str, Any]:
        command_ = command.get("command")
        step_id = command.get("step", "")
        match command_:
            case Command.GET_STATUS:
                return self._get_status(step_id)
            case Command.START_LOADING:
                return self._start_loading_cycle()
            case Command.SHUTDOWN:
                return self._shutdown()
            case _:
                return {"status": Status.ERROR, "message": "Unknown command."}

    def _get_status(self, step_id: str) -> dict[str, Any]:
        match step_id:
            case "S11":
                self._status = Status.READY
                response = {"status": self._status, "message": "Ready to go."}
                return response
            case "S13":
                if self._status == Status.BUSY:
                    message = "Still busy."
                elif self._status == Status.DONE:
                    message = "Done loading."
                else:
                    message = "I don't know what I'm doing."
                response = {"status": self._status, "message": message}
                return response
            case _:
                self._status = Status.ERROR
                response = {"status": self._status, "message": "Unknown step."}
                return response

    def _run_loading_cycle(self) -> None:
        while self.timer_loading.running:
            time.sleep(0.01)
        self.timer_loading.reset()
        self._status = Status.DONE

    def _start_loading_cycle(self) -> dict[str, Any]:
        self._status = Status.BUSY
        threading.Thread(target=self._run_loading_cycle, daemon=True).start()
        return {"status": self._status, "message": "Loading cycle started."}

    def _shutdown(self) -> dict[str, Any]:
        return {"status": self._status, "message": "Shutting down."}


def main():
    from pyberryplc.utils.remote_interface_server import init_logger

    logger = init_logger(
        name="LOADING STATION",
        log_dir="/shared/python-projects/pyberryplc-platform/demos/demo_12/logs",
        file_name="loading_station"
    )

    loading_station = LoadingStation(logger, "localhost", 65432)
    loading_station.run()


if __name__ == '__main__':
    main()
