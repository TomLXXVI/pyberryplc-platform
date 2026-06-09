from typing import Any
import logging
from pathlib import Path
import threading
import time
from enum import StrEnum
import random


from pyberryplc.utils.remote_interface_server import TCPRemoteDeviceServer
from pyberryplc.core import TimerOffDelay


class Status(StrEnum):
    OFF = "off"
    READY = "ready"
    BUSY = "busy"
    DONE = "done"
    ERROR = "error"
    TIMEOUT = "timeout"


class Command(StrEnum):
    CHECK_OPERATIONAL_STATE = "check_operational_state"
    START_LOADING = "start_loading"
    GET_LOADING_PROGRESS = "get_loading_progress"
    SHUTDOWN = "shutdown"


class LoadingStation(TCPRemoteDeviceServer):

    def __init__(
        self,
        logger: logging.Logger,
        host: str = "0.0.0.0",
        port: int = 65432,
    ) -> None:
        super().__init__(logger, host, port)
        self.timer_loading = TimerOffDelay(10)
        self.status = Status.OFF

    def initialize(self) -> None:
        pass

    def handle_command(self, command: dict[str, str]) -> dict[str, Any]:
        command_ = command.get("command")
        match command_:
            case Command.CHECK_OPERATIONAL_STATE:
                return self._check_operational_state()
            case Command.START_LOADING:
                return self._start_loading()
            case Command.GET_LOADING_PROGRESS:
                return self._get_loading_progress()
            case Command.SHUTDOWN:
                return self._shutdown()
            case _:
                return {"status": Status.ERROR, "message": "Unknown command."}

    def _response(self, message: str) -> dict[str, Any]:
        return {"status": self.status, "message": message}

    def _check_operational_state(self) -> dict[str, Any]:
        self.status = random.choices([Status.READY, Status.ERROR], weights=[0.8, 0.2])[0]
        if self.status == Status.READY:
            return self._response("Ready to go.")
        else:
            return self._response("Operational trouble.")

    def _run_loading_cycle(self) -> None:
        timeout_secs = self.timer_loading.dt
        cycle_duration = random.uniform(2.0, 12.0)
        error_after = (
            random.uniform(0.5, timeout_secs)
            if random.random() < 0.2
            else None
        )

        t_start = time.perf_counter()
        self.status = Status.BUSY

        while self.timer_loading.running:
            elapsed = time.perf_counter() - t_start

            if error_after is not None and elapsed >= error_after:
                self.status = Status.ERROR
                self.timer_loading.reset()
                return

            if elapsed >= cycle_duration:
                self.status = Status.DONE
                self.timer_loading.reset()
                return

            time.sleep(0.1)

        self.status = Status.TIMEOUT
        self.timer_loading.reset()

    def _start_loading(self) -> dict[str, Any]:
        self.status = Status.BUSY
        threading.Thread(target=self._run_loading_cycle, daemon=True).start()
        return self._response("Loading cycle started.")

    def _get_loading_progress(self) -> dict[str, Any]:
        match self.status:
            case Status.BUSY:
                return self._response("Still busy with this loading cycle.")
            case Status.DONE:
                return self._response("Loading cycle done.")
            case Status.TIMEOUT:
                return self._response("Loading cycle timeout.")
            case Status.ERROR:
                return self._response("Something went wrong during loading.")
            case _:
                return self._response("Loading cycle progression unknown.")

    def _shutdown(self) -> dict[str, Any]:
        return {"status": self.status, "message": "Shutting down."}


def main():
    from pyberryplc.utils.remote_interface_server import init_logger

    demo_dir = Path(__file__).resolve().parent
    logger = init_logger(
        name="LOADING STATION",
        log_dir=str(demo_dir / "logs"),
        file_name="loading_station"
    )

    loading_station = LoadingStation(logger, "localhost", 65432)
    loading_station.initialize()
    loading_station.run()


if __name__ == '__main__':
    main()
