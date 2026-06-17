import logging

from pyberryplc.remote_device.remote_interface_client import TCPRemoteDeviceClient

from remote_loading_station import Command, Status


class LoadingStation(TCPRemoteDeviceClient):

    def get_response(self) -> tuple[str, str]:
        try:
            response = self.wait_for_response()
        except Exception as e:
            message = str(e)
            return Status.ERROR, message
        else:
            status = str(response.get("status"))
            message = str(response.get("message"))
            return status, message

    def send_command(self, command: Command) -> None:
        try:
            super().send_command({"command": command})
        except Exception as e:
            self._log(f"Sending of command failed: {e}", level=logging.ERROR)

    def check_operational_state(self) -> None:
        self.send_command(Command.CHECK_OPERATIONAL_STATE)

    def start_loading(self) -> None:
        self.send_command(Command.START_LOADING)

    def get_loading_progress(self) -> None:
        self.send_command(Command.GET_LOADING_PROGRESS)

    def emergency_stop(self) -> None:
        self.send_command(Command.EMERGENCY_STOP)

    def reset(self) -> None:
        self.send_command(Command.RESET)

    def shutdown(self) -> None:
        self.send_command(Command.SHUTDOWN)
