import logging

from pyberryplc.utils.remote_interface_client import TCPRemoteDeviceClient

from remote_loading_station import Command, Status


class LoadingStation(TCPRemoteDeviceClient):

    def handle_response(self) -> tuple[str, str]:
        try:
            response = self.wait_for_response()
        except Exception as e:
            message = f"Error while waiting for response: {e}"
            return Status.ERROR, message
        else:
            status = str(response.get("status"))
            message = str(response.get("message"))
            return status, message

    def get_status(self, step_id: str) -> None:
        try:
            self.send_command({"command": Command.GET_STATUS, "step": step_id})
        except Exception as e:
            self._log(f"Sending of command failed: {e}", level=logging.ERROR)

    def start(self) -> None:
        try:
            self.send_command({"command": Command.START_LOADING})
        except Exception as e:
            self._log(f"Sending of command failed: {e}", level=logging.ERROR)
