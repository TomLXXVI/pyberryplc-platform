import logging

from pyberryplc.remote_device import TCPRemoteDeviceClient

from remote_loading_station import Command, Status


class LoadingStation(TCPRemoteDeviceClient):

    def _request(self, command: Command) -> tuple[str, str]:
        try:
            super().send_command({"command": command})
            response = self.wait_for_response()
        except Exception as e:
            return Status.ERROR, str(e)

        status = str(response.get("status"))
        message = str(response.get("message"))
        return status, message

    def check_operational_state(self) -> tuple[str, str]:
        return self._request(Command.CHECK_OPERATIONAL_STATE)

    def start_loading(self) -> tuple[str, str]:
        return self._request(Command.START_LOADING)

    def get_loading_progress(self) -> tuple[str, str]:
        return self._request(Command.GET_LOADING_PROGRESS)

    def emergency_stop(self) -> tuple[str, str]:
        return self._request(Command.EMERGENCY_STOP)

    def reset(self) -> tuple[str, str]:
        return self._request(Command.RESET)

    def shutdown(self) -> tuple[str, str]:
        return self._request(Command.SHUTDOWN)
