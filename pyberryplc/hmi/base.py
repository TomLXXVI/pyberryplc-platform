import gc
from typing import Type
from abc import ABC, abstractmethod
from enum import StrEnum
import threading
import time
from pathlib import Path
from logging import Logger

from pyberryplc.core import SharedData, AbstractPLC


class PLCThreadManager:
    
    def __init__(self, hmi: 'AbstractHMI') -> None:
        self.hmi = hmi
        self._plc_class = hmi.plc_app
        self.shared_data = hmi.shared_data
        self.logger = hmi.logger
        self._plc_instance = None
        self._plc_fault: bool = False
        self._clean_exit: bool = False
    
    def _start_plc_thread(self) -> None:
        """Create the PLC application instance and start its scan cycle inside a
        separate thread.
        """
        def _plc_thread_fn():
            try:
                self._plc_instance = self._plc_class(
                    shared_data=self.shared_data,
                    logger=self.logger
                )
                self._plc_instance.run()
            except Exception as e:
                self.logger.exception(f"PLC start-up failed: {e}")
                self._plc_instance = None
                gc.collect()
                self._plc_fault = True

        self._plc_thread = threading.Thread(
            target=_plc_thread_fn,
            daemon=True
        )
        self._plc_thread.start()
    
    def _start_plc_monitor_thread(self) -> None:
        """Monitors the PLC thread inside a separate thread during operation and
        raises a PLC fault should the PLC thread suddenly die (i.e. when no 
        clean exit was requested).
        """
        def _plc_monitor_thread_fn():
            while True:
                if not self._plc_thread.is_alive() and not self._clean_exit:
                    self.logger.error("PLC thread has crashed!")
                    self._plc_fault = True
                    break
                time.sleep(2)

        self._plc_monitor_thread = threading.Thread(
            target=_plc_monitor_thread_fn,
            daemon=True
        )
        self._plc_monitor_thread.start()
    
    def _plc_watchdog_fn(self) -> None:
        if self._plc_fault:
            self.hmi._plc_crash_dialog.open()
            
    def start_plc(self) -> None:
        """Launch PLC application."""
        self._start_plc_thread()
        self._start_plc_monitor_thread()
        self.hmi.ui.timer(1.0, self._plc_watchdog_fn)  # PLC watchdog
    
    def restart_plc(self) -> None:
        """Callback to restart the PLC application after a crash."""
        self.logger.warning("Restart requested from HMI")
        self._plc_fault = False  # reset plc fault flag and try to restart
        self._start_plc_thread()
        if not self._plc_monitor_thread.is_alive():
            self._start_plc_monitor_thread()
    
    def exit_plc(self) -> None:
        self._clean_exit = True
        self._plc_instance.exit()


class AbstractHMI(ABC):
    """
    Abstract base class for a NiceGUI-based HMI.

    This class defines the interface and common functionality shared by all
    concrete HMI implementations.
    """
    class Pages(StrEnum):
        HMI_PAGE = "hmi_page"
        LOG_PAGE = "log_page"
        SHUTDOWN_PAGE = "shutdown_page"
    
    def __init__(
        self,
        title: str,
        app,
        ui,
        shared_data: SharedData,
        plc_app: Type[AbstractPLC] | None,
        logger: Logger,
        port: int = 8081,
    ) -> None:
        """
        Initialize the HMI.

        Parameters
        ----------
        title : str
            Name for the HMI app.
        shared_data : SharedData
            The shared object for data exchange between PLC and HMI.
        plc_app : AbstractPLC
            PLC application class, i.e. the concrete class inherited from 
            `AbstractPLC`.
        logger : 
            Logger instance that will be attached to the PLC and HMI application
            instance.
        port: int
            Port the HMI server is listening to. Default port is 8081.
        """
        self.title = title
        self.app = app
        self.ui = ui
        self.shared_data = shared_data
        self.plc_app = plc_app
        self.logger = logger
        self.port = port
        
        self._plc_manager = PLCThreadManager(self) if self.plc_app else None
        self._plc_crash_dialog = None
        self.clean_exit: bool = False
        self.exit_flag: bool = False
        self._active_page: str = ""
        self.shutdown_flag: bool = False
        
        self._main_area: ui.column | None = None
        self._logger_area: ui.column | None = None
        self.status_html: ui.html | None = None
        
        self._log_level_colors = {
            "DEBUG": "#888",
            "INFO": "#000",
            "WARNING": "#e69f00",
            "ERROR": "#d62728",
            "CRITICAL": "#cc0000"
        }
        self._last_log_snapshot: str = ""

    @abstractmethod
    def build_gui(self) -> None:
        """
        Build the HMI user interface.

        This abstract method must be implemented by all subclasses to define
        the layout and controls of the HMI.
        """
        pass

    def update_status(self) -> None:
        """
        Update HMI status indicators.

        This method is called every 0.5 seconds. Subclasses may override
        it to reflect PLC state or I/O conditions.
        """
        pass

    def setup_timers(self) -> None:
        """
        Start recurring timers for GUI updates.

        By default, creates two periodic timers:
        - Log updates every 1.0 second
        - Status updates every 0.5 seconds
        """
        if self._plc_manager:
            self.ui.timer(0.5, self._plc_manager.start_plc, once=True)
            self.ui.timer(1.0, self._update_log_page)
            self.ui.timer(0.5, self._update_status)

    @staticmethod
    def set_button_state(button, enabled: bool) -> None:
        """
        Enable or disable a button and apply visual styling.

        Parameters
        ----------
        button : ui.button
            The NiceGUI button instance to modify.
        enabled : bool
            Whether the button should be enabled (True) or disabled (False).
        """
        if enabled:
            button.enable()
        else:
            button.disable()

    def run(self) -> None:
        """Launches PLC and HMI application."""
        @self.ui.page("/")
        async def hmi_page():
            self._build_hmi_page()

        @self.ui.page("/log")
        async def log_page():
            self._build_log_page()
        
        @self.ui.page("/shutdown")
        async def shutdown_page():
            self._build_shutdown_page()
        
        self.setup_timers()
        self.ui.run(title=self.title, port=self.port, reload=False, show=False)

    async def _exit_hmi(self) -> None:
        """Callback to cleanly close the PLC and HMI application."""
        exit_dialog = ExitDialog(self)
        await exit_dialog.show()
        if self.exit_flag:
            self.ui.navigate.to("/shutdown")
            self.logger.info("Shutting down PLC and HMI...")
            if self._plc_manager:
                self._plc_manager.exit_plc()
            self.ui.timer(2.0, self.app.shutdown, once=True)
    
    def _build_navbar(self):
        with self.ui.header().classes("bg-gray-200 p-2"):
            self._plc_crash_dialog = PLCCrashDialog(self)
            
            # Navigation buttons
            with self.ui.row().classes("w-full justify-between items-center"):
                with self.ui.row().classes("gap-4"):
                    self.ui.button("HMI", on_click=lambda: self.ui.navigate.to("/"))
                    self.ui.button("Log", on_click=lambda: self.ui.navigate.to("/log"))
                self.ui.button("Exit", color="red", on_click=self._exit_hmi)
            
            # Status bar
            self.status_html = self.ui.html('<b>Status</b>').classes(
                "w-full bg-white text-black text-sm px-2 py-1 rounded"
            )
            
    def _build_hmi_page(self) -> None:
        self._active_page = self.Pages.HMI_PAGE
        self._build_navbar()
        self._main_area = self.ui.column().classes("w-full p-4")
        with self._main_area:
            self.build_gui()
    
    def _build_log_page(self) -> None:
        self._active_page = self.Pages.LOG_PAGE
        self._build_navbar()
        with self.ui.column().classes("w-full"):
            self.ui.label("Log output").classes("text-xl p-2")
            self._logger_area = self.ui.column().classes(
                "w-full h-[300px] overflow-auto bg-white border rounded p-2"
            ).props("id=log_area")
        log_html = self._read_log_file()
        with self._logger_area:
            self.ui.html(
                f'<pre style="font-family: monospace; font-size: 13px; '
                f'margin: 0; padding: 0; line-height: 1.1;">'
                f'{log_html}'
                f'</pre>'
            )
    
    def _build_shutdown_page(self) -> None:
        with self.ui.column() \
            .classes(
            "absolute-center "
            "items-center "
            "gap-4 "
            "bg-gray-100 "
            "p-6 "
            "rounded-lg "
            "shadow-md"
        ):
            self.ui.label("Application Terminated").classes("text-2xl font-bold")
            self.ui.label("The PLC and HMI application have been shut down.")
            self.ui.label("You may now close this browser window.")
    
    def _read_log_file(self) -> str | None:
        log_file = Path("logs/plc.log")
        if not log_file.exists():
            return None
        last_lines = log_file.read_text(encoding="utf-8").splitlines()[-20:]
        colored_lines = []
        for line in last_lines:
            color = "#000"  # default black
            for level, level_color in self._log_level_colors.items():
                if f"[{level}]" in line:
                    color = level_color
                    break
            colored_lines.append(f'<span style="color: {color};">{line}</span>')
        log_html = "<br>".join(colored_lines)
        return log_html
    
    def _update_log_page(self) -> None:
        """
        Display the latest lines of the PLC log in the log area.
        """
        if self._active_page == self.Pages.LOG_PAGE:
            log_html = self._read_log_file()
            if log_html == self._last_log_snapshot:
                return
            self._last_log_snapshot = log_html
            try:
                self._logger_area.clear()
            except AttributeError:
                return
            with self._logger_area:
                self.ui.html(
                    f'<pre style="font-family: monospace; font-size: 13px; '
                    f'margin: 0; padding: 0; line-height: 1.1;">'
                    f'{log_html}'
                    f'</pre>'
                )
            self.ui.run_javascript(
                "document.getElementById('log_area').scrollTop = "
                "document.getElementById('log_area').scrollHeight;"
            )
    
    def _update_status(self) -> None:
        if self.status_html is not None:
            self.update_status()


class PLCCrashDialog:

    def __init__(self, hmi: AbstractHMI) -> None:
        self.hmi = hmi
        self._create_dialog()

    def _create_dialog(self) -> None:
        self._dialog = self.hmi.ui.dialog().classes("w-[400px]")
        with self._dialog:
            with self.hmi.ui.card():
                self.hmi.ui.label(
                    "⚠️ A critical PLC error has occurred. "
                    "Please check the log for details."
                ).classes("text-red-600 font-bold")
                with self.hmi.ui.row().classes("mt-4 justify-end"):
                    self.hmi.ui.button(
                        "Turn-off PLC",
                        on_click=self._abort_hmi
                    )
                    self.hmi.ui.button(
                        "Restart PLC",
                        color="red",
                        on_click=self._restart_plc
                    )

    def _abort_hmi(self) -> None:
        self.close()
        self.hmi.logger.warning("PLC operation has been aborted by the user...")
        self.hmi.app.shutdown()

    def _restart_plc(self) -> None:
        self.close()
        if self.hmi._plc_manager:
            self.hmi._plc_manager.restart_plc()

    def open(self) -> None:
        self._dialog.open()

    def close(self) -> None:
        self._dialog.close()


class ErrorDialog:

    def __init__(self, hmi: AbstractHMI, message: str) -> None:
        self.hmi = hmi
        self.message = message
        self._create_dialog()

    def _create_dialog(self) -> None:
        self._dialog = self.hmi.ui.dialog().classes("w-[400px]")
        with self._dialog:
            with self.hmi.ui.card():
                self.hmi.ui.label(f"⚠️ Error").classes("text-red-600 font-bold")
                self.hmi.ui.label(self.message).classes("font-bold")
                with self.hmi.ui.row().classes("mt-4 justify-center"):
                    self.hmi.ui.button(
                        "Close",
                        color="red",
                        on_click=self.close
                    )

    def open(self) -> None:
        self._dialog.open()

    def close(self) -> None:
        self._dialog.close()


class ExitDialog:
    
    def __init__(self, hmi: AbstractHMI) -> None:
        self.hmi = hmi
        self._create_dialog()
    
    def _create_dialog(self) -> None:
        self._dialog = self.hmi.ui.dialog().classes("w-[400px]")
        with self._dialog:
            with self.hmi.ui.card():
                self.hmi.ui.label(
                    "⚠️ Shutdown HMI?"
                ).classes("text-red-600 font-bold")
                with self.hmi.ui.row().classes("mt-4 justify-end"):
                    self.hmi.ui.button(
                        "Cancel",
                        on_click=lambda: self._dialog.submit("Cancel")
                    )
                    self.hmi.ui.button(
                        "Ok",
                        color="red",
                        on_click=lambda: self._dialog.submit("Ok")
                    )
    
    async def show(self):
        result = await self._dialog
        if result == "Cancel":
            self.hmi.exit_flag = False
        else:
            self.hmi.exit_flag = True
