import gc
from typing import Type
from abc import ABC, abstractmethod
import threading
import time
from pathlib import Path
from logging import Logger

from nicegui import ui, app

from pyberryplc.core import SharedData, AbstractPLC


class AbstractHMI(ABC):
    """
    Abstract base class for a NiceGUI-based HMI.

    This class defines the interface and common functionality shared by all
    concrete HMI implementations.
    """

    def __init__(
        self, 
        title: str,
        shared_data: SharedData,
        plc_app: Type[AbstractPLC],
        logger: Logger, 
        port: int = 8081
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
        self.shared_data = shared_data
        self._plc_class = plc_app
        self._plc_instance = None
        self.port = port

        self._plc_thread: threading.Thread | None = None
        self._plc_monitor_thread: threading.Thread | None = None
        self._plc_fault: bool = False
        self._plc_crash_dialog = None
        self._clean_exit: bool = False
        
        self._main_area = ui.column().classes("w-full")

        self.logger = logger
        self._log_level_colors = {
            "DEBUG": "#888",
            "INFO": "#000",
            "WARNING": "#e69f00",
            "ERROR": "#d62728",
            "CRITICAL": "#cc0000"
        }
        self._log_area = None
        self._last_log_snapshot: list[str] = []

    @abstractmethod
    def build_ui(self) -> None:
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
        ui.timer(1.0, self._update_logs)
        ui.timer(0.5, self.update_status)

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
        """Launches the PLC and HMI applications."""
        # Build the HMI GUI.
        self._build_ui()
        self.setup_timers()
        
        # Start PLC and HMI.
        self._start_plc()
        ui.run(title=self.title, port=self.port, reload=False, show=False)

    def restart_plc(self) -> None:
        """Callback to restart the PLC application after a crash."""
        self.logger.warning("Restart requested from HMI")
        self._plc_fault = False  # reset plc fault flag and try to restart
        self._start_plc_thread()
        if not self._plc_monitor_thread.is_alive():
            self._start_plc_monitor_thread()

    def exit_hmi(self) -> None:
        """Callback to cleanly close the PLC and HMI application."""
        # Close PLC app.
        self.logger.info("Clean exit PLC")
        self._clean_exit = True
        self._plc_instance.exit()
        
        # Show shutdown message.
        time.sleep(0.2)  # give PLC thread a moment to stop cleanly
        self._main_area.clear()
        self._main_area.classes("w-full items-center p-4")
        with self._main_area:
            ui.label("✅ The PLC has been stopped.").classes("text-2xl")
        
        # Close the HMI-app (and server) after 2 seconds.
        ui.timer(2.0, app.shutdown, once=True)
    
    def abort_hmi(self) -> None:
        """Callback to terminate the PLC and HMI application after a crash."""
        self.logger.warning("PLC operation aborted")
        app.shutdown()
    
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
            self._plc_crash_dialog.open()
    
    def _start_plc(self) -> None:
        """Launch PLC application."""
        self._start_plc_thread()
        self._start_plc_monitor_thread()
        ui.timer(1.0, self._plc_watchdog_fn)  # PLC watchdog
    
    def _build_ui(self) -> None:
        self._plc_crash_dialog = PLCCrashDialog(self)
        with self._main_area:
            self.build_ui()
        self._log_area = ui.column().classes(
            "w-full p-2 bg-white border rounded max-h-64 overflow-auto"
        ).props("id=log_area")
    
    def _update_logs(self) -> None:
        """
        Display the latest lines of the PLC log in the log area.
        """
        log_file = Path("logs/plc.log")
        if not log_file.exists():
            return
        
        last_lines = log_file.read_text(encoding="utf-8").splitlines()[-20:]
        if last_lines == self._last_log_snapshot:
            return
        
        self._last_log_snapshot = last_lines
        colored_lines = []
        for line in last_lines:
            color = "#000"  # default black
            for level, level_color in self._log_level_colors.items():
                if f"[{level}]" in line:
                    color = level_color
                    break
            colored_lines.append(f'<span style="color: {color};">{line}</span>')

        log_html = "<br>".join(colored_lines)

        self._log_area.clear()
        with self._log_area:
            ui.html(
                f'<pre style="font-family: monospace; font-size: 13px; '
                f'margin: 0; padding: 0; line-height: 1.1;">'
                f'{log_html}'
                f'</pre>'
            )

        ui.run_javascript(
            "document.getElementById('log_area').scrollTop = "
            "document.getElementById('log_area').scrollHeight;"
        )

class PLCCrashDialog:
    
    def __init__(self, parent: AbstractHMI):
        self.parent = parent
        self._create_dialog()
    
    def _create_dialog(self):
        self._dialog = ui.dialog().classes("w-[400px]")
        with self._dialog:
            with ui.card():
                ui.label(
                    "⚠️ A critical PLC error has occurred. "
                    "Please check the log for details."
                ).classes("text-red-600 font-bold")
                with ui.row().classes("mt-4 justify-end"):
                    ui.button(
                        "Turn-off PLC",
                        on_click=self._abort_hmi
                    )
                    ui.button(
                        "Restart PLC",
                        color="red",
                        on_click=self._restart_plc
                    )
    
    def _abort_hmi(self) -> None:
        self.close()
        self.parent.abort_hmi()
    
    def _restart_plc(self) -> None:
        self.close()
        self.parent.restart_plc()
    
    def open(self) -> None:
        self._dialog.open()
    
    def close(self) -> None:
        self._dialog.close()
    

class ErrorDialog:
    
    def __init__(self, message: str):
        self.message = message
        self._create_dialog()

    def _create_dialog(self):
        self._dialog = ui.dialog().classes("w-[400px]")
        with self._dialog:
            with ui.card():
                ui.label(f"⚠️ Error").classes("text-red-600 font-bold")
                ui.label(self.message).classes("font-bold")
                with ui.row().classes("mt-4 justify-center"):
                    ui.button(
                        "Close",
                        color="red",
                        on_click=self.close
                    )
    
    def open(self):
        self._dialog.open()

    def close(self) -> None:
        self._dialog.close()
