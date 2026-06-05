from __future__ import annotations

from collections.abc import Mapping
from collections import deque
from html import escape
from typing import Any, Callable
import logging

from pyberryplc.core import MemoryVariable, SharedMemoryBlock, SoftMachineState


class SoftMachineLogHandler(logging.Handler):
    """In-memory log handler used by the soft-machine log panel."""

    def __init__(self, max_lines: int = 200) -> None:
        super().__init__()
        self.records: deque[str] = deque(maxlen=max_lines)
        self.setFormatter(logging.Formatter(
            "[%(name)s %(asctime)s | %(levelname)s] %(message)s",
            datefmt="%H:%M:%S",
        ))

    def emit(self, record: logging.LogRecord) -> None:
        self.records.append(self.format(record))

    def read(self) -> list[str]:
        return list(self.records)


class SoftMachine:
    """
    Generic NiceGUI-based soft-machine for PLC testing.

    The soft-machine can expose one or more `SoftMachineState` objects and
    optional `SharedMemoryBlock` objects. This keeps soft I/O and PLC-to-PLC
    communication visible in one browser UI without coupling both concepts.
    """

    def __init__(
        self,
        states: SoftMachineState | Mapping[str, SoftMachineState],
        datablocks: Mapping[str, SharedMemoryBlock] | None = None,
        title: str = "PyBerryPLC Soft Machine",
        port: int = 8082,
        refresh_interval: float = 0.25,
        on_exit: Callable[[], None] | None = None,
        logger: logging.Logger | None = None,
        log_lines: int = 200,
        close_browser_on_exit: bool = True,
    ) -> None:
        self.states = self._normalize_states(states)
        self.datablocks = dict(datablocks) if datablocks else {}
        self.title = title
        self.port = port
        self.refresh_interval = refresh_interval
        self.on_exit = on_exit
        self.logger = logger
        self.log_handler = self._setup_log_handler(logger, log_lines)
        self.close_browser_on_exit = close_browser_on_exit

        self._input_switches: list[tuple[Any, SoftMachineState, str]] = []
        self._digital_output_labels: list[tuple[Any, SoftMachineState, str]] = []
        self._pwm_output_labels: list[tuple[Any, SoftMachineState, str]] = []
        self._memory_switches: list[tuple[Any, MemoryVariable]] = []
        self._memory_labels: list[tuple[Any, MemoryVariable]] = []
        self._log_html = None
        self._last_log_text = ""

    @staticmethod
    def _normalize_states(
        states: SoftMachineState | Mapping[str, SoftMachineState]
    ) -> dict[str, SoftMachineState]:
        if isinstance(states, SoftMachineState):
            return {"Machine": states}
        return dict(states)

    @staticmethod
    def _setup_log_handler(
        logger: logging.Logger | None,
        log_lines: int,
    ) -> SoftMachineLogHandler | None:
        if logger is None:
            return None
        handler = SoftMachineLogHandler(log_lines)
        logger.addHandler(handler)
        return handler

    def run(self) -> None:
        """Launch the soft-machine UI."""
        from nicegui import ui

        @ui.page("/")
        def soft_machine_page() -> None:
            self._build_page(ui)

        ui.run(
            title=self.title,
            port=self.port,
            reload=False,
            show=False,
        )

    def _build_page(self, ui: Any) -> None:
        self._clear_element_refs()

        with ui.header().classes("bg-gray-200 p-2"):
            with ui.row().classes("w-full justify-between items-center"):
                ui.label(self.title).classes("text-lg font-bold")
                ui.button("Exit", color="red", on_click=self.exit)

        with ui.column().classes("w-full p-4 gap-4"):
            tabs = ui.tabs().classes("w-full")
            with tabs:
                ui.tab("I/O")
                if self.datablocks:
                    ui.tab("Datablocks")
                if self.log_handler:
                    ui.tab("Log")

            with ui.tab_panels(tabs, value="I/O").classes("w-full"):
                with ui.tab_panel("I/O"):
                    self._build_io_panel(ui)

                if self.datablocks:
                    with ui.tab_panel("Datablocks"):
                        self._build_datablocks_panel(ui)

                if self.log_handler:
                    with ui.tab_panel("Log"):
                        self._build_log_panel(ui)

        ui.timer(self.refresh_interval, self.refresh)
        self.refresh()

    def exit(self) -> None:
        """Request a clean PLC shutdown and stop the NiceGUI application."""
        from nicegui import app, ui

        ui.notify("Shutting down soft-machine...", color="grey")
        if self.on_exit is not None:
            self.on_exit()
        if self.close_browser_on_exit:
            ui.run_javascript("setTimeout(() => window.close(), 100);")
        ui.timer(0.5, app.shutdown, once=True)

    def _clear_element_refs(self) -> None:
        self._input_switches.clear()
        self._digital_output_labels.clear()
        self._pwm_output_labels.clear()
        self._memory_switches.clear()
        self._memory_labels.clear()
        self._log_html = None
        self._last_log_text = ""

    def _build_io_panel(self, ui: Any) -> None:
        for name, state in self.states.items():
            with ui.expansion(value=True).classes("w-full") as expansion:
                with expansion.add_slot("header"):
                    ui.label(name).classes("text-lg font-bold w-[calc(100%-40px)] inline-block")

                snapshot = self._snapshot_state(state)
                has_channels = any(
                    (
                        snapshot["digital_inputs"],
                        snapshot["digital_outputs"],
                        snapshot["pwm_outputs"],
                    )
                )
                if not has_channels:
                    ui.label("No soft I/O channels registered.").classes(
                        "text-gray-500"
                    )
                    continue

                with ui.column().classes("w-full gap-4"):
                    if snapshot["digital_inputs"]:
                        self._build_digital_input_panel(
                            ui,
                            state,
                            snapshot["digital_inputs"],
                            snapshot["digital_input_labels"],
                        )
                    if snapshot["digital_outputs"]:
                        self._build_digital_output_panel(
                            ui,
                            state,
                            snapshot["digital_outputs"],
                            snapshot["digital_output_labels"],
                        )
                    if snapshot["pwm_outputs"]:
                        self._build_pwm_output_panel(
                            ui,
                            state,
                            snapshot["pwm_outputs"],
                            snapshot["pwm_output_labels"],
                        )

    def _build_digital_input_panel(
        self,
        ui: Any,
        state: SoftMachineState,
        values: dict[str, bool],
        labels: dict[str, str],
    ) -> None:
        with ui.column().classes("min-w-[260px] gap-2"):
            ui.label("Digital Inputs").classes("text-md font-bold")
            if not values:
                ui.label("None").classes("text-gray-500")
                return

            for pin, value in self._sorted_items(values):
                switch = ui.switch(
                    self._channel_title(pin, labels),
                    value=value,
                    on_change=lambda event, s=state, p=pin:
                        s.set_digital_input(p, bool(event.value)),
                )
                self._input_switches.append((switch, state, pin))

    def _build_digital_output_panel(
        self,
        ui: Any,
        state: SoftMachineState,
        values: dict[str, bool],
        labels: dict[str, str],
    ) -> None:
        with ui.column().classes("min-w-[260px] gap-2"):
            ui.label("Digital Outputs").classes("text-md font-bold")
            if not values:
                ui.label("None").classes("text-gray-500")
                return

            for pin, value in self._sorted_items(values):
                with ui.row().classes("items-center gap-2"):
                    ui.label(self._channel_title(pin, labels)).classes(
                        "min-w-[160px]"
                    )
                    value_label = ui.label(self._bool_text(value)).classes(
                        "font-mono"
                    )
                    self._digital_output_labels.append((value_label, state, pin))

    def _build_pwm_output_panel(
        self,
        ui: Any,
        state: SoftMachineState,
        values: dict[str, float],
        labels: dict[str, str],
    ) -> None:
        with ui.column().classes("min-w-[260px] gap-2"):
            ui.label("PWM Outputs").classes("text-md font-bold")
            if not values:
                ui.label("None").classes("text-gray-500")
                return

            for pin, value in self._sorted_items(values):
                with ui.row().classes("items-center gap-2"):
                    ui.label(self._channel_title(pin, labels)).classes(
                        "min-w-[160px]"
                    )
                    value_label = ui.label(self._number_text(value)).classes(
                        "font-mono"
                    )
                    self._pwm_output_labels.append((value_label, state, pin))

    def _build_datablocks_panel(self, ui: Any) -> None:
        for name, datablock in self.datablocks.items():
            with ui.expansion(name, value=True).classes("w-full"):
                if not datablock.data:
                    ui.label("No memory variables.").classes("text-gray-500")
                    continue

                with ui.column().classes("gap-2"):
                    for var_name, mem_var in self._sorted_items(datablock.data):
                        self._build_memory_variable_row(ui, var_name, mem_var)

    def _build_memory_variable_row(
        self,
        ui: Any,
        name: str,
        mem_var: MemoryVariable,
    ) -> None:
        with ui.row().classes("items-center gap-3"):
            ui.label(name).classes("min-w-[220px]")
            if mem_var.single_bit:
                switch = ui.switch(
                    value=bool(mem_var.curr_state),
                    on_change=lambda event, v=mem_var:
                        v.update(bool(event.value)),
                )
                self._memory_switches.append((switch, mem_var))
            else:
                value_label = ui.label(self._format_value(mem_var.state)).classes(
                    "font-mono"
                )
                self._memory_labels.append((value_label, mem_var))

    def _build_log_panel(self, ui: Any) -> None:
        self._log_html = ui.html(
            '<pre style="font-family: monospace; font-size: 13px; '
            'margin: 0; line-height: 1.2;"></pre>'
        ).classes("w-full h-[360px] overflow-auto bg-white border rounded p-2")

    def refresh(self) -> None:
        """Refresh UI elements from the current soft-machine state."""
        for switch, state, pin in self._input_switches:
            value = state.read_digital_input(pin)
            if switch.value != value:
                switch.value = value
                switch.update()

        for label, state, pin in self._digital_output_labels:
            label.set_text(self._bool_text(state.read_digital_output(pin)))

        for label, state, pin in self._pwm_output_labels:
            label.set_text(self._number_text(state.read_pwm_output(pin)))

        for switch, mem_var in self._memory_switches:
            value = bool(mem_var.curr_state)
            if switch.value != value:
                switch.value = value
                switch.update()

        for label, mem_var in self._memory_labels:
            label.set_text(self._format_value(mem_var.state))

        self._refresh_log()

    def _refresh_log(self) -> None:
        if self.log_handler is None or self._log_html is None:
            return

        lines = self.log_handler.read()
        log_text = "\n".join(lines)
        if log_text == self._last_log_text:
            return

        self._last_log_text = log_text
        escaped_log = escape(log_text)
        self._log_html.set_content(
            '<pre style="font-family: monospace; font-size: 13px; '
            f'margin: 0; line-height: 1.2;">{escaped_log}</pre>'
        )

    @staticmethod
    def _snapshot_state(state: SoftMachineState) -> dict[str, dict]:
        with state.lock:
            return {
                "digital_inputs": dict(state.digital_inputs),
                "digital_outputs": dict(state.digital_outputs),
                "pwm_outputs": dict(state.pwm_outputs),
                "digital_input_labels": dict(state.digital_input_labels),
                "digital_output_labels": dict(state.digital_output_labels),
                "pwm_output_labels": dict(state.pwm_output_labels),
            }

    @staticmethod
    def _sorted_items(values: Mapping[str, Any]) -> list[tuple[str, Any]]:
        return sorted(values.items(), key=lambda item: item[0])

    @staticmethod
    def _channel_title(pin: str, labels: Mapping[str, str]) -> str:
        label = labels.get(pin, "")
        if label:
            return f"{pin} | {label}"
        return pin

    @staticmethod
    def _bool_text(value: bool) -> str:
        return "ON" if value else "OFF"

    @staticmethod
    def _number_text(value: float) -> str:
        return f"{value:.3f}"

    @staticmethod
    def _format_value(value: Any) -> str:
        if isinstance(value, float):
            return f"{value:.3f}"
        return str(value)
