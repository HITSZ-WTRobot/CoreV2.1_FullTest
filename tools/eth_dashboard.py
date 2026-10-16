"""Compact, read-only Rich view of validated FULLTEST firmware snapshots.

The network client owns transport validation and acknowledgements.  This module
only parses report text and renders immutable snapshots; no peripheral result is
inferred from host-side integrity checks or historical error counters.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import re
import time
from typing import ClassVar

from rich.console import Console, ConsoleOptions, Group, RenderResult
from rich.panel import Panel
from rich.table import Table
from rich.text import Text


_HEADER = re.compile(
    r"FULLTEST uptime_ms=([0-9]+) watchdog_hal=([0-9]+) "
    r"\(live snapshots; cumulative counters\)"
)
_RECORD = re.compile(
    r"([A-Z][A-Z0-9_]*) (START|WAIT|RUN|PASS|FAIL|UNKNOWN) "
    r"tx=([0-9]+) rx=([0-9]+) ok=([0-9]+) bad=([0-9]+) "
    r"err=([0-9]+) tmo=([0-9]+) detail=0x([0-9a-fA-F]{8})"
)
_STYLES = {
    "PASS": "green",
    "FAIL": "bold red",
    "WAIT": "yellow",
    "START": "yellow",
    "RUN": "cyan",
    "UNKNOWN": "yellow",
}
_COUNTER_NAMES = ("tx", "rx", "ok", "bad", "err", "tmo")


@dataclass(frozen=True, slots=True)
class Record:
    name: str
    state: str
    tx: int
    rx: int
    ok: int
    bad: int
    err: int
    tmo: int
    detail: int


@dataclass(frozen=True, slots=True)
class Snapshot:
    uptime_ms: int
    watchdog_hal: int
    records: tuple[Record, ...]
    truncated: bool
    received_at: float


def parse_report(report: str) -> Snapshot:
    """Reject malformed/duplicate records rather than manufacturing defaults."""
    lines = report.splitlines()
    if not lines:
        raise ValueError("empty FULLTEST report")
    header = _HEADER.fullmatch(lines[0])
    if header is None:
        raise ValueError("invalid FULLTEST header")
    truncated = lines[-1] == "REPORT TRUNCATED"
    body = lines[1:-1] if truncated else lines[1:]
    records = []
    names = set()
    for line_number, line in enumerate(body, start=2):
        match = _RECORD.fullmatch(line)
        if match is None:
            raise ValueError(f"invalid FULLTEST record on line {line_number}")
        name, state, *values = match.groups()
        if name in names:
            raise ValueError(f"duplicate FULLTEST record: {name}")
        names.add(name)
        records.append(
            Record(name, state, *(int(value) for value in values[:-1]), int(values[-1], 16))
        )
    if not records and not truncated:
        raise ValueError("FULLTEST report contains no records")
    return Snapshot(
        uptime_ms=int(header[1]),
        watchdog_hal=int(header[2]),
        records=tuple(records),
        truncated=truncated,
        received_at=time.monotonic(),
    )


@dataclass(frozen=True, slots=True)
class _ViewState:
    current: Snapshot | None = None
    previous: Snapshot | None = None
    activity: str = "Waiting for the first firmware report"
    integrity: bool | None = None
    integrity_message: str = "not run"
    error: str | None = None


def _uptime(milliseconds: int) -> str:
    seconds = milliseconds // 1000
    days, seconds = divmod(seconds, 86400)
    hours, seconds = divmod(seconds, 3600)
    minutes, seconds = divmod(seconds, 60)
    clock = f"{hours:02}:{minutes:02}:{seconds:02}"
    return f"{days}d {clock}" if days else clock


class Dashboard:
    """A Live-compatible view; a render sees exactly one immutable view state."""

    _GROUPS: ClassVar[tuple[tuple[str, tuple[str, ...]], ...]] = (
        ("ETH", ("ETH",)),
        ("CAN", ("CAN1", "CAN2", "CAN3")),
        ("UART 1/4", ("USART1", "UART4")),
        ("UART 3/9", ("USART3", "UART9")),
        ("UART 6/10", ("USART6", "USART10")),
        ("RS485", ("USART2_RS485",)),
        ("LED/PWM", ("LED_WALK", "TIM15_PWM")),
        ("KEY", ("KEY0", "KEY1", "KEY2", "KEY3")),
        ("I2C5", ("I2C5_SCAN",)),
    )

    def __init__(self, endpoint: str, interval: float, timeout: float):
        self.endpoint = endpoint
        self.interval = interval
        self.timeout = timeout
        self._state = _ViewState()

    def set_activity(self, message: str) -> None:
        self._state = replace(self._state, activity=message)

    def set_integrity(self, passed: bool | None, message: str) -> None:
        self._state = replace(
            self._state, integrity=passed, integrity_message=message
        )

    def update_report(self, report: str) -> None:
        snapshot = parse_report(report)
        state = self._state
        self._state = replace(
            state, previous=state.current, current=snapshot, error=None
        )

    def set_error(self, message: str) -> None:
        self._state = replace(self._state, error=message)

    def _freshness(self, state: _ViewState) -> tuple[str, str]:
        if state.current is None:
            return "NO DATA", "bold red" if state.error else "yellow"
        age = max(0.0, time.monotonic() - state.current.received_at)
        stale = state.error is not None or age > max(2 * self.interval, self.timeout)
        return f"{'STALE' if stale else 'LIVE'}  age {age:.1f}s", (
            "bold red" if stale else "green"
        )

    @staticmethod
    def _counts(snapshot: Snapshot) -> Text:
        counts = {name: 0 for name in _STYLES}
        for record in snapshot.records:
            counts[record.state] += 1
        text = Text()
        for name in ("FAIL", "WAIT", "RUN", "START", "UNKNOWN", "PASS"):
            if counts[name]:
                if text:
                    text.append("  ")
                text.append(f"{name} {counts[name]}", _STYLES[name])
        return text if text else Text("No complete records", style="bold yellow")

    @staticmethod
    def _counters(
        records: tuple[Record, ...], state: _ViewState, restarted: bool
    ) -> Text:
        if not records:
            return Text("—", style="dim")
        text = Text(f"{sum(record.ok for record in records)}", style="white")
        if state.previous is not None:
            previous = {record.name: record for record in state.previous.records}
            comparable = not restarted and all(record.name in previous for record in records)
            reset = comparable and any(
                getattr(record, field) < getattr(previous[record.name], field)
                for record in records
                for field in _COUNTER_NAMES
            )
            if restarted or reset:
                text.append(" Δ reset", style="yellow")
            elif comparable:
                delta = sum(record.ok - previous[record.name].ok for record in records)
                text.append(f" Δ+{delta}", style="cyan" if delta else "dim")
        bad, err, tmo = (
            sum(getattr(record, field) for record in records)
            for field in ("bad", "err", "tmo")
        )
        text.append(f" | {bad}/{err}/{tmo}", style="yellow" if bad or err or tmo else "dim")
        return text

    def _table(self, state: _ViewState, restarted: bool) -> Table:
        snapshot = state.current
        assert snapshot is not None
        by_name = {record.name: record for record in snapshot.records}
        table = Table(
            box=None, padding=(0, 1), pad_edge=False, expand=True,
            show_header=True, header_style="bold dim",
        )
        table.add_column("GROUP", no_wrap=True)
        table.add_column("FIRMWARE STATE / MEMBER", ratio=2)
        table.add_column("验证增量 | 坏/错/超时", ratio=1)
        known = {name for _, names in self._GROUPS for name in names}
        groups = (*self._GROUPS, *((name, (name,)) for name in by_name if name not in known))
        for label, names in groups:
            present = tuple(by_name[name] for name in names if name in by_name)
            member_text = Text()
            for name in names:
                if member_text:
                    member_text.append(" · ", style="dim")
                record = by_name.get(name)
                # Full member names keep failing UART directions unambiguous.
                display_name = name
                if name == "USART2_RS485":
                    display_name = "USART2"
                elif name == "LED_WALK":
                    display_name = "LED"
                elif name == "TIM15_PWM":
                    display_name = "PWM"
                elif name == "I2C5_SCAN":
                    display_name = "scan"
                member_text.append(f"{display_name} ")
                member_text.append(
                    record.state if record else "MISSING",
                    _STYLES[record.state] if record else "yellow",
                )
            table.add_row(
                Text(label, style="bold red" if any(r.state == "FAIL" for r in present) else "bold"),
                member_text,
                self._counters(present, state, restarted),
            )
        return table

    def __rich_console__(self, console: Console, options: ConsoleOptions) -> RenderResult:
        state = self._state
        snapshot = state.current
        freshness, freshness_style = self._freshness(state)
        heading = Text(self.endpoint, style="bold")
        heading.append("  •  ", style="dim")
        heading.append(freshness, style=freshness_style)
        rows: list = [heading]
        if state.error:
            stale_note = "旧数据已陈旧" if snapshot is not None else "尚无有效数据"
            if "RESULT BUSY" in state.error.upper():
                message = f"板端 BUSY：{stale_note}；可能需重启（不重传ACK）"
            elif "timeout" in state.error.lower() or "timed out" in state.error.lower():
                message = f"读取超时/断线：{stale_note}；持续 BUSY 可能需重启"
            else:
                message = f"报告异常：{stale_note}；{state.error}"
            error_notice = Text(message, style="bold red")
            error_notice.truncate(max(20, options.max_width - 4), overflow="ellipsis")
            rows.append(error_notice)
        if snapshot is not None:
            failed = [record for record in snapshot.records if record.state == "FAIL"]
            if failed:
                alarm = Text("FAIL: ", style="bold red")
                failures = []
                for record in failed:
                    diagnostics = [
                        f"{label}{value}"
                        for label, value in (
                            ("坏包", record.bad), ("错误", record.err), ("超时", record.tmo)
                        )
                        if value
                    ]
                    if record.detail:
                        diagnostics.append(f"detail=0x{record.detail:08x}")
                    failures.append(record.name + (" " + "/".join(diagnostics) if diagnostics else ""))
                alarm.append("，".join(failures))
                rows.append(alarm)
            if snapshot.watchdog_hal:
                rows.append(Text(f"WATCHDOG HAL ERROR: {snapshot.watchdog_hal}", style="bold red"))
            if snapshot.truncated:
                rows.append(Text("INCOMPLETE: REPORT TRUNCATED — not a complete board result", style="bold red"))
            restarted = state.previous is not None and snapshot.uptime_ms < state.previous.uptime_ms
            if restarted:
                rows.append(Text("BOARD RESTART / UPTIME RESET — counter deltas reset", style="yellow"))
            overview = Text(f"FW uptime {_uptime(snapshot.uptime_ms)}  |  ", style="dim")
            overview.append_text(self._counts(snapshot))
            rows.append(overview)
        else:
            restarted = False
            rows.append(Text("No firmware snapshot yet; no peripheral result is assumed.", style="yellow"))
        integrity_label = {None: "NOT RUN / PENDING", True: "PASS", False: "FAIL"}[state.integrity]
        integrity_style = {None: "yellow", True: "green", False: "bold red"}[state.integrity]
        host = Text("HOST integrity: ", style="dim")
        host.append(integrity_label, integrity_style)
        host.append(f" — {state.integrity_message}", style="dim")
        rows.append(host)
        rows.append(Text(state.activity, style="dim"))
        if snapshot is not None:
            rows.append(self._table(state, restarted))
        rows.append(Text("KEY: 每键按下/释放2次  |  I2C: 未接器件可为WAIT", style="yellow"))
        rows.append(Text("LED/PWM: 人工观察，RUN≠PASS；计数均累计，Δ为本次变化。", style="dim"))
        yield Panel(
            Group(*rows), title="[bold]FULLTEST · BOARD TELEMETRY[/bold]",
            border_style="red" if state.error or (snapshot is not None and (
                snapshot.watchdog_hal or snapshot.truncated or any(
                    record.state == "FAIL" for record in snapshot.records
                )
            )) or state.integrity is False else "cyan",
            padding=(0, 1), expand=True,
        )

    def summary(self) -> str:
        state = self._state
        freshness, _ = self._freshness(state)
        if state.current is None:
            summary = f"{self.endpoint}: no valid firmware report"
        else:
            snapshot = state.current
            counts = self._counts(snapshot).plain
            summary = f"{self.endpoint}: {freshness}; firmware uptime {_uptime(snapshot.uptime_ms)}; {counts}"
            if snapshot.truncated:
                summary += "; INCOMPLETE REPORT"
            if snapshot.watchdog_hal:
                summary += f"; watchdog_hal={snapshot.watchdog_hal}"
        if state.integrity is False:
            summary += "; host integrity FAILED"
        if state.error:
            summary += f"; {state.error}"
        return summary
