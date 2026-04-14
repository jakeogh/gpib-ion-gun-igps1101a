#!/usr/bin/env python3
from __future__ import annotations

import binascii
import json
import os
import re
import threading
import time
from collections import deque
from dataclasses import asdict
from dataclasses import dataclass
from typing import Any
from typing import Iterable
from typing import List
from typing import Optional
from typing import Tuple

import serial
from rich import print as pprint  # noqa: F401

# ========================== Known Scaling ===============================
KNOWN_GI_SCALE: dict[int, tuple[str, float]] = {
    0: ("I+ ENERGY", 10.0),
    1: ("SOURCE", 1000.0),
    2: ("FIELD CONTROL", 10.0),
    3: ("EXTRACT", 1.0),
    4: ("FOCUS", 1.0),
    5: ("E- ENERGY", 10.0),
    8: ("X DEFL", 100.0),
    9: ("Y DEFL", 100.0),
}
KNOWN_ORDER = sorted(KNOWN_GI_SCALE.keys())


# ============================ Data Models ===============================
@dataclass
class IdInfo:
    firmware: Optional[str]
    model_name: Optional[str]
    model_rev: Optional[str]
    serial_number: Optional[str]


@dataclass
class StatusInfo:
    raw: Optional[str]
    code: Optional[int]


@dataclass
class ChannelHint:
    index: int
    direction: str
    name: Optional[str]
    units: Optional[str]
    scale_min: Optional[float]
    scale_max: Optional[float]
    counts_min: Optional[int]
    counts_max: Optional[int]


@dataclass
class Reading:
    cmd: str
    index: int
    raw_text: str
    raw_value: Optional[float]
    scaled_value: Optional[float]
    name: Optional[str]
    units: Optional[str]


# ============================== Debug Utilities =============================
def unix_ts() -> str:
    return f"{time.time_ns()/1e9:.9f}"


def _hexdump(b: bytes, limit: int = 256) -> str:
    if not b:
        return ""
    if len(b) > limit:
        return binascii.hexlify(b[:limit]).decode() + f"...(+{len(b)-limit}B)"
    return binascii.hexlify(b).decode()


class DebugLog:
    def __init__(self, enabled: bool) -> None:
        self.enabled = enabled

    def log(self, *a: object) -> None:
        if self.enabled:
            t = time.strftime("%Y-%m-%d %H:%M:%S") + f".{int((time.time()%1)*1000):03d}"
            print("[DEBUG]", t, *a, flush=True)


# ================================ Serial I/O ================================
class HostAscii:
    """Minimal HostASCII client (CRLF, XON/XOFF). Read-only commands only, robust RX."""

    def __init__(
        self,
        port: str,
        baud: int,
        timeout: float,
        xonxoff: bool,
        dbg: DebugLog,
        rx_chunk: int = 4096,
        idle_quiet: float = 0.05,
        max_rx_time: float = 0.30,
    ) -> None:
        self.dbg = dbg
        self.rx_chunk = rx_chunk
        self.idle_quiet = idle_quiet
        self.max_rx_time = max_rx_time
        self.ser = serial.Serial(
            port,
            baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            xonxoff=xonxoff,
            rtscts=False,
            timeout=0.02,
            write_timeout=float(timeout),
        )
        self.ser.dtr = True
        self.ser.rts = True
        self.dbg.log(
            f"Opened serial port {port} @ {baud} 8N1 xonxoff={xonxoff}, timeout={timeout}"
        )
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self) -> None:
        try:
            self.dbg.log("Closing serial port")
            self.ser.close()
        except Exception as e:
            self.dbg.log("Error during close:", e)

    def _read_response(self, terminator: bytes = b"\r\n") -> bytes:
        start = time.time()
        last_data = start
        buf = bytearray()
        while True:
            chunk = self.ser.read(self.rx_chunk)
            now = time.time()
            if chunk:
                buf.extend(chunk)
                last_data = now
                if terminator in buf:
                    drain_start = time.time()
                    while (time.time() - drain_start) < self.idle_quiet:
                        tail = self.ser.read(self.rx_chunk)
                        if tail:
                            buf.extend(tail)
                            drain_start = time.time()
                    return bytes(buf)
                continue
            if (now - start) >= self.max_rx_time:
                return bytes(buf)
            if buf and (now - last_data) >= self.idle_quiet:
                return bytes(buf)

    def txrx(self, cmd: str, sleep_before_read: float = 0.0) -> str:
        data = cmd.encode("ascii") + b"\r\n"
        self.dbg.log(f"TX {cmd!r} [{len(data)}B] hex={_hexdump(data)}")
        self.ser.reset_input_buffer()
        self.ser.write(data)
        self.ser.flush()
        if sleep_before_read > 0:
            time.sleep(sleep_before_read)
        raw = self._read_response()
        self.dbg.log(f"RX [{len(raw)}B] hex={_hexdump(raw)}")
        try:
            return raw.decode("ascii", errors="replace")
        except Exception:
            return repr(raw)


# ============================ Parsing / Utilities ===========================
_NUM_RE = re.compile(r"([-+]?\d+(?:\.\d+)?)")
_KV_RE = re.compile(r"^([a-z]+):\s*(.+)$", re.I)
_GI_PAIR_RE = re.compile(r"^\s*(?:gi|go)\s*:\s*\d+\s*,\s*([-+]?\d+(?:\.\d+)?)", re.I)


def _first_kv(line: str) -> tuple[Optional[str], Optional[str]]:
    m = _KV_RE.search(line.strip())
    return (m.group(1).lower(), m.group(2)) if m else (None, None)


def _parse_reply_value(s: str) -> Optional[float]:
    m = _GI_PAIR_RE.search(s)
    if m:
        try:
            return float(m.group(1))
        except Exception:
            pass
    nums = _NUM_RE.findall(s)
    return float(nums[-1]) if nums else None


def _parse_status_code(s: str) -> Optional[int]:
    m = re.search(r"gs\s*:\s*([0-9A-Fa-fx]+)", s)
    if not m:
        return None
    txt = m.group(1).strip()
    for base in (0, 10):
        try:
            return int(txt, base or 10)
        except Exception:
            continue
    return None


def parse_id(gfw: str, gmn: str, gmr: str, gsn: str) -> IdInfo:
    fw = mn = mr = sn = None
    for line in gfw.splitlines():
        k, v = _first_kv(line)
        if k == "gfw":
            fw = v.strip()
            break
    for line in gmn.splitlines():
        k, v = _first_kv(line)
        if k == "gmn":
            mn = v.strip()
            break
    for line in gmr.splitlines():
        k, v = _first_kv(line)
        if k == "gmr":
            mr = v.strip()
            break
    for line in gsn.splitlines():
        k, v = _first_kv(line)
        if k == "gsn":
            sn = v.strip()
            break
    return IdInfo(firmware=fw, model_name=mn, model_rev=mr, serial_number=sn)


def parse_status(gs: str) -> StatusInfo:
    return StatusInfo(raw=gs.strip(), code=_parse_status_code(gs))


_CHAN_LINE_RE = re.compile(
    r"(?:(?:^|\s)chan(?:nel)?\s*[:#]?\s*(\d+)|^\s*(?:out|output|in|input)\s*[:#]?\s*(\d+)|^\s*(\d+)\s*[:\-])",
    re.I,
)
_DIR_HINT_RE = re.compile(r"\b(output|out|input|in)\b", re.I)
_NAME_RE = re.compile(r"name\s*[:=]\s*([A-Za-z0-9 _\-\./%]+)", re.I)
_UNITS_RE = re.compile(r"units?\s*[:=]\s*([A-Za-z0-9 _\-\./%]+)", re.I)
_SCALE_RE = re.compile(
    r"(?P<cmin>[-+]?\d+(?:\.\d+)?)\s*\.\.\s*(?P<cmax>[-+]?\d+(?:\.\d+)?)\s*(?:counts?|cnts?)\s*[-=]>\s*(?P<smin>[-+]?\d+(?:\.\d+)?)\s*\.\.\s*(?P<smax>[-+]?\d+(?:\.\d+)?)\s*(?P<units>[A-Za-z%/]+)?",
    re.I,
)


def parse_gmc_hints(gmc_text: str) -> list[ChannelHint]:
    hints: list[ChannelHint] = []
    for raw in gmc_text.splitlines():
        line = raw.strip()
        if not line:
            continue
        m = _CHAN_LINE_RE.search(line)
        if not m:
            continue
        idx_txt = next((g for g in m.groups() if g), None)
        if not idx_txt:
            continue
        try:
            idx = int(idx_txt)
        except Exception:
            continue
        dir_hint = None
        mdir = _DIR_HINT_RE.search(line)
        if mdir:
            d = mdir.group(1).lower()
            dir_hint = "out" if d.startswith("out") else "in"
        name = None
        mname = _NAME_RE.search(line)
        if mname:
            name = mname.group(1).strip()
        units = None
        scale_min = scale_max = None
        counts_min = counts_max = None
        mscale = _SCALE_RE.search(line)
        if mscale:
            try:
                counts_min = int(float(mscale.group("cmin")))
                counts_max = int(float(mscale.group("cmax")))
                scale_min = float(mscale.group("smin"))
                scale_max = float(mscale.group("smax"))
                u = mscale.group("units")
                units = u.strip() if u else units
            except Exception:
                pass
        if units is None:
            munits = _UNITS_RE.search(line)
            if munits:
                units = munits.group(1).strip()
        hints.append(
            ChannelHint(
                index=idx,
                direction=dir_hint or "unknown",
                name=name,
                units=units,
                scale_min=scale_min,
                scale_max=scale_max,
                counts_min=counts_min,
                counts_max=counts_max,
            )
        )
    seen: set[int] = set()
    uniq: list[ChannelHint] = []
    for h in hints:
        if h.index in seen:
            continue
        seen.add(h.index)
        uniq.append(h)
    return sorted(uniq, key=lambda h: h.index)


# ================================ Core Logic ================================
def verify_expected_device(
    cli: HostAscii,
    expected_model_substr: str,
    dbg: DebugLog,
) -> IdInfo:
    resp_gmn = cli.txrx("gmn")
    if not resp_gmn.strip():
        raise RuntimeError("No response to 'gmn'. Check wiring, baud, flow, CRLF.")
    resp_gfw = cli.txrx("gfw")
    resp_gmr = cli.txrx("gmr")
    resp_gsn = cli.txrx("gsn")
    idinfo = parse_id(resp_gfw, resp_gmn, resp_gmr, resp_gsn)
    dbg.log("Identity parsed:", idinfo)
    model = (idinfo.model_name or "").strip()
    if not model:
        raise RuntimeError("Device did not return a model name (gmn).")
    if expected_model_substr and expected_model_substr.lower() not in model.lower():
        raise RuntimeError(
            f"Model mismatch. Expected substring '{expected_model_substr}', got '{model}'."
        )
    return idinfo


def gi_read(cli: HostAscii, idx: int) -> tuple[Optional[float], str]:
    txt = cli.txrx(f"gi:{idx}")
    if not txt.strip():
        return (None, txt)
    if re.search(r"(invalid|error|unknown|egi:c)", txt, re.I):
        return (None, txt.strip())
    return (_parse_reply_value(txt), txt.strip())


def read_status(cli: HostAscii) -> StatusInfo:
    """Send 'gs' and return parsed StatusInfo."""
    return parse_status(cli.txrx("gs"))


def build_indices(
    probe_min: int,
    probe_max: int,
    only_known: bool,
) -> tuple[List[int], List[int]]:
    if only_known:
        known = [i for i in KNOWN_ORDER if probe_min <= i <= probe_max]
        unknown: List[int] = []
    else:
        known = [i for i in KNOWN_ORDER if probe_min <= i <= probe_max]
        unknown = [
            i for i in range(probe_min, probe_max + 1) if i not in KNOWN_GI_SCALE
        ]
    return known, unknown


def probe_responsive_unknown(
    client: HostAscii,
    unknown_indices: List[int],
    sleep_between: float = 0.0,
) -> List[int]:
    """One-shot probe: keep only unknown channels that return a parsable value."""
    survivors: list[int] = []
    for i in unknown_indices:
        val, _txt = gi_read(client, i)
        if val is not None:
            survivors.append(i)
        if sleep_between > 0:
            time.sleep(sleep_between)
    return survivors


def build_csv_header(known_indices: List[int], unknown_indices: List[int]) -> List[str]:
    header_cols: list[str] = ["timestamp"]
    for i in known_indices:
        header_cols.append(KNOWN_GI_SCALE[i][0])
    for i in unknown_indices:
        header_cols.append(f"gi:{i}")
    return header_cols


def build_csv_row(
    client: HostAscii,
    known_indices: List[int],
    unknown_indices: List[int],
    sleep_between: float,
) -> List[str]:
    values: list[str] = [unix_ts()]
    for i in known_indices:
        val, txt = gi_read(client, i)
        if val is None:
            values.append("")
        else:
            divisor = KNOWN_GI_SCALE[i][1]
            values.append(f"{(val/divisor):.6f}")
        if sleep_between > 0:
            time.sleep(sleep_between)
    for i in unknown_indices:
        val, txt = gi_read(client, i)
        if val is None:
            token = (txt or "").strip().replace(",", ";")
            values.append("" if token.lower() == "egi:c" else token)
        else:
            values.append(f"{val:.6f}")
        if sleep_between > 0:
            time.sleep(sleep_between)
    return values


def sample_dict(
    client: HostAscii,
    known_indices: List[int],
    unknown_indices: List[int],
    sleep_between: float = 0.0,
    include_status: bool = False,
) -> dict[str, Any]:
    """Return one sample as a dict: {column_name: float|None, 'timestamp': float}.
    If include_status, also adds 'status_code' (int|None) and 'status_raw' (str)."""
    row: dict[str, Any] = {"timestamp": time.time_ns() / 1e9}
    if include_status:
        st = read_status(client)
        row["status_code"] = st.code
        row["status_raw"] = st.raw
        if sleep_between > 0:
            time.sleep(sleep_between)
    for i in known_indices:
        val, _ = gi_read(client, i)
        name, divisor = KNOWN_GI_SCALE[i]
        row[name] = (val / divisor) if val is not None else None
        if sleep_between > 0:
            time.sleep(sleep_between)
    for i in unknown_indices:
        val, _ = gi_read(client, i)
        row[f"gi:{i}"] = val if val is not None else None
        if sleep_between > 0:
            time.sleep(sleep_between)
    return row


def format_table(
    idinfo: IdInfo,
    status: StatusInfo,
    readings: list[Reading],
) -> str:
    headers = ["CMD", "IDX", "NAME", "RAW", "SCALED", "UNITS"]
    rows: list[list[str]] = []
    for r in sorted(readings, key=lambda x: (x.cmd, x.index)):
        rows.append(
            [
                r.cmd,
                str(r.index),
                (r.name or ""),
                (
                    f"{r.raw_value:.6g}"
                    if r.raw_value is not None
                    else r.raw_text.replace("\r", "\\r").replace("\n", "\\n")
                ),
                (f"{r.scaled_value:.6g}" if r.scaled_value is not None else ""),
                (r.units or ""),
            ]
        )

    def fmt_row(cols: Iterable[str], widths: list[int]) -> str:
        return "  ".join(
            (c if len(c) <= w else c[: w - 1] + "…").ljust(w)
            for c, w in zip(cols, widths)
        )

    widths = (
        [max(len(h), *(len(row[i]) for row in rows)) for i, h in enumerate(headers)]
        if rows
        else [3, 3, 4, 3, 6, 5]
    )
    out = []
    out.append("\n=== Identity ===")
    out.append(f"Firmware : {idinfo.firmware or ''}")
    out.append(f"Model    : {idinfo.model_name or ''}  rev: {idinfo.model_rev or ''}")
    out.append(f"Serial   : {idinfo.serial_number or ''}")
    out.append("\n=== Status ===")
    out.append(f"{status.raw or ''}")
    if status.code is not None:
        out.append(f"Status code (int): {status.code}")
    out.append("\n=== Readings (read-only) ===")
    out.append(fmt_row(headers, widths))
    out.append(fmt_row(["-" * w for w in widths], widths))
    for row in rows:
        out.append(fmt_row(row, widths))
    out.append("")
    return "\n".join(out)


# ============================== Background Logger ===========================
class BackgroundLogger:
    """
    Polls the device in a daemon thread and keeps the last `buffer_size`
    samples in a ring buffer. Embed in another app:

        from gpib_ion_gun_igps1101a import start_logger
        h = start_logger("/dev/ttyUSB0", interval=1.0)
        ...
        rows = h.recent(10)        # list[dict], oldest -> newest
        last = h.latest()          # single dict or None
        cols = h.header()          # column names in the order present in row dicts
        h.stop()

    Or as a context manager:

        with start_logger("/dev/ttyUSB0") as h:
            ...
    """

    def __init__(
        self,
        port: str,
        baud: int = 19200,
        timeout: float = 1.0,
        xonxoff: bool = True,
        expect_model: str = "IGPS-1101A",
        idle_quiet: float = 0.05,
        max_rx_time: float = 0.30,
        debug: bool = False,
        probe_min: int = 0,
        probe_max: int = 31,
        only_known: bool = True,
        probe_skip_silent: bool = True,
        sleep_between: float = 0.0,
        interval: float = 1.0,
        buffer_size: int = 1000,
        include_status: bool = True,
    ) -> None:
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.xonxoff = xonxoff
        self.expect_model = expect_model
        self.idle_quiet = idle_quiet
        self.max_rx_time = max_rx_time
        self.debug = debug
        self.probe_min = probe_min
        self.probe_max = probe_max
        self.only_known = only_known
        self.probe_skip_silent = probe_skip_silent
        self.sleep_between = sleep_between
        self.interval = interval
        self.include_status = include_status

        self._buffer: deque[dict[str, Any]] = deque(maxlen=buffer_size)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._client: HostAscii | None = None
        self._dbg: DebugLog | None = None
        self._idinfo: IdInfo | None = None
        self._known: list[int] = []
        self._unknown: list[int] = []
        self._header: list[str] = []
        self._exception: BaseException | None = None
        self._last_status: StatusInfo | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            raise RuntimeError("BackgroundLogger already running")
        self._dbg = DebugLog(enabled=self.debug)
        self._client = HostAscii(
            port=self.port,
            baud=self.baud,
            timeout=self.timeout,
            xonxoff=self.xonxoff,
            dbg=self._dbg,
            idle_quiet=self.idle_quiet,
            max_rx_time=self.max_rx_time,
        )
        try:
            self._idinfo = verify_expected_device(
                self._client, self.expect_model, self._dbg
            )
            self._known, self._unknown = build_indices(
                self.probe_min, self.probe_max, self.only_known
            )
            if self.probe_skip_silent and self._unknown:
                survivors = probe_responsive_unknown(
                    self._client, self._unknown, self.sleep_between
                )
                self._dbg.log(
                    f"probe-skip: {len(self._unknown)} candidates -> "
                    f"{len(survivors)} responsive: {survivors}"
                )
                self._unknown = survivors
            self._header = build_csv_header(self._known, self._unknown)
            if self.include_status:
                self._header = (
                    [self._header[0], "status_code", "status_raw"] + self._header[1:]
                )
        except Exception:
            self._client.close()
            self._client = None
            raise

        self._stop_event.clear()
        self._exception = None
        self._thread = threading.Thread(
            target=self._run, name=f"igps-logger-{self.port}", daemon=True
        )
        self._thread.start()

    def _run(self) -> None:
        next_t = time.monotonic()
        try:
            while not self._stop_event.is_set():
                assert self._client is not None
                row = sample_dict(
                    self._client,
                    self._known,
                    self._unknown,
                    self.sleep_between,
                    include_status=self.include_status,
                )
                with self._lock:
                    self._buffer.append(row)
                    if self.include_status:
                        self._last_status = StatusInfo(
                            raw=row.get("status_raw"),
                            code=row.get("status_code"),
                        )
                next_t += self.interval
                wait = next_t - time.monotonic()
                if wait > 0:
                    if self._stop_event.wait(timeout=wait):
                        break
                else:
                    next_t = time.monotonic()
        except BaseException as e:
            self._exception = e

    def stop(self, timeout: float = 2.0) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=timeout)
            self._thread = None
        if self._client:
            try:
                self._client.close()
            finally:
                self._client = None

    def recent(self, n: int = 1) -> list[dict[str, Any]]:
        """Return the last `n` samples, oldest -> newest. Returns at most n."""
        if n <= 0:
            return []
        with self._lock:
            if n >= len(self._buffer):
                return list(self._buffer)
            return list(self._buffer)[-n:]

    def latest(self) -> dict[str, Any] | None:
        with self._lock:
            return self._buffer[-1] if self._buffer else None

    def header(self) -> list[str]:
        return list(self._header)

    def identity(self) -> IdInfo | None:
        return self._idinfo

    def status(self) -> StatusInfo | None:
        """Most recent StatusInfo from the polling loop (None until first sample)."""
        with self._lock:
            return self._last_status

    def buffer_len(self) -> int:
        with self._lock:
            return len(self._buffer)

    def clear(self) -> None:
        with self._lock:
            self._buffer.clear()

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def exception(self) -> BaseException | None:
        return self._exception

    def __enter__(self) -> "BackgroundLogger":
        self.start()
        return self

    def __exit__(self, *exc_info: Any) -> None:
        self.stop()


def start_logger(port: str, **kwargs: Any) -> BackgroundLogger:
    """Construct, start, and return a BackgroundLogger in one call."""
    bl = BackgroundLogger(port, **kwargs)
    bl.start()
    return bl


__all__ = [
    "KNOWN_GI_SCALE",
    "KNOWN_ORDER",
    "IdInfo",
    "StatusInfo",
    "ChannelHint",
    "Reading",
    "DebugLog",
    "HostAscii",
    "parse_gmc_hints",
    "parse_status",
    "parse_id",
    "verify_expected_device",
    "build_indices",
    "probe_responsive_unknown",
    "build_csv_header",
    "build_csv_row",
    "sample_dict",
    "gi_read",
    "read_status",
    "format_table",
    "unix_ts",
    "BackgroundLogger",
    "start_logger",
]
