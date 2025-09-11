#!/usr/bin/env python3
from __future__ import annotations

import binascii
import json
import os
import re
import time
from dataclasses import asdict
from dataclasses import dataclass
from typing import Any
from typing import Iterable
from typing import List
from typing import Optional
from typing import Tuple

import serial
from rich import print as pprint  # noqa: F401

# =========================== Known Scaling ===============================
# Hard-coded counts -> volts divisors from your front-panel mapping.
KNOWN_GI_SCALE: dict[int, tuple[str, float]] = {
    0: ("I+ ENERGY", 10.0),  # 5598 -> 559.8 V
    1: ("SOURCE", 1000.0),  # 1699 -> 1.699 V
    2: ("FIELD CONTROL", 10.0),  # 1698 -> 169.8 V
    3: ("EXTRACT", 1.0),  # 1050 -> 1050 V
    4: ("FOCUS", 1.0),  # 569  -> 569 V
    5: ("E- ENERGY", 10.0),  # 899  -> 89.9 V
    8: ("X DEFL", 100.0),  # 98   -> 0.98 V
    9: ("Y DEFL", 100.0),  # 1701 -> 17.01 V
}
KNOWN_ORDER = sorted(KNOWN_GI_SCALE.keys())  # [0,1,2,3,4,5,8,9]


# ============================= Data Models ===============================
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
    """Unix seconds with 9 decimal places (ns precision as string)."""
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
            # Simple timestamp for human logs
            t = time.strftime("%Y-%m-%d %H:%M:%S") + f".{int((time.time()%1)*1000):03d}"
            print(
                "[DEBUG]",
                t,
                *a,
                flush=True,
            )


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
        idle_quiet: float = 0.20,
        max_rx_time: float = 1.50,
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
            timeout=float(timeout),
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

    def _read_until_quiet(self) -> bytes:
        start = time.time()
        last_data = start
        buf = bytearray()
        while True:
            chunk = self.ser.read(self.rx_chunk)
            now = time.time()
            if chunk:
                buf.extend(chunk)
                last_data = now
                continue
            if (now - last_data) >= self.idle_quiet:
                break
            if (now - start) >= self.max_rx_time:
                break
        return bytes(buf)

    def txrx(
        self,
        cmd: str,
        sleep_before_read: float = 0.05,
    ) -> str:
        data = cmd.encode("ascii") + b"\r\n"
        self.dbg.log(f"TX {cmd!r} [{len(data)}B] hex={_hexdump(data)}")
        self.ser.reset_input_buffer()
        self.ser.write(data)
        self.ser.flush()
        time.sleep(sleep_before_read)
        raw = self._read_until_quiet()
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
    """Prefer the value after the comma in 'gi:N,<value>' / 'go:N,<value>'."""
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


def parse_id(
    gfw: str,
    gmn: str,
    gmr: str,
    gsn: str,
) -> IdInfo:
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
    return IdInfo(
        firmware=fw,
        model_name=mn,
        model_rev=mr,
        serial_number=sn,
    )


def parse_status(gs: str) -> StatusInfo:
    return StatusInfo(raw=gs.strip(), code=_parse_status_code(gs))


# gmc parser (your unit returns compact "gmc:05-004102")
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
    # dedupe by index
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
    help_txt = cli.txrx("help")
    if not help_txt.strip():
        raise RuntimeError("No response to 'help'. Check wiring, baud, flow, CRLF.")
    resp_gfw = cli.txrx("gfw")
    resp_gmn = cli.txrx("gmn")
    resp_gmr = cli.txrx("gmr")
    resp_gsn = cli.txrx("gsn")
    idinfo = parse_id(
        resp_gfw,
        resp_gmn,
        resp_gmr,
        resp_gsn,
    )
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


def build_csv_header(known_indices: List[int], unknown_indices: List[int]) -> List[str]:
    header_cols: list[str] = ["timestamp"]  # unix seconds, 9 decimals
    for i in known_indices:
        header_cols.append(KNOWN_GI_SCALE[i][0])  # volts
    for i in unknown_indices:
        header_cols.append(f"gi:{i}")  # counts
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
            values.append(f"{(val/divisor):.6f}")  # volts
        time.sleep(sleep_between)
    for i in unknown_indices:
        val, txt = gi_read(client, i)
        if val is None:
            token = (txt or "").strip().replace(",", ";")
            values.append("" if token.lower() == "egi:c" else token)
        else:
            values.append(f"{val:.6f}")  # counts
        time.sleep(sleep_between)
    return values


def format_table(
    idinfo: IdInfo,
    status: StatusInfo,
    readings: list[Reading],
) -> str:
    """Return a human-readable table string for diagnostics."""
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
    "build_csv_header",
    "build_csv_row",
    "gi_read",
    "format_table",
    "unix_ts",
]
