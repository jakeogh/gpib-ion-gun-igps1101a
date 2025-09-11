#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import re
import sys
import time
from dataclasses import asdict
from typing import Any

import click
from rich import print as pprint  # pretty JSON if TTY

from .gpib_ion_gun_igps1101a import KNOWN_GI_SCALE
from .gpib_ion_gun_igps1101a import KNOWN_ORDER
from .gpib_ion_gun_igps1101a import ChannelHint
from .gpib_ion_gun_igps1101a import DebugLog
from .gpib_ion_gun_igps1101a import HostAscii
from .gpib_ion_gun_igps1101a import IdInfo
from .gpib_ion_gun_igps1101a import Reading
from .gpib_ion_gun_igps1101a import StatusInfo
from .gpib_ion_gun_igps1101a import build_csv_header
from .gpib_ion_gun_igps1101a import build_csv_row
from .gpib_ion_gun_igps1101a import build_indices
from .gpib_ion_gun_igps1101a import format_table
from .gpib_ion_gun_igps1101a import gi_read
from .gpib_ion_gun_igps1101a import parse_gmc_hints
from .gpib_ion_gun_igps1101a import parse_id
from .gpib_ion_gun_igps1101a import parse_status
from .gpib_ion_gun_igps1101a import verify_expected_device

CONTEXT = dict(help_option_names=["--help"])


@click.group(context_settings=CONTEXT)
@click.argument("port", metavar="PORT", nargs=1)
@click.option(
    "--baud",
    default=19200,
    show_default=True,
    type=int,
    help="Baud rate.",
)
@click.option(
    "--timeout",
    default=1.0,
    show_default=True,
    type=float,
    help="Per-op serial timeout (s).",
)
@click.option(
    "--xonxoff/--no-xonxoff",
    default=True,
    show_default=True,
    help="Enable/disable software flow control.",
)
@click.option(
    "--expect-model",
    default="IGPS-1101A",
    show_default=True,
    help="Substring to verify against gmn (model name).",
)
@click.option(
    "--debug",
    is_flag=True,
    help="Verbose TX/RX debug logs to stderr.",
)
@click.option(
    "--idle-quiet",
    default=0.20,
    show_default=True,
    type=float,
    help="RX: end read after this many seconds of silence.",
)
@click.option(
    "--max-rx-time",
    default=1.50,
    show_default=True,
    type=float,
    help="RX: hard cap on a single response read (s).",
)
@click.pass_context
def cli(
    ctx: click.Context,
    port: str,
    baud: int,
    timeout: float,
    debug: bool,
    xonxoff: bool,
    expect_model: str,
    idle_quiet: float,
    max_rx_time: float,
) -> None:
    """IGPS-1101A tools (READ-ONLY).  PORT: e.g. /dev/ttyUSB0"""
    dbg = DebugLog(enabled=debug)
    try:
        client = HostAscii(
            port=port,
            baud=baud,
            timeout=timeout,
            xonxoff=xonxoff,
            dbg=dbg,
            idle_quiet=idle_quiet,
            max_rx_time=max_rx_time,
        )
    except Exception as e:
        click.echo(f"ERROR: Unable to open serial port {port}: {e}", err=True)
        raise click.Abort()

    try:
        idinfo = verify_expected_device(client, expect_model, dbg)
    except Exception as e:
        click.echo(f"ERROR: {e}", err=True)
        client.close()
        raise click.Abort()

    ctx.obj = dict(
        client=client,
        dbg=dbg,
        idinfo=idinfo,
        port=port,
        baud=baud,
    )


@cli.result_callback()
def close_client(*args: Any, **kwargs: Any) -> None:
    ctx = click.get_current_context(silent=True)
    if ctx and ctx.obj and "client" in ctx.obj:
        try:
            ctx.obj["client"].close()
        except Exception:
            pass


# ---------------------------- diagnostic subcmd ----------------------------
@cli.command("diagnostic", context_settings=CONTEXT)
@click.option(
    "--probe-min",
    default=0,
    show_default=True,
    type=int,
    help="First channel index to probe with go/gi (read-only).",
)
@click.option(
    "--probe-max",
    default=31,
    show_default=True,
    type=int,
    help="Last channel index to probe with go/gi (read-only).",
)
@click.option(
    "--sleep",
    "sleep_between",
    default=0.06,
    show_default=True,
    type=float,
    help="Sleep between read commands (s).",
)
@click.option(
    "--json-out",
    type=click.Path(dir_okay=False),
    help="Write full JSON dump to this path.",
)
@click.option(
    "--no-table",
    is_flag=True,
    help="Suppress human table output (still writes JSON if --json-out).",
)
@click.pass_context
def diagnostic(
    ctx: click.Context,
    probe_min: int,
    probe_max: int,
    sleep_between: float,
    json_out: None | str,
    no_table: bool,
) -> None:
    """Full sweep of go/gi channels; shows raw counts and any parsed hints."""
    if probe_min < 0 or probe_max < probe_min or probe_max > 255:
        raise click.UsageError(
            "--probe-min/max must define a sane non-negative range (e.g., 0..31)."
        )

    client: HostAscii = ctx.obj["client"]
    dbg: DebugLog = ctx.obj["dbg"]
    idinfo: IdInfo = ctx.obj["idinfo"]

    resp_gs = client.txrx("gs")
    resp_gmc = client.txrx("gmc")
    status = parse_status(resp_gs)
    hints = parse_gmc_hints(resp_gmc)

    readings: list[Reading] = []
    for idx in range(probe_min, probe_max + 1):
        for cmd in ("go", "gi"):
            txt = client.txrx(f"{cmd}:{idx}")
            if not txt.strip():
                dbg.log(f"Empty reply for {cmd}:{idx}")
                continue
            if re.search(r"(invalid|error|unknown|egi:c|ego:c)", txt, re.I):
                dbg.log(f"Rejected {cmd}:{idx}: {txt.strip()}")
                continue
            raw_val = None
            m = re.search(
                r"^\s*(?:gi|go)\s*:\s*\d+\s*,\s*([-+]?\d+(?:\.\d+)?)", txt, re.I
            )
            if m:
                try:
                    raw_val = float(m.group(1))
                except Exception:
                    raw_val = None
            name = units = None
            scaled = None
            if cmd == "gi" and idx in KNOWN_GI_SCALE and raw_val is not None:
                name, div = KNOWN_GI_SCALE[idx]
                units = "V"
                scaled = raw_val / div
            readings.append(
                Reading(
                    cmd=cmd,
                    index=idx,
                    raw_text=txt.strip(),
                    raw_value=raw_val,
                    scaled_value=scaled,
                    name=name,
                    units=units,
                )
            )
            time.sleep(sleep_between)

    doc: dict[str, Any] = {
        "identity": asdict(idinfo),
        "status": asdict(status),
        "config_text": resp_gmc,
        "readings": [asdict(r) for r in readings],
        "port": ctx.obj["port"],
        "baud": ctx.obj["baud"],
        "timestamp": time.time(),
    }
    if not no_table:
        print(format_table(idinfo, status, readings))
    if json_out:
        with open(json_out, "w", encoding="utf-8") as f:
            json.dump(doc, f, indent=2)
        print(f"Wrote JSON to {json_out}")
    else:
        if sys.stdout.isatty() and pprint is not print:
            pprint(doc)  # type: ignore[misc]
        else:
            print(json.dumps(doc, indent=2))


# ------------------------------- read subcmd --------------------------------
@cli.command("read", context_settings=CONTEXT)
@click.option(
    "--probe-min",
    default=0,
    show_default=True,
    type=int,
    help="First GI channel index to include (read-only).",
)
@click.option(
    "--probe-max",
    default=31,
    show_default=True,
    type=int,
    help="Last GI channel index to include (read-only).",
)
@click.option(
    "--only-known/--all-gi",
    default=False,
    show_default=True,
    help="Poll only mapped channels (faster).",
)
@click.option(
    "--sleep",
    "sleep_between",
    default=0.02,
    show_default=True,
    type=float,
    help="Sleep between GI reads (s).",
)
@click.option(
    "--csv-header/--no-csv-header",
    default=True,
    show_default=True,
    help="Print header before the data row.",
)
@click.option(
    "--autowrite",
    type=click.Path(exists=True, file_okay=False, dir_okay=True, resolve_path=True),
    help=(
        "Existing directory; auto-create igps_1101a/YYYY-MM-DD and write a CSV "
        "named '<epoch>_read.csv'. Also prints to stdout."
    ),
)
@click.pass_context
def read_cmd(
    ctx: click.Context,
    probe_min: int,
    probe_max: int,
    only_known: bool,
    sleep_between: float,
    csv_header: bool,
    autowrite: None | str,
) -> None:
    """
    One-shot CSV of GI readings:
    - Known channels in volts (engineering units).
    - Unknown channels as counts columns named 'gi:<index>'.
    Timestamp is unix seconds with 9 decimals.
    """
    if probe_min < 0 or probe_max < probe_min or probe_max > 255:
        raise click.UsageError(
            "--probe-min/max must define a sane non-negative range (e.g., 0..31)."
        )

    client: HostAscii = ctx.obj["client"]
    known_indices, unknown_indices = build_indices(probe_min, probe_max, only_known)

    header_cols = build_csv_header(known_indices, unknown_indices)
    values = build_csv_row(
        client,
        known_indices,
        unknown_indices,
        sleep_between,
    )

    out = sys.stdout
    if csv_header:
        print(",".join(header_cols), file=out)
    print(",".join(values), file=out)

    # Optional autowrite to file while keeping stdout output
    if autowrite:
        date_str = time.strftime("%Y-%m-%d")
        epoch_s = int(time.time())
        dest_dir = os.path.join(autowrite, "igps_1101a", date_str)
        os.makedirs(dest_dir, exist_ok=True)
        auto_path = os.path.join(dest_dir, f"{epoch_s}_read.csv")
        try:
            with open(auto_path, "w", encoding="utf-8") as fp:
                if csv_header:
                    fp.write(",".join(header_cols) + "\n")
                fp.write(",".join(values) + "\n")
            click.echo(f"[autowrite] wrote {auto_path}", err=True)
        except Exception as e:
            click.echo(f"[autowrite] ERROR writing {auto_path}: {e}", err=True)
            raise


# -------------------------------- log subcmd --------------------------------
@cli.command("log", context_settings=CONTEXT)
@click.option(
    "--probe-min",
    default=0,
    show_default=True,
    type=int,
    help="First GI channel index to include (read-only).",
)
@click.option(
    "--probe-max",
    default=31,
    show_default=True,
    type=int,
    help="Last GI channel index to include (read-only).",
)
@click.option(
    "--only-known/--all-gi",
    default=False,
    show_default=True,
    help="Poll only mapped channels (faster).",
)
@click.option(
    "--sleep",
    "sleep_between",
    default=0.02,
    show_default=True,
    type=float,
    help="Sleep between GI reads (s).",
)
@click.option(
    "--interval",
    default=5.0,
    show_default=True,
    type=float,
    help="Seconds between snapshots.",
)
@click.option(
    "--count",
    default=0,
    show_default=True,
    type=int,
    help="Number of samples (0 = run until Ctrl-C).",
)
@click.option(
    "--out",
    "out_path",
    type=click.Path(dir_okay=False),
    help="CSV file to write. Defaults to stdout if omitted.",
)
@click.option(
    "--append/--no-append",
    default=True,
    show_default=True,
    help="Append to --out if it exists; otherwise overwrite.",
)
@click.option(
    "--header/--no-header",
    "write_header",
    default=True,
    show_default=True,
    help="Write header row (stdout writes once).",
)
@click.option(
    "--autowrite",
    type=click.Path(exists=True, file_okay=False, dir_okay=True, resolve_path=True),
    help=(
        "Existing directory; auto-create igps_1101a/YYYY-MM-DD and write a CSV "
        "named '<epoch>_log.csv'. Also prints to stdout."
    ),
)
@click.pass_context
def log_cmd(
    ctx: click.Context,
    probe_min: int,
    probe_max: int,
    only_known: bool,
    sleep_between: float,
    interval: float,
    count: int,
    out_path: None | str,
    append: bool,
    write_header: bool,
    autowrite: None | str,
) -> None:
    """
    Periodic CSV snapshots:
    - Known channels in volts (engineering units).
    - Unknown channels as counts columns named 'gi:<index>'.
    Timestamp is unix seconds with 9 decimals.
    """
    if probe_min < 0 or probe_max < probe_min or probe_max > 255:
        raise click.UsageError(
            "--probe-min/max must define a sane non-negative range (e.g., 0..31)."
        )
    if interval <= 0:
        raise click.UsageError("--interval must be > 0")

    client: HostAscii = ctx.obj["client"]
    known_indices, unknown_indices = build_indices(probe_min, probe_max, only_known)

    header_cols = build_csv_header(known_indices, unknown_indices)
    header_line = ",".join(header_cols)

    out = sys.stdout
    fp = None
    if out_path:
        mode = "a" if append else "w"
        fp = open(out_path, mode, encoding="utf-8")
        out = fp
        header_already = (
            append and os.path.exists(out_path) and os.path.getsize(out_path) > 0
        )
        if write_header and not header_already:
            print(header_line, file=out, flush=True)
    else:
        if write_header:
            print(header_line, file=out, flush=True)

    # Prepare autowrite file (new file each run, prefixed with epoch seconds)
    auto_fp = None
    auto_path = None
    if autowrite:
        try:
            date_str = time.strftime("%Y-%m-%d")
            epoch_s = int(time.time())
            dest_dir = os.path.join(autowrite, "igps_1101a", date_str)
            os.makedirs(dest_dir, exist_ok=True)
            auto_path = os.path.join(dest_dir, f"{epoch_s}_log.csv")
            auto_fp = open(auto_path, "w", encoding="utf-8")
            if write_header:
                print(header_line, file=auto_fp, flush=True)
            click.echo(f"[autowrite] writing to {auto_path}", err=True)
        except Exception as e:
            click.echo(f"[autowrite] ERROR creating file: {e}", err=True)
            auto_fp = None
            auto_path = None

    n = 0
    next_t = time.monotonic()
    try:
        while True:
            values = build_csv_row(
                client,
                known_indices,
                unknown_indices,
                sleep_between,
            )
            line = ",".join(values)
            # Always write to the primary destination
            print(line, file=out, flush=True)
            # Also write to autowrite file if enabled
            if autowrite and auto_fp:
                print(line, file=auto_fp, flush=True)
            # And ensure values go to stdout when autowrite is used,
            # even if primary 'out' is a file.
            if autowrite and out is not sys.stdout:
                print(line, file=sys.stdout, flush=True)

            n += 1
            if count and n >= count:
                break

            next_t += interval
            time.sleep(max(0.0, next_t - time.monotonic()))
    except KeyboardInterrupt:
        click.echo("\nStopped by user (Ctrl-C).", err=True)
    finally:
        if fp:
            fp.flush()
            fp.close()
        if autowrite and auto_fp:
            try:
                auto_fp.flush()
                auto_fp.close()
                click.echo(f"[autowrite] closed {auto_path}", err=True)
            except Exception:
                pass
