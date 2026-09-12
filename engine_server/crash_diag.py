"""Fatal-signal crash log for the engine server process (engine-local).

Same artifact contract as the environment's own crash diagnostics — both
programs write into one crash-log directory, so their files must look alike:
``<KICAD_CRASH_LOG_DIR>/<yymmdd_HHMMSS>_engine_server_<host>_<pid>.log``,
native C++ backtrace first (``crashtrace.c``, compiled on first use next to
the logs), then Python's faulthandler stacks appended. A clean exit removes
the empty log, so only real crashes leave a file. ``KICAD_CRASH_DIAG=0``
turns everything off; ``KICAD_CRASH_LOG_DIR`` chooses the directory (the
environment's client points it at its own tree).

Bundle-local on purpose: the server imports nothing from the environment
package. Unlike ``wire.py`` this is not a shared contract — the environment
keeps its own implementation for its own processes.
"""

from __future__ import annotations

import ctypes
import faulthandler
import os
import socket
import subprocess
import time
from typing import TextIO

_HERE = os.path.dirname(os.path.abspath(__file__))
_installed: tuple[str, TextIO] | None = None


def install_crash_handler(role: str = "engine_server") -> tuple[str, TextIO] | None:
    """Install per-process fatal-signal logging; returns (log_path, file).

    Idempotent per process. Fail-soft: without gcc the native backtrace is
    skipped and faulthandler alone writes the Python stacks; if even that
    fails, returns None. Never raises — a failure here must not block startup.
    """
    global _installed
    if os.environ.get("KICAD_CRASH_DIAG", "1") == "0":
        return None
    if _installed is not None:
        return _installed
    try:
        log_dir = os.environ.get("KICAD_CRASH_LOG_DIR") or os.path.join(
            os.path.dirname(_HERE), "var", "crashlogs")
        os.makedirs(log_dir, exist_ok=True)
        host = socket.gethostname().split(".")[0]
        stem = f"{time.strftime('%y%m%d_%H%M%S')}_{role}_{host}_{os.getpid()}"
        path = os.path.join(log_dir, stem + ".log")
        log = open(path, "a")  # O_APPEND: faulthandler's writes land after the native part
        faulthandler.enable(file=log, all_threads=True)
        _installed = (path, log)
    except Exception:  # noqa: BLE001
        return None
    try:  # native part — degrade silently to faulthandler-only (e.g. no gcc)
        so = _compile_crashtrace(log_dir)
        os.environ["CRASHTRACE_FILE"] = path
        ctypes.CDLL(so)  # constructor installs the handlers, chaining to faulthandler
    except Exception:  # noqa: BLE001
        pass
    return _installed


def remove_log_if_empty() -> None:
    """Clean-exit teardown: no crash happened, leave no trace."""
    if _installed is None:
        return
    path, log = _installed
    try:
        faulthandler.disable()
        if not log.closed:
            log.close()
        if os.path.getsize(path) == 0:
            os.remove(path)
    except OSError:
        pass


def _compile_crashtrace(log_dir: str) -> str:
    src = os.path.join(_HERE, "crashtrace.c")
    so = os.path.join(log_dir, "crashtrace_engine.so")
    if not os.path.exists(so) or os.path.getmtime(so) < os.path.getmtime(src):
        tmp = f"{so}.{os.getpid()}.tmp"  # processes may race-compile: build + atomic replace
        subprocess.run(
            ["gcc", "-shared", "-fPIC", "-O1", "-o", tmp, src],
            check=True, capture_output=True,
        )
        os.replace(tmp, so)
    return so
