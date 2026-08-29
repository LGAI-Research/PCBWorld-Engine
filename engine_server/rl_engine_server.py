"""Engine server: hosts kicad_rl_router.RLRouter behind a unix socket.

This is the only process in the project that loads the KiCad shared library
(``kicad_rl_router.so``). The environment's client — ``router_client.py`` in
its own repository — spawns one server per engine and talks a
primitives-only pickle protocol, so no KiCad type ever crosses the process
boundary.

Protocol (length-prefixed pickle, one request/response pair at a time):
  server → client on connect: handshake
      {"protocol": 2, "schema": KRL_FIELDS, "constants": {name: int},
       "pid": int}
    The schema is validated against the LIVE binding at startup and the
    client validates it against its own copy of ``wire.py`` — a drifted
    binding fails loudly on both sides (the "constant handshake").
  client → server requests, server → client {"ok": True, "value": wire}
  or {"ok": False, "etype", "msg", "tb"} (client re-raises):
      ("construct",  kwargs)              build the RLRouter (refuses if
                                          one is live — KiCad global state
                                          allows one router per process)
      ("call",       (name, args, kwargs))  getattr(router, name)(*args, **kwargs)
      ("batch",      [(name, args, kwargs), ...])  sequential calls, one roundtrip
      ("module_call",(name, args))        getattr(kicad_rl_router, name)(*args)
      ("close_router", None)              cancel sessions + drop the router
      ("ping",       None)                liveness probe
  Protocol 2 (port v0.31) added the kwargs slot to call/batch — the binding
  surface now has keyword-arg call sites (``cleanup_tracks``).
  EOF / socket close → server exits (ties lifetime to the client).

No per-call timeouts by design: routing calls have legitimate multi-minute
outliers (shove). A wedged server is killed by the client via process kill.

This repository is self-contained: nothing here imports the environment
package. ``wire.py`` — the protocol both programs speak — ships on both
sides as a byte-identical copy, and ``thread_cap.py`` is the bundle-local
thread-pool cap.
"""

from __future__ import annotations

import faulthandler
import os
import pickle
import signal
import socket
import struct
import sys
import traceback

_BUNDLE_DIR = os.path.dirname(os.path.abspath(__file__))
if _BUNDLE_DIR not in sys.path:
    sys.path.insert(0, _BUNDLE_DIR)
_REPO_ROOT = os.path.dirname(_BUNDLE_DIR)

# kicad_rl_router.so resolution — the same env-var contract the client uses
# (the client's sys.path is not inherited).
_RL_MODULE_DIR = os.environ.get("CADAGENT_KICAD_RL_MODULE_DIR")
if _RL_MODULE_DIR:
    _RL_LIB_PATH = _RL_MODULE_DIR
else:
    _RL_LIB_PATH = os.path.join(
        os.environ.get("CADAGENT_KICAD_RL_BUILD_DIR",
                       os.path.join(_REPO_ROOT, "build_rl")),
        "pcbnew", "python", "rl")
if os.path.isdir(_RL_LIB_PATH) and _RL_LIB_PATH not in sys.path:
    sys.path.insert(0, _RL_LIB_PATH)

from wire import (  # noqa: E402  (bundle-local — see _BUNDLE_DIR above)
    KRL_CONSTANT_NAMES,
    KRL_FIELDS,
    from_wire,
    to_wire,
)

_LEN = struct.Struct(">Q")
PROTOCOL_VERSION = 2

# Kept alive for the process lifetime: faulthandler writes to this fd from a
# signal handler, so it must not be garbage-collected.
_CRASH_LOG = None


def _send(sock: socket.socket, obj) -> None:
    data = pickle.dumps(obj, protocol=pickle.HIGHEST_PROTOCOL)
    sock.sendall(_LEN.pack(len(data)) + data)


def _recv(sock: socket.socket):
    hdr = _recv_exact(sock, _LEN.size)
    if hdr is None:
        return None
    (n,) = _LEN.unpack(hdr)
    data = _recv_exact(sock, n)
    if data is None:
        return None
    return pickle.loads(data)


def _recv_exact(sock: socket.socket, n: int):
    chunks = []
    while n:
        try:
            b = sock.recv(min(n, 1 << 20))
        except ConnectionError:
            return None
        if not b:
            return None
        chunks.append(b)
        n -= len(b)
    return b"".join(chunks)


def _validate_schema(krl) -> None:
    """Fail loudly at startup if the live binding drifted from the registry."""
    mismatches = []
    for tname, fields in KRL_FIELDS.items():
        cls = getattr(krl, tname, None)
        if cls is None:
            mismatches.append(f"{tname}: missing in binding")
            continue
        live = {n for n in dir(cls) if not n.startswith("_")}
        if set(fields) != live:
            mismatches.append(
                f"{tname}: registry-only={sorted(set(fields) - live)} "
                f"binding-only={sorted(live - set(fields))}")
    missing_consts = [c for c in KRL_CONSTANT_NAMES if not hasattr(krl, c)]
    if missing_consts:
        mismatches.append(f"constants missing in binding: {missing_consts}")
    if mismatches:
        raise RuntimeError(
            "engine server: wire schema drifted from live binding — update "
            "wire.py (both copies):\n  " + "\n  ".join(mismatches))


def _plain_to_native(krl, value):
    """Convert plain mirror args (DesignRules/NetClassInfo) to native objects.

    Everything else passes through — the binding surface takes only
    primitives apart from ``set_design_rules``.
    """
    tname = type(value).__name__
    if tname == "DesignRules":
        native = krl.DesignRules()
        for f in KRL_FIELDS["DesignRules"]:
            v = getattr(value, f)
            if f == "default_netclass":
                v = _plain_to_native(krl, v)
            elif f == "netclasses":
                v = [_plain_to_native(krl, n) for n in v]
            setattr(native, f, v)
        return native
    if tname == "NetClassInfo":
        native = krl.NetClassInfo()
        for f in KRL_FIELDS["NetClassInfo"]:
            setattr(native, f, getattr(value, f))
        return native
    if isinstance(value, list):
        return [_plain_to_native(krl, v) for v in value]
    return value


class _EngineHost:
    """One RLRouter (at most) + request dispatch."""

    def __init__(self, krl):
        self.krl = krl
        self.router = None

    # --- ops ---

    def construct(self, kwargs: dict) -> dict:
        if self.router is not None:
            raise RuntimeError(
                "engine server: construct while a router is live — KiCad "
                "global state allows one router per process; close_router "
                "first")
        # Strict load contract (develop v0.28+): open the SOURCE file directly,
        # no upgrade/normalize cache. The post-load contract checks
        # (was_legacy_design_settings_loaded / was_project_loaded_from_file /
        # allow_default_rules) run ENV-SIDE through ordinary proxy calls.
        board_path = kwargs["board_path"]
        self.router = self.krl.RLRouter(
            str(board_path),
            kwargs.get("project_path") or "",
            int(kwargs.get("seed", -1)),
            int(kwargs.get("shove_iter_limit", 250)),
            int(kwargs.get("followbranch_iter_limit", 1_000_000)),
        )
        return {"board_path": str(board_path)}

    def close_router(self) -> None:
        r, self.router = self.router, None
        if r is not None:
            try:
                if r.is_routing():
                    r.cancel_route()
                if r.is_dragging():
                    r.cancel_drag()
            finally:
                r = None

    def call(self, name: str, args, kwargs=None) -> object:
        if self.router is None:
            raise RuntimeError(
                f"engine server: call({name!r}) before construct")
        fn = getattr(self.router, name)
        return fn(
            *[_plain_to_native(self.krl, from_wire(a)) for a in args],
            **{k: _plain_to_native(self.krl, from_wire(v))
               for k, v in (kwargs or {}).items()})

    def dispatch(self, op: str, payload):
        if op == "call":
            name, args, kwargs = payload
            return to_wire(self.call(name, args, kwargs))
        if op == "batch":
            return [to_wire(self.call(name, args, kwargs))
                    for name, args, kwargs in payload]
        if op == "construct":
            return self.construct(payload)
        if op == "close_router":
            self.close_router()
            return None
        if op == "module_call":
            name, args = payload
            return to_wire(getattr(self.krl, name)(*[from_wire(a) for a in args]))
        if op == "ping":
            return "pong"
        raise RuntimeError(f"engine server: unknown op {op!r}")


def _cleanup_ipc_dir(sock_path: str) -> None:
    """Best-effort removal of the client-created ``/tmp/krl_ipc_*`` dir.

    Clients that exit without killing their servers leave the dir behind
    (multiprocessing workers skip atexit via os._exit; kill -9): on EOF /
    SIGTERM this process is the last one standing, so it removes its own
    socket dir. Guarded to ``krl_ipc_`` basenames so a manually chosen
    socket path never gets its parent dir deleted. The client's ``kill()``
    performs the same removal — every unlink/rmdir is OSError-guarded on
    both sides, so the double cleanup is race-free.
    """
    d = os.path.dirname(os.path.abspath(sock_path))
    if not os.path.basename(d).startswith("krl_ipc_"):
        try:
            os.unlink(sock_path)
        except OSError:
            pass
        return
    for p in (sock_path, os.path.join(d, "server_stderr.log")):
        try:
            os.unlink(p)
        except OSError:
            pass
    try:
        os.rmdir(d)
    except OSError:
        pass


def _install_crash_handler() -> None:
    """Dump this process's stack to the crash-log dir on a fatal signal.

    The router is C++, so a segfault or abort here kills the process with no
    Python traceback. faulthandler writes the interpreter-level stack (and
    the C frames the signal handler can reach) to a per-pid file, which the
    client points at in its crash message. Same env-var contract as the
    environment's diagnostics: ``KICAD_CRASH_LOG_DIR`` chooses the directory
    (the client sets it to its own tree), ``KICAD_CRASH_DIAG=0`` turns it
    off. Best effort — a failure here must never block startup.
    """
    if os.environ.get("KICAD_CRASH_DIAG", "1") == "0":
        return
    try:
        log_dir = os.environ.get(
            "KICAD_CRASH_LOG_DIR", os.path.join(_REPO_ROOT, "var", "crashlogs"))
        os.makedirs(log_dir, exist_ok=True)
        global _CRASH_LOG
        _CRASH_LOG = open(
            os.path.join(log_dir, f"engine_server_{os.getpid()}.log"), "w")
        faulthandler.enable(file=_CRASH_LOG, all_threads=True)
    except Exception as exc:  # noqa: BLE001
        print(f"engine server: crash handler unavailable ({exc})",
              file=sys.stderr, flush=True)


def _drop_empty_crash_log() -> None:
    """A clean exit leaves no artifact — only real crashes write to the log."""
    if _CRASH_LOG is None:
        return
    try:
        name = _CRASH_LOG.name
        _CRASH_LOG.flush()
        if os.path.getsize(name) == 0:
            faulthandler.disable()
            _CRASH_LOG.close()
            os.unlink(name)
    except OSError:
        pass


def main(sock_path: str) -> None:
    _install_crash_handler()

    import kicad_rl_router as krl

    _validate_schema(krl)

    from thread_cap import apply_thread_pool_cap
    apply_thread_pool_cap(krl)

    if os.path.exists(sock_path):
        os.unlink(sock_path)
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(sock_path)
    srv.listen(1)

    def _on_sigterm(*_a):
        _cleanup_ipc_dir(sock_path)
        _drop_empty_crash_log()
        sys.exit(0)

    signal.signal(signal.SIGTERM, _on_sigterm)

    conn, _ = srv.accept()
    srv.close()
    host = _EngineHost(krl)
    _send(conn, {
        "protocol": PROTOCOL_VERSION,
        "schema": KRL_FIELDS,
        "constants": {c: getattr(krl, c) for c in KRL_CONSTANT_NAMES},
        "pid": os.getpid(),
    })

    while True:
        msg = _recv(conn)
        if msg is None:          # client gone → die with it
            break
        op, payload = msg
        try:
            value = host.dispatch(op, payload)
            reply = {"ok": True, "value": value}
        except SystemExit:
            raise
        except BaseException as exc:  # noqa: BLE001 — ship the error to the client
            reply = {"ok": False, "etype": type(exc).__name__,
                     "msg": str(exc), "tb": traceback.format_exc()}
        _send(conn, reply)

    host.close_router()
    conn.close()
    _cleanup_ipc_dir(sock_path)
    _drop_empty_crash_log()


if __name__ == "__main__":
    main(sys.argv[1])
