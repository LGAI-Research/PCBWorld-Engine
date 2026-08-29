"""KiCad thread-pool cap for the engine server process.

Bundle-local so the server imports nothing from the environment package.
The environment keeps its own copy for the in-process build path it still
supports; the two are independent — this one is not a shared contract like
``wire.py``.
"""

from __future__ import annotations

import os
import warnings

__all__ = ["apply_thread_pool_cap"]


# Once-per-process guard: the pool is process-global and each reset
# destroys/respawns its threads, so re-applying on every engine
# construction (board reloads recreate the engine) would be pure churn.
_THREAD_POOL_CAP_APPLIED = False


def apply_thread_pool_cap(krl) -> None:
    """Cap the process-global KiCad thread pool (DRC / build_connectivity).

    The native pool defaults to ``hardware_concurrency()`` threads *per
    process*; with N parallel env workers that means N × nproc threads
    fighting over the same cores. Policy via ``KICAD_ENGINE_THREADS``:

        unset / "1"   → 1 thread (default: each worker stays single-threaded)
        "<int>"       → that many threads
        "physical"    → the host's physical core count

    Called with the imported ``kicad_rl_router`` module, once per process,
    before the first ``RLRouter`` construction. No-op (with a warning) on
    router builds that predate ``set_thread_pool_size``.

    Why a pybind API instead of KiCad's ``kicad_advanced`` config file
    (``MaximumThreads`` + ``KICAD_CONFIG_HOME``): headless bindings never
    read that file — ``ADVANCED_CFG::loadFromConfigFile()`` returns early
    when ``!wxTheApp``, so the config route is silently a no-op here.
    """
    global _THREAD_POOL_CAP_APPLIED
    if _THREAD_POOL_CAP_APPLIED:
        return
    _THREAD_POOL_CAP_APPLIED = True

    if not hasattr(krl, "set_thread_pool_size"):
        warnings.warn(
            "kicad_rl_router build predates set_thread_pool_size(); KiCad "
            "thread pool left at hardware_concurrency(). Rebuild via "
            "build_rl_router.sh to enable the cap.",
            RuntimeWarning,
        )
        return

    raw = os.environ.get("KICAD_ENGINE_THREADS", "1").strip().lower()
    if raw == "physical":
        import psutil

        n = psutil.cpu_count(logical=False) or os.cpu_count() or 1
    else:
        try:
            n = int(raw)
        except ValueError:
            raise ValueError(
                f"KICAD_ENGINE_THREADS must be a positive integer or "
                f"'physical', got {raw!r}"
            ) from None
        if n < 1:
            raise ValueError(f"KICAD_ENGINE_THREADS must be >= 1, got {n}")
    krl.set_thread_pool_size(n)
