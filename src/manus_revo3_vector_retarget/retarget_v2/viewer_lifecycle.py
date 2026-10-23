"""Wait for the passive viewer's native rendering cleanup before process exit."""
from contextlib import contextmanager
import threading


@contextmanager
def passive_viewer(model, data, **kwargs):
    import mujoco.viewer

    before = set(threading.enumerate())
    handle = mujoco.viewer.launch_passive(model, data, **kwargs)
    # On Linux MuJoCo starts a daemon _launch_internal thread but exposes no
    # public join handle. Track only viewer-owned threads created by this call.
    # Handle.close()/is_running() signal exit before simulate.destroy finishes.
    owned = [thread for thread in threading.enumerate() if thread not in before
             and getattr(getattr(thread, "_target", None), "__module__", None) == "mujoco.viewer"]
    try:
        yield handle
    finally:
        handle.close()
        for thread in owned:
            thread.join(timeout=5.)
            if thread.is_alive():
                raise RuntimeError("MuJoCo viewer thread did not finish shutdown within 5 seconds")
