"""No GUI: verify render cleanup completes even when a consumer raises."""
import threading
import time
from types import SimpleNamespace

import mujoco.viewer
import pytest

from retarget_v2.viewer_lifecycle import passive_viewer


@pytest.mark.parametrize("fail", [False, True])
def test_viewer_context_joins_its_async_render_cleanup(monkeypatch, fail):
    stop, cleaned = threading.Event(), threading.Event()
    threads = []

    def render():
        stop.wait(2.)
        time.sleep(.02)  # close returns before native render cleanup completes.
        cleaned.set()

    render.__module__ = "mujoco.viewer"

    def launch(*args, **kwargs):
        thread = threading.Thread(target=render, daemon=True)
        threads.append(thread)
        thread.start()
        return SimpleNamespace(close=stop.set)

    monkeypatch.setattr(mujoco.viewer, "launch_passive", launch)
    try:
        with passive_viewer(None, None):
            if fail:
                raise ValueError("consumer error")
    except ValueError:
        assert fail
    assert cleaned.is_set()
    assert not threads[0].is_alive()
