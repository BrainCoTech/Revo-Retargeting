"""ROS-independent bone-length preparation with a verified on-disk cache.

The caller owns timing: call ``advance`` once per fresh, newly received sample
and ``clear_pending`` after an input gap. This module does not infer freshness
from SDK poses, which have no acquisition timestamp. Pause/resume does not
require any call here; only an explicit ``reset`` discards a ready calibration.
"""
from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import tempfile

import numpy as np

from .input import Calibration, chains, decode, normalize_hand_side, palm_coordinates


CACHE_SCHEMA_VERSION = 1
# Local semantic contract, not a version supplied by the MANUS message. Bump
# when landmark meanings, topology, units or coordinate conventions change.
SKELETON_PROFILE = "manus_sdk_common_25_v1"
CACHE_CHECK_FRAMES = 30


def default_cache_dir():
    root = Path(os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache")
    return root.expanduser() / "manus_revo3_vector_retarget"


class CalibrationSession:
    """One selected glove's preparation state, owned by the retarget thread.

    ``advance`` returns a Calibration only when ready. Invalid input raises
    ValueError and clears unfinished samples. Cache errors are nonfatal and
    available in ``status``; a successful fresh calibration remains usable
    even if the cache directory cannot be written.
    """

    def __init__(self, calibration_frames=120, hand_side="right", normal_sign=-1., cache_dir=None):
        if (isinstance(calibration_frames, bool) or not isinstance(calibration_frames, int)
                or calibration_frames < 30):
            raise ValueError("calibration_frames must be an integer >=30")
        if isinstance(normal_sign, bool) or normal_sign not in (-1., 1.):
            raise ValueError("normal_sign must explicitly be -1 or +1")
        self.calibration_frames = calibration_frames
        self.hand_side = normalize_hand_side(hand_side)
        self.normal_sign = float(normal_sign)
        self.cache_dir = default_cache_dir() if cache_dir is None else Path(cache_dir).expanduser()
        self.glove_id = None
        self.calibration = None
        self._candidate = None
        self._frames = []
        self._cache_checked = False
        self._cache_used = False
        self._cache_error = self._last_error = None
        self._completed_frames = 0

    @property
    def cache_path(self):
        if self.glove_id is None:
            return None
        sign = "negative" if self.normal_sign == -1. else "positive"
        return self.cache_dir / f"{self.glove_id}_{self.hand_side}_{sign}_{SKELETON_PROFILE}.json"

    @property
    def progress(self):
        required = CACHE_CHECK_FRAMES if self._candidate is not None else self.calibration_frames
        if self.calibration is not None:
            return self._completed_frames, self._completed_frames
        return len(self._frames), required

    def status(self):
        if self.calibration is not None:
            state = "ready"
        elif self.glove_id is None:
            state = "waiting_input"
        else:
            state = "checking_cache" if self._candidate is not None else "calibrating"
        count, required = self.progress
        return dict(state=state, count=count, required_frames=required,
                    glove_id=self.glove_id, hand_side=self.hand_side, normal_sign=self.normal_sign,
                    skeleton_profile=SKELETON_PROFILE, cache_used=self._cache_used,
                    cache_path=None if self.cache_path is None else str(self.cache_path),
                    cache_error=self._cache_error, last_error=self._last_error)

    def clear_pending(self):
        """Discard incomplete sampling after a gap, preserving ready results."""
        self._frames.clear()

    def reset(self, force=True):
        """Begin asynchronous preparation, keeping the selected glove identity.

        A forced recalibration bypasses the existing file but keeps it intact
        until a new full calibration succeeds. ``force=False`` retries cache
        loading and its fresh-frame validation window.
        """
        self.calibration = self._candidate = None
        self.clear_pending()
        self._cache_checked = bool(force)
        self._cache_used = False
        self._completed_frames = 0
        self._cache_error = self._last_error = None

    def _identity(self):
        return dict(schema_version=CACHE_SCHEMA_VERSION, skeleton_profile=SKELETON_PROFILE,
                    glove_id=self.glove_id, hand_side=self.hand_side, normal_sign=self.normal_sign)

    def _load(self):
        try:
            value = json.loads(self.cache_path.read_text())
            if not isinstance(value, dict) or any(value.get(k) != v for k, v in self._identity().items()):
                raise ValueError("Cache identity or skeleton version does not match")
            if (type(value["schema_version"]) is not int or type(value["glove_id"]) is not int
                    or isinstance(value["normal_sign"], bool)):
                raise ValueError("Cache identity has invalid field types")
            count = value["sample_count"]
            if isinstance(count, bool) or not isinstance(count, int) or count < 30:
                raise ValueError("Cache sample_count must be an integer >=30")
            calibration = Calibration(value["lengths_m"], self.normal_sign, self.hand_side)
            if np.any(calibration.lengths < .008) or np.any(calibration.lengths > .1):
                raise ValueError("Cached bone lengths are outside 8..100 mm")
            return calibration
        except FileNotFoundError:
            return None
        except (OSError, UnicodeError, ValueError, KeyError, TypeError) as exc:
            self._cache_error = f"Cache ignored: {exc}"
            return None

    def _save(self, calibration, maximum_deviation):
        temporary = None
        try:
            self.cache_dir.mkdir(parents=True, exist_ok=True)
            value = dict(self._identity(), lengths_m=calibration.lengths.tolist(),
                         sample_count=len(self._frames), max_length_deviation=maximum_deviation,
                         created_at=datetime.now(timezone.utc).isoformat())
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=self.cache_dir,
                                             prefix=self.cache_path.name + ".", suffix=".tmp",
                                             delete=False) as stream:
                temporary = Path(stream.name)
                json.dump(value, stream, indent=2, allow_nan=False)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.cache_path)
            self._cache_error = None
        except OSError as exc:
            self._cache_error = f"Calibration ready; cache could not be saved: {exc}"
        finally:
            if temporary is not None:
                try:
                    temporary.unlink(missing_ok=True)
                except OSError:
                    pass

    def advance(self, message):
        """Validate and accumulate one new fresh frame, returning the ready result."""
        try:
            identity = message["glove_id"]
            if (isinstance(identity, bool) or not isinstance(identity, int)
                    or not -(2**31) <= identity < 2**31):
                raise ValueError("glove_id must be a signed 32-bit integer")
            if self.glove_id is not None and identity != self.glove_id:
                raise ValueError("Glove identity changed")
            local = chains(palm_coordinates(decode(message, self.hand_side), self.normal_sign))
            lengths = np.linalg.norm(np.diff(local, axis=1), axis=2)
            if np.any(lengths < .008) or np.any(lengths > .1):
                raise ValueError("Unexpected bone lengths; expected 8..100 mm in metre units")
            if self.glove_id is None:
                self.glove_id = identity
            if self.calibration is not None:
                self.calibration.features(message)
                self._last_error = None
                return self.calibration
            if not self._cache_checked:
                self._candidate = self._load()
                self._cache_checked = True
            self._frames.append(deepcopy(message))
            if len(self._frames) < self.progress[1]:
                self._last_error = None
                return None
            measured = Calibration.from_frames(self._frames, self.normal_sign, self.hand_side)
            if self._candidate is not None:
                deviation = float(np.max(np.abs(measured.lengths / self._candidate.lengths - 1.)))
                if deviation > .1:
                    self._candidate = None
                    self.clear_pending()
                    self._cache_error = f"Cached bone lengths differ by {deviation:.1%}; collecting full calibration"
                    self._last_error = None
                    return None
                self.calibration = self._candidate
                self._cache_used = True
            else:
                lengths = np.array([np.linalg.norm(np.diff(chains(decode(frame, self.hand_side)), axis=1), axis=2)
                                    for frame in self._frames])
                deviation = float(np.max(np.abs(lengths / measured.lengths - 1.)))
                self._save(measured, deviation)
                self.calibration = measured
            self._completed_frames = len(self._frames)
            self.clear_pending()
            self._last_error = None
            return self.calibration
        except (KeyError, TypeError, ValueError, AttributeError, FloatingPointError) as exc:
            self.clear_pending()
            self._last_error = str(exc)
            raise ValueError(str(exc)) from exc
