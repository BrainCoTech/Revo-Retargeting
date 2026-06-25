"""Retargeting algorithms registry and base class.

Usage:
    from manus_revo2_retarget.retargeters import RetargeterRegistry
    retargeter = RetargeterRegistry.create("dex_vector", config_path)
"""

import importlib
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Callable, ClassVar, Dict, Tuple

import numpy as np


class BaseRetargeter(ABC):
    """Abstract base class for hand retargeting algorithms.

    All retargeting implementations must subclass this and implement
    ``retarget_process``.
    """

    @abstractmethod
    def retarget_process(
        self,
        left_finger_tip_pos: list,
        right_finger_tip_pos: list,
        left_thumb_dip_pos: list | None = None,
        left_thumb_pip_pos: list | None = None,
        right_thumb_dip_pos: list | None = None,
        right_thumb_pip_pos: list | None = None,
        left_ergonomics: dict | None = None,
        right_ergonomics: dict | None = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Compute target joint positions from finger tip positions.

        Args:
            left_finger_tip_pos: 5 fingertip positions for the left hand.
            right_finger_tip_pos: 5 fingertip positions for the right hand.
            left_thumb_dip_pos: Optional thumb DIP position for the left hand.
            left_thumb_pip_pos: Optional thumb PIP position for the left hand.
            right_thumb_dip_pos: Optional thumb DIP position for the right hand.
            right_thumb_pip_pos: Optional thumb PIP position for the right hand.
            left_ergonomics: Optional Manus ergonomics values keyed by type.
            right_ergonomics: Optional Manus ergonomics values keyed by type.

        Returns:
            Tuple of (left_target, right_target), each a 1-D array of 6 joint
            positions in radians, ordered by the shared Revo2 command contract.
        """
        ...


class RetargeterRegistry:
    """Factory / registry for retargeting algorithms.

    Algorithms are registered by a string name and can be instantiated from a
    YAML configuration file.
    """

    _registry: ClassVar[Dict[str, Callable[..., BaseRetargeter]]] = {}
    _registration_errors: ClassVar[Dict[str, BaseException]] = {}

    @classmethod
    def register(
        cls,
        name: str,
        builder: Callable[..., BaseRetargeter],
    ) -> None:
        """Register a retargeter under *name*."""
        cls._registry[name] = builder

    @classmethod
    def create(cls, name: str, config_path: Path, **kwargs) -> BaseRetargeter:
        """Instantiate a retargeter by *name* using *config_path*."""
        if name not in cls._registry:
            message = (
                f"Unknown retargeting algorithm '{name}'. "
                f"Available: {list(cls._registry.keys())}"
            )
            if cls._registration_errors:
                details = "; ".join(
                    f"{algorithm}: {type(exc).__name__}: {exc}"
                    for algorithm, exc in cls._registration_errors.items()
                )
                message += f". Import errors for unavailable built-ins: {details}"
            raise KeyError(message)
        return cls._registry[name](config_path, **kwargs)

    @classmethod
    def list_algorithms(cls) -> list:
        """Return a list of registered algorithm names."""
        return list(cls._registry.keys())


def _import_builtin(algorithm: str, module: str) -> None:
    """Import a built-in retargeter and remember why optional ones failed."""
    try:
        importlib.import_module(module)
    except Exception as exc:
        RetargeterRegistry._registration_errors[algorithm] = exc


# Import built-in retargeters so their registration hooks run.
_import_builtin(
    "dex",
    "manus_revo2_retarget.retargeters.dex_retargeter",
)
_import_builtin(
    "revo3_thumb",
    "manus_revo2_retarget.retargeters.revo3_thumb_retargeter",
)
_import_builtin(
    "joint_thumb",
    "manus_revo2_retarget.retargeters.joint_thumb_retargeter",
)
