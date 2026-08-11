from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING

from rosys.driving.pose_provider import PoseProvider

if TYPE_CHECKING:
    from .navigation.path_driver import PathDriver


@dataclass(frozen=True)
class WorkContext:
    """What a tool may use while it works: how to move, and where the robot is."""

    motion: PathDriver
    pose: PoseProvider


WorkFunction = Callable[[WorkContext], Awaitable[None]]
"""A tool's work loop: runs while a workable stretch is driven, cancelled at its end.

It must not return on its own; returning would look like the stretch being over.
"""


async def never() -> None:
    """Idle until cancelled."""
    await asyncio.Event().wait()
