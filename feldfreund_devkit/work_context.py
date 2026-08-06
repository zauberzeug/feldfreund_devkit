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
    """What a tool may use while it works: how to move, and where the robot is.

    Run-scoped, so it carries only what a tool cannot own itself. Hardware, and anything a tool sets
    up for itself when it is readied, stay with the implement.
    """

    motion: PathDriver
    pose: PoseProvider


WorkFunction = Callable[[WorkContext], Awaitable[None]]
"""A tool's work loop. Runs while the robot drives a working stretch and is cancelled at its end,
so it must not return on its own -- see :func:`drive_and_work`."""


async def never() -> None:
    """Idle until cancelled.

    For a tool with nothing to do but stay alive for the stretch -- a mower, whose actuation runs
    from activation to deactivation, or a tool that only needs to act when the stretch ends.
    """
    await asyncio.Event().wait()
