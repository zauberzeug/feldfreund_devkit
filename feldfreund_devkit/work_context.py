from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING, NoReturn

from .robot_locator import RobotLocator

if TYPE_CHECKING:
    from .navigation.path_driver import PathDriver


@dataclass(frozen=True)
class WorkContext:
    """What a tool may use while it works: how to move, and where the robot is."""

    motion: PathDriver
    locator: RobotLocator


WorkFunction = Callable[[WorkContext], Awaitable[NoReturn]]
"""A tool's work loop: runs while a workable stretch is driven, cancelled at its end."""


async def never() -> NoReturn:
    """Idle until cancelled."""
    await asyncio.Event().wait()
    raise AssertionError('an Event that is never set was set')
