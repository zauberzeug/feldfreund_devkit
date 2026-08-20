from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING, NoReturn, Protocol

from rosys.driving.pose_provider import PoseProvider
from rosys.geometry import Velocity

if TYPE_CHECKING:
    from .navigation.path_driver import PathDriver


class PoseVelocityProvider(PoseProvider, Protocol):
    """Provides where the robot is and how fast it is currently moving."""

    @property
    def velocity(self) -> Velocity:
        ...


@dataclass(frozen=True)
class WorkContext:
    """What a tool may use while it works: how to move, where the robot is, and how fast it moves."""

    motion: PathDriver
    pose: PoseVelocityProvider


WorkFunction = Callable[[WorkContext], Awaitable[NoReturn]]
"""A tool's work loop: runs while a workable stretch is driven, cancelled at its end."""


async def never() -> NoReturn:
    """Idle until cancelled."""
    await asyncio.Event().wait()
    raise AssertionError('an Event that is never set was set')
